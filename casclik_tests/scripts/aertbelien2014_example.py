#!/usr/bin/env python
import rospy

# For visualization
from py_viz_marker.srv import AddMarker
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import ColorRGBA, Header

# For waiting for robot
from sensor_msgs.msg import JointState

# CasADi and CASCLIK things
import casclik as cc
import casadi as cs
import casclik_basics.robot_interface as robintrfc
from urdf2casadi import converter as xmlconv

if __name__ == "__main__":
    rospy.init_node("aertbelien_example", anonymous=True)
    rospy.sleep(5)  # Wait for other things to start

    ####################################################################
    # Forward kinematics and core symbols
    ####################################################################
    # Setup time
    t = cs.MX.sym("t")
    rospy.loginfo("--Reading right arm")
    # Right arm
    fk_dict_r = xmlconv.from_parameter_server(
        root="base_link",
        tip="r_gripper_tool_frame",
        key="/robot_description"
    )
    rospy.loginfo("--Right arm size(q): " + str(len(fk_dict_r)))
    # Right arm robot_var
    q_r = cs.MX.sym("q_r", len(fk_dict_r["joint_names"]))
    T_r_fk = fk_dict_r["T_fk"]
    p_r_fk = cs.Function("p_r_fk", [t, q_r], [T_r_fk(q_r)[:3, 3]])
    R_r_fk = cs.Function("R_r_fk", [t, q_r], [T_r_fk(q_r)[:3, :3]])

    # Left arm
    rospy.loginfo("--Reading left arm")
    fk_dict_l = xmlconv.from_parameter_server(
        root="base_link",
        tip="l_gripper_tool_frame",
        key="/robot_description"
    )
    rospy.loginfo("--Left arm size(q): " + str(len(fk_dict_l)))
    # Left arm robot_var
    q_l = cs.MX.sym("q_l", len(fk_dict_l["joint_names"]))
    T_l_fk = fk_dict_l["T_fk"]
    p_l_fk = cs.Function("p_l_fk", [t, q_l], [T_l_fk(q_l)[:3, 3]])
    R_l_fk = cs.Function("R_l_fk", [t, q_l], [T_l_fk(q_l)[:3, :3]])

    # Head
    rospy.loginfo("--Reading head")
    fk_dict_h = xmlconv.from_parameter_server(
        root="base_link",
        tip="narrow_stereo_optical_frame",
        key="/robot_description"
    )
    rospy.loginfo("--Head size(q): " + str(len(fk_dict_h)))
    # Head robot_var
    q_h = cs.MX.sym("q_h", len(fk_dict_h["joint_names"]))
    T_h_fk = fk_dict_h["T_fk"]
    p_h_fk = cs.Function("p_h_fk", [t, q_h], [T_h_fk(q_h)[:3, 3]])
    R_h_fk = cs.Function("R_h_fk", [t, q_h], [T_h_fk(q_h)[:3, :3]])

    # Full robot var is a combination of the three vectors
    q = cs.vertcat(q_h, q_l, q_r)
    joint_names = fk_dict_h["joint_names"]
    joint_names += fk_dict_l["joint_names"]
    joint_names += fk_dict_r["joint_names"]
    rospy.loginfo("--Joints involved: "+str(joint_names))

    ####################################################################
    # Initialize robot
    ####################################################################
    # Wait for robot to start
    rospy.loginfo("Waiting for /joint_states to start")
    rospy.wait_for_message(
        "/joint_states",
        JointState,
        timeout=None)

    # Prep position controller
    rospy.loginfo("Waiting for controller switch.")
    resp = robintrfc.switch_hw_controller(  # sane switching of controllers
        "joint_position_controller",
        resources=joint_names,
        namespace="")

    if resp.ok == 1:
        rospy.loginfo("Controller switched!")
    else:
        rospy.logerr("Controller not switched!")
        rospy.logerr("Cannot run without joint_position_controller.")
        quit()

    ####################################################################
    # Setup the problem expressions
    ####################################################################
    # Circle tracking for the right arm
    radius = 0.15
    omega = 0.5
    circ_center_r = cs.vertcat(0.55,
                               -0.1,
                               0.9453274)
    circ_traj_r = radius*cs.vertcat(cs.cos(omega*t),
                                    cs.sin(omega*t),
                                    0.) + circ_center_r
    traj_dist_r = circ_traj_r - p_r_fk(t, q_r)

    # Circle tracking for the left arm
    circ_center_l = cs.vertcat(0.55,
                               0.1,
                               0.9453274)
    path_time = cs.MX.sym("path_time")  # Virtual variable
    circ_path_l = radius*cs.vertcat(cs.cos(omega*path_time),
                                    cs.sin(omega*path_time),
                                    0.) + circ_center_l
    path_dist_l = circ_path_l - p_l_fk(t, q_l)
    path_time_follow = 0.75*t - path_time

    # Look at right arm head
    look_var = cs.MX.sym("look_var")  # Virtual variable
    look_vector = cs.mtimes(R_h_fk(t, q_h),
                            cs.vertcat(0,
                                       look_var,
                                       0))
    look_dist = look_vector - p_r_fk(t, q_r)

    # Don't let the hands collide
    hand_dist = cs.norm_2(p_r_fk(t, q_r)[:2] - p_l_fk(t, q_l)[:2])
    hand_dist2 = p_l_fk(t, q_l)[1]
    minimum_hand_dist = 0.2

    # Combined virtual var:
    virtual_var = cs.vertcat(path_time,
                             look_var)
    ####################################################################
    # Setup constraints and skill
    ####################################################################
    # Trajectory tracking - Right arm
    traj_r_cnstr = cc.EqualityConstraint(
        label="traj_r_cnstr",
        expression=traj_dist_r,
        priority=1,
        constraint_type="soft",
        gain=4.,
        slack_weight=100.
    )

    # Path following - Left arm
    path_l_cnstr = cc.EqualityConstraint(
        label="path_l_cnstr",
        expression=path_dist_l,
        priority=2,
        constraint_type="soft",
        gain=4.,
        slack_weight=1.
    )

    # Path timing - virtual var
    path_timing_cnstr = cc.EqualityConstraint(
        label="path_timing_cnstr",
        expression=path_time_follow,
        priority=3,
        constraint_type="soft",
        gain=0.0,  # See the article
        slack_weight=1e-10
    )
    no_back_on_path_cnstr = cc.VelocitySetConstraint(
        label="no_back_on_path",
        expression=path_time,
        priority=4,
        constraint_type="hard",
        set_min=0.0,
        set_max=0.75
    )
    # Look at hand - Head
    look_at_hand_cnstr = cc.EqualityConstraint(
        label="look_at_hand_cnstr",
        expression=look_dist,
        priority=4,
        constraint_type="soft",
        gain=4.
    )

    # Hand collision - Left and right arm
    hand_collision_cnstr = cc.SetConstraint(
        label="hand_collision_cnstr",
        expression=hand_dist2,
        set_min=p_r_fk(t, q_r)[1],
        priority=0,
        constraint_type="hard",
        gain=10.
    )
    # The upper and lower constraints
    arm_r_limits_cnstr = cc.SetConstraint(
        label="arm_r_limits_cnstr",
        expression=q_r,
        set_min=cs.np.array(fk_dict_r["lower"]),
        set_max=cs.np.array(fk_dict_r["upper"])
    )
    arm_l_limits_cnstr = cc.SetConstraint(
        label="arm_l_limits_cnstr",
        expression=q_l,
        set_min=cs.np.array(fk_dict_l["lower"]),
        set_max=cs.np.array(fk_dict_l["upper"])
    )

    # List the constraints
    constraints = [arm_l_limits_cnstr,
                   arm_r_limits_cnstr,
                   traj_r_cnstr,
                   path_l_cnstr,
                   path_timing_cnstr,
                   no_back_on_path_cnstr,
                   look_at_hand_cnstr,
                   hand_collision_cnstr]

    # Setup the skill and print info
    skill = cc.SkillSpecification(
        label="motionskill",
        time_var=t,
        robot_var=q,
        virtual_var=virtual_var,
        constraints=constraints
    )
    skill.print_constraints()

    ####################################################################
    # Setup stop monitors
    ####################################################################
    # The controller stops if we've run for 80 seconds
    timer80s = cs.Function(
        "timer80s",
        [t, q, virtual_var], [t > 80.0]
    )

    ####################################################################
    # Setup robot interfaces
    ####################################################################
    # WARNING: The model predictive contorller uses around 0.03 seconds to
    # solve this problem with default settings, while the update rate of the
    # joint state publisher and joint position controller are 50 hz. This means
    # that we cannot run this problem with the MPC as is.
    cntrllr_class = cc.ReactiveQPController
    timestep = 1.0/50.0  # Default controller Hz
    casclik_joint_names = joint_names
    # Setup robot interface
    robot_interface = robintrfc.DefaultRobotInterface(
        skill,
        timestep=timestep,
        cntrllr_class=cntrllr_class,
        casclik_joint_names=casclik_joint_names,
        monitors=[timer80s],
        max_robot_vel_var=[0.3]*len(joint_names),
        min_robot_vel_var=[-0.3]*len(joint_names)
    )
    ####################################################################
    # Start following trajectory
    ####################################################################
    rospy.loginfo("Starting robot_interface in 1 second!")
    rospy.sleep(1)
    robot_interface.start()
    try:
        while not rospy.is_shutdown() and robot_interface.running:
            rospy.sleep(0.1)  # Check every 100 ms
    except rospy.ROSInterruptException:
        quit()
    rospy.loginfo("Stopped robot_interface because: "
                  + robot_interface.stop_reason)
    robot_interface.disconnect()
