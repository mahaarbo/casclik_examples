#!/usr/bin/env python
"""This is the second example from the article 'Set-Based Tasks within the Singularity-Robust Multiple Task-Priority Inverse Kinematics Framework: General Formulation, Stability Analysis, and Experimental Results' in Frontiers in Robotics and AI by Signe Moe (2016), doi: 10.3389/frobt.2016.00016 with minor modifications in the placement to fit our reference frames."""
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
    rospy.init_node("moe2016_example1", anonymous=True)
    rospy.sleep(5)  # Wait for other things to start

    ####################################################################
    # Forward kinematics and core symbols
    ####################################################################
    # Get the forward kinematics
    fk_dict = xmlconv.from_parameter_server(
        root="world",
        tip="tool0",
        key="/robot_description")
    rospy.loginfo("--Got size(q):" + str(len(fk_dict["joint_names"])))

    # Setup time and robot_var
    t = cs.MX.sym("t")
    q = cs.MX.sym("q", len(fk_dict["joint_names"]))

    # Functions for end-effector things (casadi functions of q)
    T_fk = fk_dict["T_fk"]
    p_fk = cs.Function("p_fk", [t, q], [T_fk(q)[:3, 3]])
    R_fk = cs.Function("R_fk", [t, q], [T_fk(q)[:3, :3]])

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
        resources=fk_dict["joint_names"],
        ns="")

    if resp.ok == 1:
        rospy.loginfo("Controller switched!")
    else:
        rospy.logerr("Controller not switched!")
        rospy.logerr("Cannot run without joint_position_controller.")
        quit()

    ####################################################################
    # Setup the problem expressions
    ####################################################################
    # Wall avoidance:
    x_min, x_max = 0.1, 0.5
    y_min, y_max = -0.5, 0.4
    z_min, z_max = 0.3, 0.85

    # Desired trajectory
    path_des = cs.vertcat(0.5*cs.sin(0.1*t)*cs.sin(0.1*t) + 0.2,
                          0.5*cs.cos(0.1*t)+0.25*cs.sin(0.1*t),
                          0.5*cs.sin(0.1*t)*cs.cos(0.1*t) + 0.7)
    ####################################################################
    # Setup constraints and skill
    ####################################################################
    # collision avoidance
    colav_x_cnstr = cc.SetConstraint(
        label="colav_x",
        expression=p_fk(t, q)[0],
        set_min=x_min,
        set_max=x_max,
        priority=0,
        constraint_type="hard"
    )
    colav_y_cnstr = cc.SetConstraint(
        label="colav_y",
        expression=p_fk(t, q)[1],
        set_min=y_min,
        set_max=y_max,
        priority=1,
        constraint_type="hard"
    )
    colav_z_cnstr = cc.SetConstraint(
        label="colav_z",
        expression=p_fk(t, q)[2],
        set_min=z_min,
        set_max=z_max,
        priority=2,
        constraint_type="hard"
    )
    # Tracking trajectory
    path_cnstr = cc.EqualityConstraint(
        label="move_point2",
        expression=p_fk(t, q) - path_des,
        priority=3,
        constraint_type="soft",
        gain=0.15
    )

    # List constraints
    constraints = [colav_x_cnstr,
                   colav_y_cnstr,
                   colav_z_cnstr,
                   path_cnstr]
    # Setup the skill and print info
    skill = cc.SkillSpecification(
        label="box_move",
        time_var=t,
        robot_var=q,
        constraints=constraints
    )
    skill.print_constraints()

    ####################################################################
    # Setup stop monitors
    ####################################################################
    # The controller stop if we've run for 80 seconds
    timer80s = cs.Function(
        "timer80s",
        [t, q], [t > 80.0]
    )

    ####################################################################
    # Setup robot interfaces
    ####################################################################
    # Things for both:
    cntrllr_class = cc.ReactiveQPController  #cc.PseudoInverseController  #  
    timestep = 1.0/50.0  # Default controller Hz
    casclik_joint_names = fk_dict["joint_names"]

    # Setup first
    robot_interface = robintrfc.DefaultRobotInterface(
        skill,
        timestep=timestep,
        cntrllr_class=cntrllr_class,
        casclik_joint_names=casclik_joint_names,
        monitors=[timer80s],
        max_robot_vel_var=[1.5]*len(fk_dict["joint_names"]),
        min_robot_vel_var=[-1.5]*len(fk_dict["joint_names"]),
        #options={"converge_final_set_to_max": True}
    )

    ####################################################################
    # Setup Visualizaton
    ####################################################################
    rospy.loginfo("checking for visualizer")
    try:
        rospy.wait_for_service("/add_marker", timeout=1)
        viz_available = True
        add_marker = rospy.ServiceProxy("/add_marker",
                                        AddMarker)
        rospy.loginfo("--visualizer found!")
    except rospy.ROSException:
        rospy.loginfo("If you run vizhandler we can visualize the balls")
        viz_available = False
    if viz_available:
        # Define the marker messages
        box_mrkr = Marker(
            type=Marker.CUBE,
            id=0,
            scale=Vector3(x_max - x_min,
                          y_max - y_min,
                          z_max - z_min),
            pose=Pose(
                Point((x_max + x_min)/2,
                      (y_max + y_min)/2,
                      (z_max + z_min)/2),
                Quaternion(0, 0, 0, 1)),
            header=Header(frame_id="world"),
            color=ColorRGBA(0.0, 0.75, 0.05, 0.5),
            lifetime=rospy.Duration(1.)
        )
        # Add them to the vizhandler
        add_marker(label="box",
                   marker=box_mrkr)
        line_mrkr = Marker(
            type=Marker.LINE_STRIP,
            id=1,
            scale=Vector3(0.01, 1., 1.),
            action=Marker.ADD)
        line_mrkr.color.r = 1.0
        line_mrkr.color.a = 1.0
        line_mrkr.header.frame_id = "world"
        fpath_des = cs.Function("fpath_des", [t], [path_des])
        points = []
        npoints = 40
        for i in range(npoints):
            des_p = fpath_des(2*cs.pi*10*i/(npoints-1)).toarray()[:, 0]
            points += [Point(des_p[0], des_p[1], des_p[2])]
        line_mrkr.points = points
        add_marker(label="desired_path",
                   marker=line_mrkr)

    ####################################################################
    # Start moving to the points
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
