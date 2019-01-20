#!/usr/bin/env python
"""This is a minimal example of using input_var. The controller is trying to make the end-effector position match the position of the input_marker. This is not true 
'tracking' as there is no time signal on input_var. The robot is, as with moe2016_example2, bound to a box. And there are both joint and cartesian velocity constraints."""
import rospy

# For waiting for robot
from sensor_msgs.msg import JointState

# For the input marker
from geometry_msgs.msg import Pose

# For the visualization
from visualization_msgs.msg import Marker
from py_viz_marker.srv import AddMarker
from geometry_msgs.msg import Quaternion, Point, Vector3
from std_msgs.msg import ColorRGBA, Header

# CasADi and CASCLIK things
import casclik as cc
import casadi as cs
import casclik_basics.robot_interface as robintrfc
from urdf2casadi import converter as xmlconv


if __name__ == "__main__":
    rospy.init_node("casclik_input_marker")
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
    # The position of the end-effector should follow the marker
    p_input = cs.MX.sym("p_input", 3)
    ee_dist = p_fk(t, q) - p_input

    # Wall avoidance:
    x_min, x_max = 0.1, 0.4
    y_min, y_max = -0.5, 0.4
    z_min, z_max = 0.3, 0.85

    ####################################################################
    # Setup constraints and skill
    ####################################################################
    # Tracking constraint
    track_cnstr = cc.EqualityConstraint(
        label="track_cnstr",
        expression=ee_dist,
        constraint_type="soft",
        priority=100)

    # Box constraint
    rospy.loginfo("TYPE X_MIN:"+str(type(x_min)))
    rospy.loginfo("TYPE X_MAX:"+str(type(x_max)))
    box_x_cnstr = cc.SetConstraint(
        label="box_x",
        expression=p_fk(t, q)[0],
        set_min=x_min,
        set_max=x_max,
        priority=0,
        constraint_type="hard")
    box_y_cnstr = cc.SetConstraint(
        label="box_y",
        expression=p_fk(t, q)[1],
        set_min=y_min,
        set_max=y_max,
        priority=1,
        constraint_type="hard")
    box_z_cnstr = cc.SetConstraint(
        label="box_z",
        expression=p_fk(t, q)[2],
        set_min=z_min,
        set_max=z_max,
        priority=2,
        constraint_type="hard")

    # Setup joint rate constraints
    joint_rate_cnstrs = []
    for q_idx in xrange(q.size()[0]):
        joint_rate_cnstrs += [cc.VelocitySetConstraint(
            label="dq"+str(q_idx),
            expression=q[q_idx],
            set_min=-0.1,
            set_max=0.1,
            gain=1.0)]
    # Setup cartesian velocity constraints
    cart_vel_cnstrs = [
        cc.VelocitySetConstraint(
            label="cartvel_x",
            expression=p_fk(t, q)[0],
            set_min=-0.5,
            set_max=0.5,
            gain=1.0),
        cc.VelocitySetConstraint(
            label="cartvel_y",
            expression=p_fk(t, q)[1],
            set_min=-0.5,
            set_max=0.5,
            gain=1.0),
        cc.VelocitySetConstraint(
            label="cartvel_z",
            expression=p_fk(t, q)[2],
            set_min=-0.5,
            set_max=0.5,
            gain=1.0)
    ]

    # All the constraints combined
    constraints = [track_cnstr,
                   box_x_cnstr, box_y_cnstr, box_z_cnstr]
    constraints += cart_vel_cnstrs
    constraints += joint_rate_cnstrs

    # Setup the skill
    skill = cc.SkillSpecification(
        label="track_input_position",
        time_var=t,
        robot_var=q,
        input_var=p_input,
        constraints=constraints
    )
    skill.print_constraints()

    ####################################################################
    # Setup Input mapping
    ####################################################################
    # The input is an interactive marker that publishes a geometry_msgs/Pose
    # We need to create a mapping function that takes this and makes it fit
    # the size and shape of our input_var
    def input_mapping(pose_msg, idx):
        return cs.np.array([pose_msg.position.x,
                            pose_msg.position.y,
                            pose_msg.position.z])
    input_topic_names = ["input_marker"]
    input_topic_msg_types = [Pose]

    ####################################################################
    # Setup Robot Interface
    ####################################################################
    cntrllr_class = cc.ReactiveQPController
    timestep = 1.0/50.0
    casclik_joint_names = fk_dict["joint_names"]

    robot_interface = robintrfc.DefaultRobotInterface(
        skill,
        timestep=timestep,
        cntrllr_class=cntrllr_class,
        casclik_joint_names=casclik_joint_names,
        max_robot_vel_var=[3.0]*len(fk_dict["joint_names"]),
        min_robot_vel_var=[-3.0]*len(fk_dict["joint_names"]),
        input_topic_names=input_topic_names,
        input_topic_msg_types=input_topic_msg_types,
        input_topic_mapping=input_mapping)

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
