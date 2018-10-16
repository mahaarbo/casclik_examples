#!/usr/bin/env python
"""This is the first example from the article 'Set-Based Tasks within the Singularity-Robust Multiple Task-Priority Inverse Kinematics Framework: General Formulation, Stability Analysis, and Experimental Results' in Frontiers in Robotics and AI by Signe Moe (2016), doi: 10.3389/frobt.2016.00016"""
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
    # Collision avoidance for two balls:
    ball_A_radius = 0.18
    ball_B_radius = 0.15
    ball_A_pos = cs.vertcat(0.4, -0.25, 0.33+0.2)
    ball_B_pos = cs.vertcat(0.4, 0.15, 0.33+0.2)
    ball_A_dist = p_fk(t, q) - ball_A_pos
    ball_B_dist = p_fk(t, q) - ball_B_pos

    # Desired points
    point1 = cs.vertcat(0.486, -0.066, 0.25+0.2)
    point2 = cs.vertcat(0.32, 0.37, 0.25+0.2)
    point1_dist = p_fk(t, q) - point1
    point2_dist = p_fk(t, q) - point2

    # Field of view
    a_des = cs.vertcat(1., 0., 0.)
    a_dist = R_fk(t, q)[:3, 2] - a_des
    fov_max = 0.2622

    ####################################################################
    # Setup constraints and skill
    ####################################################################
    # collision avoidance ball A
    colav_A_cnstr = cc.SetConstraint(
        label="colav_A",
        expression=cs.dot(ball_A_dist, ball_A_dist),
        set_min=ball_A_radius**2,
        priority=0,  # for pseudoinverse systems, this is most important
        constraint_type="hard",  # For QP/NLP/MPC
        gain=1.0  # For QP/NLP/MPC
    )

    # collision avoidance ball B
    colav_B_cnstr = cc.SetConstraint(
        label="colav_B",
        expression=cs.dot(ball_B_dist, ball_B_dist),
        set_min=ball_B_radius**2,
        priority=1,  # Second most important, etc
        constraint_type="hard",
        gain=1.0
    )

    # Moving to desired point1
    move_point1_cnstr = cc.EqualityConstraint(
        label="move_point1",
        expression=point1_dist,
        priority=2,
        constraint_type="soft",
        gain=0.3
    )

    # Moving to desired point2
    move_point2_cnstr = cc.EqualityConstraint(
        label="move_point2",
        expression=point2_dist,
        priority=2,
        constraint_type="soft",
        gain=0.3
    )

    # Trying to maintain field of view
    fov_cnstr = cc.SetConstraint(
        label="fov",
        expression=cs.dot(a_dist, a_dist),
        set_max=fov_max**2,
        set_min=0.0,
        gain=1.0,
        constraint_type="soft",
        priority=3
    )
    # List constraints for the two skills
    constraints_point1 = [move_point1_cnstr,
                          colav_A_cnstr,
                          colav_B_cnstr,
                          fov_cnstr]
    constraints_point2 = [move_point2_cnstr,
                          colav_A_cnstr,
                          colav_B_cnstr,
                          fov_cnstr]
    # Setup the first skill and print info
    skill_point1 = cc.SkillSpecification(
        label="move_point1",
        time_var=t,
        robot_var=q,
        constraints=constraints_point1
    )
    skill_point1.print_constraints()

    # Setup the second skill and print info
    skill_point2 = cc.SkillSpecification(
        label="move_point2",
        time_var=t,
        robot_var=q,
        constraints=constraints_point2
    )
    skill_point2.print_constraints()

    ####################################################################
    # Setup stop monitors
    ####################################################################
    # The controllers stop if we're within 2 cm of the desired point
    stop_point1 = cs.Function(
        "stop_point1",
        [t, q], [cs.norm_2(point1_dist) < 0.02]
    )

    stop_point2 = cs.Function(
        "stop_point2",
        [t, q], [cs.norm_2(point2_dist) < 0.02]
    )

    ####################################################################
    # Setup robot interfaces
    ####################################################################
    # Things for both:
    cntrllr_class = cc.PseudoInverseController  #cc.ReactiveQPController# 
    timestep = 1.0/50.0  # Default controller Hz
    casclik_joint_names = fk_dict["joint_names"]

    # Setup first
    robot_interface_skill1 = robintrfc.DefaultRobotInterface(
        skill_point1,
        timestep=timestep,
        cntrllr_class=cntrllr_class,
        casclik_joint_names=casclik_joint_names,
        monitors=[stop_point1],
        max_robot_vel_var=[1.5]*len(fk_dict["joint_names"]),
        min_robot_vel_var=[-1.5]*len(fk_dict["joint_names"]),
        #options={"converge_final_set_to_max": True}
    )

    # Setup second
    robot_interface_skill2 = robintrfc.DefaultRobotInterface(
        skill_point2,
        timestep=timestep,
        cntrllr_class=cntrllr_class,
        casclik_joint_names=casclik_joint_names,
        monitors=[stop_point2],
        max_robot_vel_var=[0.1]*len(fk_dict["joint_names"]),
        min_robot_vel_var=[-0.1]*len(fk_dict["joint_names"]),
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
        ball_A_mrkr = Marker(
            type=Marker.SPHERE,
            id=0,
            scale=Vector3(2*ball_A_radius,
                          2*ball_A_radius,
                          2*ball_A_radius),
            pose=Pose(
                Point(ball_A_pos[0],
                      ball_A_pos[1],
                      ball_A_pos[2]),
                Quaternion(0, 0, 0, 1)),
            header=Header(frame_id="world"),
            color=ColorRGBA(0.0, 0.75, 0.05, 1.0),
            lifetime=rospy.Duration(1.)
        )
        ball_B_mrkr = Marker(
            type=Marker.SPHERE,
            id=0,
            scale=Vector3(2*ball_B_radius,
                          2*ball_B_radius,
                          2*ball_B_radius),
            pose=Pose(
                Point(ball_B_pos[0],
                      ball_B_pos[1],
                      ball_B_pos[2]),
                Quaternion(0, 0, 0, 1)),
            header=Header(frame_id="world"),
            color=ColorRGBA(0.0, 0.75, 0.05, 1.0),
            lifetime=rospy.Duration(1.)
        )
        # Add them to the vizhandler
        add_marker(label="ball_A",
                   marker=ball_A_mrkr)
        add_marker(label="ball_B",
                   marker=ball_B_mrkr)

    ####################################################################
    # Start moving to the points
    ####################################################################
    rospy.loginfo("Starting robot_interface_skill1 in 1 second!")
    rospy.sleep(1)
    robot_interface_skill1.start()
    try:
        while not rospy.is_shutdown() and robot_interface_skill1.running:
            rospy.sleep(0.1)  # Check every 100 ms
    except rospy.ROSInterruptException:
        quit()
    rospy.loginfo("Stopped robot_interface_skill1")
    robot_interface_skill1.disconnect()
    robot_interface_skill2.start()
    try:
        while not rospy.is_shutdown() and robot_interface_skill2.running:
            rospy.sleep(0.1)  # Check every 100 ms
    except rospy.ROSInterruptException:
        quit()
    rospy.loginfo("Stopped robot_interface_skill2")
