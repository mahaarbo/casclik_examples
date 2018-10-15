#!/usr/bin/env python
import rospy

# Services
from py_viz_marker.srv import AddMarkerRequest, AddMarker

# Messages (Mainly for visualization)
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import JointState

# CasADi and CasCLIK things
import casclik as cc
import casadi as cs
import casclik_basics.robot_interface as robintrfc
from urdf2casadi import converter as xmlconv

# Others
import argparse

if __name__ == "__main__":
    ####################################################################
    # Handle arguments
    ####################################################################
    parser = argparse.ArgumentParser(description="Default robot interface"
                                     + " tester.")
    parser.add_argument("-namespace",
                        help="The robot namespace",
                        default="",
                        type=str)
    parser.add_argument("-root_link",
                        help="The root link from which FK is calculated.",
                        default="root",
                        type=str)
    parser.add_argument("-tip_link",
                        help="The tip link to which FK is calculated.",
                        default="floor_tool0",
                        type=str)
    parser.add_argument("-robot_description",
                        help="If the robot_description is not at "
                        + "/namespace/robot_description, use this to overload",
                        default="",
                        type=str)
    parser.add_argument("-rate",
                        help="The update rate of the control law. Used for timestep.",
                        default=50,
                        type=int)
    parser.add_argument("-ball_center",
                        help="Centers of the colav balls",
                        nargs=3,
                        default=[],
                        type=float,
                        action="append")
    parser.add_argument("-ball_radius",
                        help="Radii of the colav ball",
                        default=[],
                        nargs=1,
                        type=float,
                        action="append")
    parser.add_argument("-traj_center",
                        help="Center of the trajectory circle",
                        default=[-1.4, -1.078363, 1.1084],
                        type=float,
                        nargs=3)
    parser.add_argument("-traj_radius",
                        help="Radius of the trajectory circle",
                        default=0.3,
                        nargs=1,
                        type=float)
    parser.add_argument("-traj_omega",
                        help="Angular speed of the trajectory circle",
                        default=0.1,
                        nargs=1,
                        type=float)
    args, unknown_args = parser.parse_known_args()
    namespace = robintrfc.sanitize_namespace(args.namespace)

    rospy.init_node("test_default_robot_interface", anonymous=True)
    rospy.sleep(4)  # Wait for other things to start
    ####################################################################
    # Forward kinematics and core symbols
    ####################################################################
    robot_description = args.robot_description
    if robot_description == "":
        robot_description = namespace + "/robot_description"
    # Get the forward kinematics
    fk_dict = xmlconv.from_parameter_server(args.root_link,
                                            args.tip_link,
                                            robot_description)
    rospy.loginfo("--Got size(q)" + str(len(fk_dict["joint_names"])))
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
    rospy.loginfo("Waiting " + namespace + "/joint_states to start")
    rospy.wait_for_message(namespace+"/joint_states",
                           JointState,
                           timeout=None)

    # Prep position controller
    rospy.loginfo("Waiting for controller switch.")
    resp = robintrfc.switch_hw_controller("joint_position_controller",
                                          resources=fk_dict["joint_names"],
                                          ns=namespace)
    if resp.ok == 1:
        rospy.loginfo("Controller switched!")
    else:
        rospy.logerr("Controller not switched!")
        rospy.logerr("Cannot run without joint_position_controller.")
        quit()

    ####################################################################
    # Setup the problem expressions
    ####################################################################
    # Prep colav balls
    ball_centers = args.ball_center
    ball_radii = [lradius[0] for lradius in args.ball_radius]

    # Do we have the opportunity to visualize the balls?
    rospy.loginfo("Checking for visualizer.")
    try:
        rospy.wait_for_service(namespace+"/add_marker", timeout=1)
        viz_available = True
        add_marker = rospy.ServiceProxy(namespace+"/add_marker",
                                        AddMarker)
        rospy.loginfo("--Visualizer found!")
    except rospy.ROSException:
        rospy.loginfo("If you run viz_handler with " + namespace + " as the"
                      + " namespace, we can visualize the colav balls.")
        viz_available = False

    # Ensure correct numbers
    if len(ball_radii) != len(ball_centers):
        raise IndexError("Provide as many radii as centers for the balls.")

    # Setup colav balls
    rospy.loginfo("Setting up colav balls")
    ball_cnstrs = []
    idx = 0
    rospy.loginfo("--len(ball_centers):"+str(len(ball_centers)))
    for center in ball_centers:
        p_err = p_fk(t, q) - cs.MX([center[0], center[1], center[2]])
        dist = cs.dot(p_err, p_err) - ball_radii[idx]*ball_radii[idx]
        ball_cnstrs.append(cc.SetConstraint("ball"+str(idx),
                                            dist,
                                            gain=1.0,
                                            set_min=0.0,
                                            set_max=1e20,
                                            constraint_type="hard",
                                            priority=idx))  # For Pseud.Inv.
        if viz_available:
            ball_mrkr = Marker(
                type=Marker.SPHERE,
                id=0,
                scale=Vector3(2*ball_radii[idx],
                              2*ball_radii[idx],
                              2*ball_radii[idx]),
                pose=Pose(Point(center[0],
                                center[1],
                                center[2]),
                          Quaternion(0,
                                     0,
                                     0,
                                     1)),
                header=Header(frame_id="world"),
                color=ColorRGBA(cs.np.random.rand(),
                                cs.np.random.rand(),
                                cs.np.random.rand(),
                                1.0),
                lifetime=rospy.Duration(1.))
            add_marker(label="ball"+str(idx),
                       marker=ball_mrkr)
        idx += 1
    # Finish setting up colav balls
    rospy.loginfo("Setting up trajectory")

    # Setup desired trajectory
    traj_radius = args.traj_radius
    traj_center = args.traj_center
    traj_omega = args.traj_omega
    p_des = cs.vertcat(
        traj_radius*cs.cos(traj_omega*t) + traj_center[0],
        traj_radius*cs.sin(traj_omega*t) + traj_center[1],
        traj_center[2])
    traj_cnstr = cc.EqualityConstraint(
        "traj",
        p_des-p_fk(t, q),
        gain=1.0,
        constraint_type="soft",
        priority=idx+1)

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
    cart_vel_cnstrs = [cc.VelocitySetConstraint(label="cartvel_x",
                                                expression=p_fk(t, q)[0],
                                                set_min=-0.5,
                                                set_max=0.5,
                                                gain=1.0),
                       cc.VelocitySetConstraint(label="cartvel_y",
                                                expression=p_fk(t, q)[1],
                                                set_min=-0.5,
                                                set_max=0.5,
                                                gain=1.0),
                       cc.VelocitySetConstraint(label="cartvel_z",
                                                expression=p_fk(t, q)[2],
                                                set_min=-0.5,
                                                set_max=0.5,
                                                gain=1.0)]
    # The constraints we will use:
    constraints = [traj_cnstr]
    constraints += ball_cnstrs
    constraints += cart_vel_cnstrs
    #constraints += joint_rate_cnstrs

    # Setup skill specification
    skill = cc.SkillSpecification("avoid_balls",
                                  time_var=t,
                                  robot_var=q,
                                  constraints=constraints)
    rospy.loginfo(str(fk_dict["joint_names"]))
    robot_interface = robintrfc.DefaultRobotInterface(
        skill,
        timestep=1.0/args.rate,
        cntrllr_class=cc.ReactiveQPController,
        casclik_joint_names=fk_dict["joint_names"],
        namespace=namespace)

    # Start the skill after 5 seconds
    rospy.sleep(5)
    robot_interface.start()
    try:
        while not rospy.is_shutdown():
            rospy.sleep(999)
    except rospy.ROSInterruptException:
        pass
