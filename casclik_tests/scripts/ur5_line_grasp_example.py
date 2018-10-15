#!/usr/bin/env python
import rospy

# Messages for waiting for joint_state
from sensor_msgs.msg import JointState

# CasADi and CASCLIK things
import casclik as cc
import casadi as cs
import casclik_basics.robot_interface as robintrfc
from urdf2casadi import converter as xmlconv

if __name__ == "__main__":
    rospy.init_node("ur5_line_grasp_example", anonymous=True)
    rospy.sleep(4)  # Wait for other things to start

    ####################################################################
    # Forward kinematics and core symbols
    ####################################################################
    # Get the forward kinematics
    #fk_dict = xmlconv.from_parameter_server(
    #    root="world",
    #    tip="tool0",
    #    key="/robot_description")
    fk_dict = xmlconv.from_file(
        "world",
        "tool0",
        "/home/mathia/Programming/python/casclik/examples/notebooks/urdf/ur5.urdf"
    )
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
    # Desired point
    p_des = [0.3,
             0.3,
             0.4]
    # Position of tool0 should move to p_des
    p_dist = p_fk(t, q) - p_des

    # Normal vector from point
    n_vector = [0.0,
                1/cs.sqrt(2),
                1/cs.sqrt(2)]
    # z axis in frame and normal vector should be opposite
    z_align_dist = R_fk(t, q)[:3, 2] + n_vector

    ####################################################################
    # Setup constraints and skill
    ####################################################################
    # Align and position as equal constraint, but align faster
    dist_cnstr = cc.EqualityConstraint(
        label="dist_cnstr",
        expression=p_dist,
        gain=1.0,
        constraint_type="soft"
    )
    align_cnstr = cc.EqualityConstraint(
        label="align_cnstr",
        expression=z_align_dist,
        gain=5.0,
        constraint_type="soft"
    )
    
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
    # Don't let the shoulder go too low
    shoulder_cnstr = cc.SetConstraint(
        label="shoulder_cnstr",
        expression=q[1],
        set_max=-cs.np.pi/2+cs.np.pi/4,
        set_min=-cs.np.pi/2-cs.np.pi/4
    )
    constraints = [dist_cnstr, align_cnstr, shoulder_cnstr]
    constraints += cart_vel_cnstrs
    constraints += joint_rate_cnstrs
    # Prepare skill

    skill = cc.SkillSpecification(
        "align_approach",
        time_var=t,
        robot_var=q,
        constraints=constraints
    )
    skill.print_constraints()
    
    ####################################################################
    # Setup stop monitor and robot_interface
    ####################################################################
    # Stop if we're close enough
    stop_monitor = cs.Function(
        "close_enough",  # reason why we're stopping
        [t, q], [cs.norm_2(p_dist) < 0.005])
    # Setup robot interface
    robot_interface = robintrfc.DefaultRobotInterface(
        skill,
        timestep=1.0/50.0,  # default controller Hz
        cntrllr_class=cc.ReactiveQPController,
        casclik_joint_names=fk_dict["joint_names"],
        monitors=[stop_monitor],
    )
    # Start the skill after 5 seconds
    rospy.sleep(5)
    robot_interface.start()

    # Handle ROSInterrupt cleanly:
    try:
        while not rospy.is_shutdown() and robot_interface.running:
            rospy.sleep(1)
        rospy.loginfo("Stopped because: "+robot_interface.stop_reason)
    except rospy.ROSInterruptException:
        pass
