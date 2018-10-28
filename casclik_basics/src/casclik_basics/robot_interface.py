import rospy

# Controller services
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest

# Command and sensor messages
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# CasCLIK, data, and thread safety
import casclik as cc
import numpy as np
from threading import Lock


####################################
# General functions useful anywhere
####################################


def sanitize_namespace(ns):
    """Tries to fix / issues on namespace."""
    if ns != "":
        if ns[0] == "/":
            ns = ns[1:]
        if ns[-1] == "/":
            ns = ns[:-1]
        ns = "/"+ns
    else:
        ns = ""
    return ns


def switch_hw_controller(desired_controller, resources, ns="", timeout=None):
    """Function to switch hw controller when there can be resource conflict."""
    rospy.wait_for_service(ns + "/controller_manager/list_controllers",
                           timeout=None)
    rospy.loginfo("wait for switch"+ns+"/controller_manager/switch_controller")
    rospy.wait_for_service(ns + "/controller_manager/switch_controller",
                           timeout=None)
    ls = rospy.ServiceProxy(ns + "/controller_manager/list_controllers",
                            ListControllers)
    sw = rospy.ServiceProxy(ns + "/controller_manager/switch_controller",
                            SwitchController)

    # Find controllers that control same resources
    hw_controllers = ls()
    stop_cntrllrs = []
    for cntrllr in hw_controllers.controller:
        if cntrllr.state == "running":
            if cntrllr.name == desired_controller:
                return SwitchControllerResponse(ok=True)
            for resource in resources:
                try:
                    # Kinetic Kame and above
                    for claimed_obj in cntrllr.claimed_resources:
                        if resource in claimed_obj.resources:
                            if cntrllr.name not in stop_cntrllrs:
                                stop_cntrllrs.append(cntrllr.name)
                except AttributeError:
                    # Older versions of ROS
                    if resource in cntrllr.resources:
                        if cntrllr.name != desired_controller:
                            if cntrllr.name not in stop_cntrllrs:
                                stop_cntrllrs.append(cntrllr.name)
    return sw([str(desired_controller)], stop_cntrllrs,
              SwitchControllerRequest.STRICT)


####################################
# Default Robot callback interface
####################################
class DefaultRobotInterface(object):
    """A default robot interface that runs as a callback routine.  The

    robot is controlled by reading /namespace/joint_states, and
    sending commands on /namespace/joint_position_controller/command.

    Internalizes the virtual_var so that you don't have to. Uses
    rospy.get_time to try to ensure that it used the ``real'' timestep
    for integration.

    Note: the ordering of the joints may differ between the ROS messages
    and what was used in CASCLIK. To circumvent this, we require the
    joint_names list used in CASCLIK so that we can find the mapping
    from CASCLIK to ROS.

    As not all solvers have support for initial problem solving yet, we
    also provide the option of specifiying an initial virtual variable.

    Monitors can be added and are casadi functions of time_var,
    robot_var, and/or virtual_var that will stop the controller if any of the
    conditions are true. Note: it will stop on the next timestep, and as
    such it is not at all as safe as eTaSL's approach. This is an ad-hoc
    approach to allowing stopping of controllers and should be treated
    as such.

    Args:
        skill_spec (cc.SkillSpecification): Skill specification
        timestep (float): control interval of speeds.
        casclik_joint_names (list): List of joint names as used in casclik.
        cntrllr_class (): CASCLIK controller, defaults to ReactiveQPController.
        monitors (list): casclik functions of time and robotvar to stop us.
        max_robot_vel_var (list, cs.np.array): Enforced through saturation.
        min_robot_vel_var (list, cs.np.array): Enforced through saturation.
        namespace (str): namespace of the robot. Defaults to None.
        virtual_var0 (cs.np.array): Initial virtual var. Defaults to None.
        options (dict): options passed to the controller.
        """
    def __init__(self, skill_spec,
                 timestep,
                 casclik_joint_names,
                 cntrllr_class=None,
                 namespace="",
                 monitors=[],
                 max_robot_vel_var=[],
                 min_robot_vel_var=[],
                 cost_expr=None,
                 virtual_var0=None,
                 options=None):
        # Sanitize namespace
        ns = sanitize_namespace(namespace)

        # Setup controller
        if cntrllr_class is None:
            cntrllr_class = cc.ReactiveQPController
        if cost_expr is None:
            self.cntrllr = cntrllr_class(skill_spec=skill_spec,
                                         options=options)
        else:
            self.cntrllr = cntrllr_class(skill_spec=skill_spec,
                                         cost_expr=cost_expr,
                                         options=options)
        self.cntrllr.setup_solver()
        self.cntrllr.setup_problem_functions()

        # Initial message variables
        self.is_first = True
        self.joint_names = casclik_joint_names

        # Learn mapping from casclik to commanded positions
        commanded_ordering = rospy.get_param(
            ns+"/joint_position_controller/joints")
        self.command_remap = []
        for jname in casclik_joint_names:
            self.command_remap.append(commanded_ordering.index(jname))
        # Sane stop and start
        self.running = False
        self.stop_reason = ""

        # Setup local variables
        self.timestep = timestep  # is used for command
        self.initial_time = rospy.get_time()
        self.previous_time = rospy.get_time()
        self.current_robot_var = np.zeros(skill_spec.n_robot_var)
        if skill_spec.virtual_var is None:
            self.current_virtual_var = None
        else:
            nvirt = skill_spec.n_virtual_var
            if virtual_var0 is None:
                self.current_virtual_var = np.zeros(nvirt)
            else:
                self.current_virtual_var = virtual_var0
                self.current_virtual_vel_var = np.zeros(nvirt)
        if options is None:
            options = {"open_loop": False}
        self.options = options
        self.monitors = monitors
        self.max_robot_vel_var = max_robot_vel_var
        self.min_robot_vel_var = min_robot_vel_var
        self.stop_reason = "initialized"
        self.lock = Lock()  # Callback may require semaphores.
        # Setup connections and callback
        self.sub = rospy.Subscriber(ns+"/joint_states",
                                    JointState,
                                    self.callback_control)
        self.pub = rospy.Publisher(ns+"/joint_position_controller/command",
                                   Float64MultiArray,
                                   queue_size=2)

    @property
    def options(self):
        """Get or set the options. Passed to the controller class.
        """
        return self._options

    @options.setter
    def options(self, opt):
        if opt is None:
            opt = {}
        if "open_loop" not in opt:
            opt["open_loop"] = False
        self._options = opt

    def callback_control(self, msg):
        with self.lock:
            if self.running:
                if self.is_first:
                    # First message is used to initialize
                    self.is_first = False
                    # We have to learn how the publisher has the joints ordered
                    published_ordering = msg.name
                    casclik_ordering = self.joint_names
                    publish_remap = []
                    for jname in casclik_ordering:
                        publish_remap.append(published_ordering.index(jname))
                    self.publish_remap = publish_remap
                    # Update values with remap
                    for curr_idx, new_idx in enumerate(self.publish_remap):
                        self.current_robot_var[curr_idx] = msg.position[new_idx]
                    return

                # Prepare for solving
                if not self.options["open_loop"]:
                    # Remap commanded ordering to casclik ordering
                    for curr_idx, new_idx in enumerate(self.publish_remap):
                        self.current_robot_var[curr_idx] = msg.position[new_idx]
                # Figure out time and timestep estimate (assuming next timestep
                # is equal current)
                current_time = rospy.get_time() - self.initial_time
                actual_dt = current_time - self.previous_time

                if self.current_virtual_var is None:
                    res = self.cntrllr.solve(time_var=current_time,
                                             robot_var=self.current_robot_var)
                    robot_vel_var = res[0].toarray()[:, 0]
                else:
                    dv = self.current_virtual_vel_var
                    self.current_virtual_var += actual_dt*dv
                    res = self.cntrllr.solve(time_var=current_time,
                                             robot_var=self.current_robot_var,
                                             virtual_var=self.current_virtual_var)
                    robot_vel_var = res[0].toarray()[:, 0]
                    self.current_virtual_vel_var = res[1].toarray()[:, 0]
                # Some solvers fail silently. Ensure that we don't send failed
                # commands:
                if np.isnan(robot_vel_var).any():
                    self.running = False
                    self.is_first = True
                    self.stop_reason = "solver_failure_nan"
                    return
                # Saturate the robot velocities if necesasry:
                for rob_idx, max_rate in enumerate(self.max_robot_vel_var):
                    robot_vel_var[rob_idx] = min(robot_vel_var[rob_idx],
                                                 max_rate)
                for rob_idx, min_rate in enumerate(self.min_robot_vel_var):
                    robot_vel_var[rob_idx] = max(robot_vel_var[rob_idx],
                                                 min_rate)
                # Iterate
                self.current_robot_var += robot_vel_var*self.timestep
                # Initialize command
                command = Float64MultiArray()
                command.data = np.zeros(len(self.joint_names))
                # Remap urdf ordering to commanded ordering before publishing
                for curr_idx, new_idx in enumerate(self.command_remap):
                    command.data[new_idx] = self.current_robot_var[curr_idx]
                self.pub.publish(command)
                self.previous_time = current_time
                # Check if we're violating any monitors and stop.
                for monit_func in self.monitors:
                    if self.current_virtual_var is not None:
                        monit_res = monit_func(current_time,
                                               self.current_robot_var,
                                               self.current_virtual_var)
                    else:
                        monit_res = monit_func(current_time,
                                               self.current_robot_var)
                    if monit_res != 0:
                        self.running = False
                        self.is_first = True
                        self.stop_reason = monit_func.name()

    def start(self):
        """Indicates to the callback function to start responding to
        joint messages."""
        with self.lock:
            self.running = True

    def stop(self, reason=""):
        """Indicates to the callback function to stop responding to
        joint messages."""
        with self.lock:
            self.running = False
            self.is_first = True
            self.stop_reason = reason

    def disconnect(self):
        """Disconnect from the suscribed and published topics. The robot
        interface must be reinitialized to start again."""
        self.stop("disconnect")
        with self.lock:
            self.sub.unregister()
            self.pub.unregister()
