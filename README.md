# CASCLIK EXAMPLES
This is a metapackage containing some example usage of CASCLIK.

## casclik_basics
This contains a default robot interface which is a python class that just tries to keep track of current `robot_var`, current `virtual_var`, and sends commands to a robot using `joint_position_controller` and callback on `joint_states`.

## casclik_tests
This contains simple tests of CASCLIK with robots. They are generally based on examples from constraint-based robotics articles, and some simple home-made examples. Many of these examples require [`py_viz_marker`](https://github.com/mahaarbo/py_viz_marker) which is a simple rviz marker adder.

1. moe2016_example1/2 requires the UR packages (in particular: `ur_description` and `ur_gazebo`), `gazebo`, and `py_viz_marker`.
2. aertbelien2013_example requires the PR2 packages. First test if `roslaunch pr2_gazebo pr2_empty_world.launch` works. If it doesn't, try [kinetic_pr2](https://github.com/RichardKelley/kinetic_pr2) if you're working with kinetic kame. We recommend running the PR2 examples in a dedicated workspace. Remember to state that we're simulating: `export ROBOT=sim`.


### Troubleshooting:
1. PR2 connection issues: remember to run `export ROBOT=sim` before running the pr2 example.
2. Moe2016_example1/2 not working after running pr2 example: the `kinetic_pr2` workspace contains a `controller_manager` that seems to override the standard installation. Deleting `./devel/` and `./build` fixes this issue.
3. If you see mentions of missing position controllers, try `sudo apt-get install ros-VERSION-position-controllers`
