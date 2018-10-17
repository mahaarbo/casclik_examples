# CASCLIK EXAMPLES
This is a metapackage containing some example usage of CASCLIK.

## casclik_basics
This contains a default robot interface which is a python class that just tries to keep track of current `robot_var`, current `virtual_var`, and sends commands to a robot using `joint_position_controller` and callback on `joint_states`.

## casclik_tests
This contains simple tests of CASCLIK with robots. They are generally based on examples from constraint-based robotics articles, and some simple home-made examples. Many of these examples require [`py_viz_marker`](https://github.com/mahaarbo/py_viz_marker) which is a simple rviz marker adder.

1. Moe2016_example1/2 requires the UR packages (in particular: `ur_description` and `ur_gazebo`), `gazebo`, and `py_viz_marker`.
2. Aertbelien2013_example requires the PR2 packages. First test if `roslaunch pr2_gazebo pr2_empty_world.launch` works. If it doesn't, try [kinetic_pr2](https://github.com/RichardKelley/kinetic_pr2) if you're working with kinetic kame. Remember to state that we're simulating: `export ROBOT=sim`.
