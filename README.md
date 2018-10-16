# CASCLIK EXAMPLES
This is a metapackage containing some example usage of CASCLIK.

## casclik_basics
This contains a default robot interface which is a python class that just tries to keep track of current `robot_var`, current `virtual_var`, and sends commands to a robot using `joint_position_controller` and callback on `joint_states`.

## casclik_tests
This contains simple tests of CASCLIK with robots. They are generally based on examples from constraint-based robotics articles, and some simple home-made examples. 
