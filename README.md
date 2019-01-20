# CASCLIK EXAMPLES
This is a metapackage containing some example usage of CASCLIK.

## casclik_basics
This contains a default robot interface which is a python class that just tries to keep track of current `robot_var`, current `virtual_var`, and sends commands to a robot using `joint_position_controller` and callback on `joint_states`.

## casclik_tests
This contains simple tests of CASCLIK with robots. They are generally based on examples from constraint-based robotics articles, and some simple home-made examples. Many of these examples require [`py_viz_marker`](https://github.com/mahaarbo/py_viz_marker) which is a simple rviz marker adder.

1. `moe2016_example1/2` requires the UR packages (in particular: `ur_description` and `ur_gazebo`), `gazebo`, and `py_viz_marker`.
2. `aertbelien2014_example` requires the PR2 packages. If you want to explore the PR2 with kinetic kame try [kinetic_pr2](https://github.com/RichardKelley/kinetic_pr2). We're running our own butchered version of the URDF from that git repository, so we only need the meshes from `pr2_description` to be available. It might work to simply download the `pr2_description` from apt.


### Troubleshooting:
1. If you see mentions of missing position controllers, try `sudo apt-get install ros-VERSION-position-controllers`
2. TIFFFieldWithTag: Internal error, unknown tag 0xa20c when running `aertbelien2014_example`. This comes from Rviz not understanding TIFF files in the current version. You can safely ignore these errors.
