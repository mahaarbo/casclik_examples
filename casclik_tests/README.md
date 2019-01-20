# Examples

## aertbelien2014\_example
`roslaunch casclik_tests aertbelien2014_example.launch` will run an example of a PR2 with one hand moving in a circle, another moving its hand in another circle while trying to avoid the first, and the head looking at the first hand. Simple example of something that can be hard for a human to coordinate as well.

## moe2016\_example1
`roslaunch casclik_tests moe2016_example1.launch` will run an example of a UR5 moving to two different points in sequence while trying to keep the end-effector position outside a ball and keeping the end-effector oriented forward.

## moe2016\_example2
`roslaunch casclik_tests moe2016_example2.launch` will run an example of a UR5 tracking a trajectory while remaining inside a box.
	
## tracking\_input\_marker
`roslaunch casclik_tests tracking_input_marker.launch` will run an example of a UR5 trapped inside a box while it is trying to track an input marker. That is, CASCLIK is trying to minimize the distance between the input marker (that the user can move) and the position of the end-effector.
