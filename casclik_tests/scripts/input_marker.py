#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import Pose, Point, Quaternion


class InteractivePosePublisher(object):
    """Spawns an interactive marker that publishes its pose.
    """
    def __init__(self, root,
                 name,
                 description,
                 server,
                 ns="",
                 position0=[0., 0., 0.]):
        # Make interactive marker
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = root
        self.int_marker.pose.position = Point(*position0)
        self.int_marker.scale = 0.1
        self.int_marker.name = name
        self.name = name
        self.int_marker.description = description
        
        # Make the way it should look
        mrkrmsg = Marker()
        mrkrmsg.type = Marker.CUBE
        mrkrmsg.scale.x = 0.03
        mrkrmsg.scale.y = 0.03
        mrkrmsg.scale.z = 0.03
        mrkrmsg.color.r = 0.7
        mrkrmsg.color.a = 1.0

        # Make the Inner cube control mode:
        cubecntrl = InteractiveMarkerControl()
        cubecntrl.always_visible = True
        cubecntrl.interaction_mode = InteractiveMarkerControl.MOVE_3D
        cubecntrl.markers.append(mrkrmsg)
        controls = [cubecntrl]
        # Make XYZ linear motion control modes:
        for i in range(3):
            direction = [0]*i + [1] + [0]*(2-i)
            control = InteractiveMarkerControl()
            control.name = "move_"+chr(ord("x")+i)
            control.orientation = Quaternion(*(direction+[1]))
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            controls.append(control)

        # Make XYZ rotation control modes:
        for i in range(3):
            direction = [0]*i + [1] + [0]*(2-i)
            control = InteractiveMarkerControl()
            control.name = "rotate_"+chr(ord("x")+i)
            control.orientation = Quaternion(*(direction+[1]))
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            controls.append(control)
        # Add them to the interactive marker
        self.int_marker.controls = controls

        # Setup pose publisher
        self.pose_pub = rospy.Publisher(ns+name,
                                        Pose,
                                        queue_size=1)
        # Setup the interactive marker in the server
        self.server = server
        self.server.insert(self.int_marker, self.callbackPublish)
        self.server.applyChanges()

    def callbackPublish(self, feedback):
        if feedback.marker_name == self.name:
            self.pose_pub.publish(feedback.pose)
        self.server.applyChanges()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Node that spawns an "
                                     + "interactive Rviz marker that p"
                                     + "ublishes its pose.")
    parser.add_argument("-n", "--name",
                        help="name of the marker and publish topic.",
                        type=str, required=True)
    parser.add_argument("-d", "--description",
                        help="description of the marker",
                        type=str, default="input_marker")
    parser.add_argument("-ns", "--namespace",
                        help="The namespace in which the published topic is.",
                        type=str, default="")
    parser.add_argument("-p0", "--position0",
                        help="The initial position of the marker.",
                        nargs=3, default=[0.5, 0.5, 0.5], type=float)
    parser.add_argument("-s", "--server",
                        help="Namespace of the InteractiveMarkerServer topic",
                        type=str, default="input_marker")
    parser.add_argument("-r", "--root",
                        help="Root frame that the marker pose is relative to.",
                        type=str, default="world")
    args, unknown_args = parser.parse_known_args()

    rospy.init_node("input_marker", anonymous=True)
    server = InteractiveMarkerServer(args.server)
    ipp = InteractivePosePublisher(root=args.root,
                                   name=args.name,
                                   description=args.description,
                                   server=server,
                                   ns=args.namespace,
                                   position0=args.position0)

    # Run until things die
    rospy.spin()
        
