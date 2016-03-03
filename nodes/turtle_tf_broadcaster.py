#!/usr/bin/env python
import rospy
import tf

import turtlesim.msg


def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    # Publish this turtle's translation and rotation
    # as a translation from frame 'world' to frame'turtlename'.
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0,
                                                              0,
                                                              msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")


if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    # Get the name of the turtle from ROS.
    # The value is specified in the launch file.?
    turtlename = rospy.get_param('~turtle')
    # Subscribe to topic '/turtlename/pose' and
    # run function 'handle_turtle_pose' on every incoming message.
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
