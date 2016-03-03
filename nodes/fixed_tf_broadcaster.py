#!/usr/bin/env python

import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('carrot_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # Create a fixed transform from parent 'turtle1' to child 'carrot1'
        # with a 2m offset in positive y direction between the frames.
        # That is, 'carrot1' is 2m to the left of 'turtle1'.
        br.sendTransform((0.0, 2.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "carrot1",
                         "turtle1")
        rate.sleep()
