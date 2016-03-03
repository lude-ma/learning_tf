#!/usr/bin/env python

import math
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('carrot_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # Create a transform from parent 'turtle1' to child 'carrot1'
        # that changes over time.
        t = rospy.Time.now().to_sec()*math.pi
        br.sendTransform((2.0*math.sin(t), 2.0*math.cos(t), 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "carrot1",
                         "turtle1")
        rate.sleep()
