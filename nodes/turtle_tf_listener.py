#!/usr/bin/env python
import math
import rospy
import tf

import geometry_msgs.msg
import turtlesim.srv


if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    # The listener receives tf transformations over the wire
    # and buffers them for up to 10 seconds (by default).
    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel',
                                 geometry_msgs.msg.Twist,
                                 queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # We query the listener for a specific transformation,
        # from frame 'turtle2' to frame 'carrot1' at time 'now'.
        # Since it takes some time (typically a few milliseconds) for
        # broadcasted transforms to get into the buffer, this query fails
        # with a ExtrapolationException.
        try:
            now = rospy.Time.now()
            (trans, rot) = listener.lookupTransform('/turtle2',
                                                    '/carrot1',
                                                    now)
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            continue

        angular = 4*math.atan2(trans[1], trans[0])
        linear = 0.5*math.sqrt(trans[0]**2 + trans[1]**2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()
