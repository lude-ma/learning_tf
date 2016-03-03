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
    timeout = rospy.Duration(1.0)
    listener.waitForTransform("/turtle2", "/turtle1", rospy.Time(0), timeout)
    rospy.sleep(5.0)
    while not rospy.is_shutdown():
        # We query the listener for a specific transformation,
        # from current frame 'turtle2' to frame 'turtle1' 5 seconds ago.
        try:
            now = rospy.Time.now()
            past = now - rospy.Duration(5.0)
            # The advanced API for lookupTransform() computes the transform
            # from frame 'turtle2' at time 'now' to frame 'turtle1' at time
            # 'past' using the help of a fixed frame 'world' that does not
            # change over time.
            # What happens is the following:
            # - In the past tf computes the transform from 'turtle1' to 'world'.
            # - In the world frame tf time travels from the past to now.
            # - At time now tf computes the transform from 'world' to 'turtle2'.
            listener.waitForTransformFull('/turtle2', now,
                                          '/turtle1', past,
                                          '/world',
                                          timeout)
            (trans, rot) = listener.lookupTransformFull('/turtle2', now,
                                                        '/turtle1', past,
                                                        '/world')
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
