#!/usr/bin/env python
import roslib
roslib.load_manifest('tf_example')
import rospy

import tf
import turtlesim.msg

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0, 1, 2),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         '/tip',
                         '/base')
        rate.sleep()
    
