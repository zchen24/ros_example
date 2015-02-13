#!/usr/bin/env python
import roslib
roslib.load_manifest('tf_example')
import rospy
import tf

"""
Notes: Zihan Chen   2015-02-12

- tf listener takes time to build, so if you put it inside the while
  loop, this slows the run loop to around 2 hz

- lookupTransform, takes base frame 1st, tip frame 2nd, the return
  value is tip w.r.t. base

- tf subscribes to /tf topic, which takes time to build,
  waitForTransform can be used to wait for the info
"""


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            t = rospy.Time(0)
            listener.waitForTransform('/base', '/tip', t, rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransform('/base', '/tip', t)
            print 'trans = ' + str(trans)
            print 'rot   = ' + str(rot)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()
