#!/usr/bin/env python

"""
Shows how to use rqt_bag Recorder to record
ROS messages.

Run:
./rosbag_example.py

Bag Info
rosbag info tmp.bag

Zihan Chen
2018-06-11
"""

import rospy
from rqt_bag import recorder
import time

if __name__ == '__main__':
    bag_file_name = 'tmp.bag'
    rospy.init_node('example_recorder')
    r = recorder.Recorder(bag_file_name,
                          all=False,
                          topics=['/rosout'])
    r.start()
    time.sleep(2.0)
    r.stop()
