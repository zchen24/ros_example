#!/usr/bin/env python

# Reference: 
# https://answers.ros.org/question/10499/best-way-to-check-if-ros-is-running/

import rosgraph 

if rosgraph.is_master_online():
    print 'ROS MASTER is Online'
else:
    print 'ROS MASTER is Offline'