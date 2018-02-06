#!/usr/bin/env python

"""
Zihan Chen   2018-02-05

- Converts between KDL Frame and Pose

"""

from PyKDL import *
from geometry_msgs.msg import Pose
import tf_conversions.posemath as pm


kdl_frame = Frame(Vector(1,2,3))
pose_msg = pm.toMsg(kdl_frame)
kdl_frame_back = pm.fromMsg(pose_msg)

import ipdb; ipdb.set_trace()
