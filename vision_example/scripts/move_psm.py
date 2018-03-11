#!/usr/bin/env python

"""
Script to define a line virtual fixture:
 - 1 2 x
 - 3 4 y
 - 5 6 z
 - q w rot x
 - e r rot y
 - t y rot z
 
"""

import rospy
import tf
import copy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_msgs.msg import Float32
import tf_conversions.posemath as pm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose

import math
import PyKDL

import readline
import cmd
import threading 


class KeyboardTeleop(cmd.Cmd):
    """Simple command processor example"""

    def __init__(self, ns):
        cmd.Cmd.__init__(self)

        self.prompt = 'cmd: '
        self.intro = 'Use Keyboard to Move PSM, Name = ' + ns

        # help doc
        self.ruler = '-'

        # ros
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.pub_psm_cmd_pose = rospy.Publisher( ns + '/set_position_cartesian', Pose)
        self.pub_open_angle = rospy.Publisher(ns + '/set_open_angle', Float32)
        self.sub_psm_pose = rospy.Subscriber(ns + '/cartesian_pose_current', Pose, self.callback_pose)

        # states
        self.open_angle = 0.0
        self.tip_cmd_frame = PyKDL.Frame()
        self.tip_frame = None
        self.offset_frame = PyKDL.Frame(PyKDL.Rotation(0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0),
                                        PyKDL.Vector(0.0, 0.0102, 0.0))

        self.inc_pos = 0.0002
        self.inc_rot = 0.01
        self.is_sync = False

    def callback_pose(self, msg):
        self.tip_frame = pm.fromMsg(msg) * self.offset_frame

    def publish_cmd_frame(self):
        """
        Publish cmd slave pose with safety check
        """
        if self.tip_frame is not None:
            pos_diff = self.tip_frame.p - self.tip_cmd_frame.p
            rot_diff = (self.tip_frame.M.Inverse() * self.tip_cmd_frame.M).GetRot()

            if (pos_diff.Norm() > 0.01 or rot_diff.Norm() > 0.5):
                rospy.logerr('Command is not synchronized, Press s to sync')
                rospy.logerr('pos: ' + str(pos_diff.Norm()) + '   rot: ' + str(rot_diff.Norm()))
            else:
                cmd6 = self.tip_cmd_frame * self.offset_frame.Inverse()
                msg = pm.toMsg(cmd6)
                self.pub_psm_cmd_pose.publish(msg)

    def do_1(self, line):
        """
        increment x 
        """
        self.tip_cmd_frame = PyKDL.Frame(PyKDL.Rotation(),
                                         PyKDL.Vector(self.inc_pos, 0, 0)) * self.tip_cmd_frame
        self.publish_cmd_frame()

    def do_2(self, line):
        """
        decrement x
        """
        self.tip_cmd_frame = PyKDL.Frame(PyKDL.Rotation(),
                                         PyKDL.Vector(-self.inc_pos, 0, 0)) * self.tip_cmd_frame
        self.publish_cmd_frame()

    def do_3(self, line):
        """
        increment y
        """
        self.tip_cmd_frame = PyKDL.Frame(PyKDL.Rotation(),
                                         PyKDL.Vector(0, self.inc_pos, 0)) * self.tip_cmd_frame
        self.publish_cmd_frame()

    def do_4(self, line):
        """
        decrement y
        """
        self.tip_cmd_frame = PyKDL.Frame(PyKDL.Rotation(),
                                         PyKDL.Vector(0, -self.inc_pos, 0)) * self.tip_cmd_frame
        self.publish_cmd_frame()        

    def do_5(self, line):
        """
        increment z
        """
        self.tip_cmd_frame = PyKDL.Frame(PyKDL.Rotation(),
                                         PyKDL.Vector(0, 0, self.inc_pos)) * self.tip_cmd_frame
        self.publish_cmd_frame()

    def do_6(self, line):
        """
        decrement z
        """
        self.tip_cmd_frame = PyKDL.Frame(PyKDL.Rotation(),
                                         PyKDL.Vector(0, 0, -self.inc_pos)) * self.tip_cmd_frame
        self.publish_cmd_frame()

    def do_7(self, line):
        """
        close gripper
        """
        if self.is_sync:
            self.open_angle = max(self.open_angle - 0.05, -0.262)
            self.pub_open_angle.publish(self.open_angle)
        else:
            rospy.logerr('Sync Cmd Position')

    def do_8(self, line):
        """
        open gripper
        """
        if self.is_sync:
            self.open_angle = min(self.open_angle + 0.05, 1.05)
            self.pub_open_angle.publish(self.open_angle)
        else:
            rospy.logerr('Sync Cmd Position')

    def do_q(self, line):
        """
        rot +x
        """
        self.tip_cmd_frame = self.tip_cmd_frame * PyKDL.Frame(PyKDL.Rotation.RPY(self.inc_rot, 0, 0),
                                              PyKDL.Vector(0, 0, 0))
        self.publish_cmd_frame()
        
    def do_w(self, line):
        """
        rot -x
        """
        self.tip_cmd_frame = self.tip_cmd_frame * PyKDL.Frame(PyKDL.Rotation.RPY(-self.inc_rot, 0, 0),
                                              PyKDL.Vector(0, 0, 0))
        self.publish_cmd_frame()

    def do_e(self, line):
        """
        rot +y
        """
        self.tip_cmd_frame = self.tip_cmd_frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, self.inc_rot, 0),
                                              PyKDL.Vector(0, 0, 0))
        self.publish_cmd_frame()
        
    def do_r(self, line):
        """
        rot -y
        """
        self.tip_cmd_frame = self.tip_cmd_frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, -self.inc_rot, 0),
                                              PyKDL.Vector(0, 0, 0))
        self.publish_cmd_frame()

    def do_t(self, line):
        """
        rot +z
        """
        self.tip_cmd_frame = self.tip_cmd_frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, self.inc_rot),
                                              PyKDL.Vector(0, 0, 0))
        self.publish_cmd_frame()
        
    def do_y(self, line):
        """
        rot -z
        """
        self.tip_cmd_frame = self.tip_cmd_frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -self.inc_rot),
                                              PyKDL.Vector(0, 0, 0))
        self.publish_cmd_frame()

    def do_s(self, line):
        print 'Syncing command frame to current tip frame'
        self.tip_cmd_frame = self.tip_frame
        self.is_sync = True
        self.publish_cmd_frame()
        
    def do_p(self, line):
        print self.tip_frame.p
        print self.tip_frame.M.GetRPY()
        print self.tip_frame.M.GetQuaternion()

    def do_EOF(self, line):
        print "Exit"
        if not rospy.is_shutdown():
            rospy.signal_shutdown('Quit from CMD');
        return True

def main():

    # init ROS
    rospy.init_node('move_psm')
    rospy.loginfo('Move PSM with Keyboard')
    ns = rospy.get_param('~ns', '/dvrk_psm1')   # namespace

    print ns

    KeyboardTeleop(ns).cmdloop()

if __name__ == '__main__':
    main()
