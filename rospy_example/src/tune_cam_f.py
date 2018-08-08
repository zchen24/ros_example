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
import tf2_ros
import PyKDL
from PyKDL import *
import cmd
import threading
from geometry_msgs.msg import TransformStamped


class FrameKeyboardTuner(cmd.Cmd):
    """Simple command processor example"""

    def __init__(self):
        cmd.Cmd.__init__(self)

        self.prompt = 'prompt: '
        self.intro = "Simple Interface for Defining Line VF"

        # help doc
        self.ruler = '-'

        # ros
        self.br = tf2_ros.TransformBroadcaster()

        # states
        # self.frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.462, -0.481, 0.566, -0.485),
        #                          PyKDL.Vector(-0.071, 0.003, 0.002))
        self.frame = Frame(PyKDL.Rotation.Quaternion(0, 0, 0, 1),
                           PyKDL.Vector(0, 0, 0))

        self.inc_pos = 0.005
        self.inc_rot = 0.005

        self.t = threading.Thread(target=self.thread_update_pose)
        self.t.start()

    def thread_update_pose(self):
        # update robot pose
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "my_camera"
            t.transform.translation.x = self.frame.p.x()
            t.transform.translation.y = self.frame.p.y()
            t.transform.translation.z = self.frame.p.z()
            x, y, z, w = self.frame.M.GetQuaternion()
            t.transform.rotation.x = x
            t.transform.rotation.y = y
            t.transform.rotation.z = z
            t.transform.rotation.w = w
            self.br.sendTransform(t)
            rate.sleep()

    def do_1(self, line):
        """
        increment x
        """
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(self.inc_pos, 0, 0))

    def do_2(self, line):
        """
        decrement x
        """
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(-self.inc_pos, 0, 0))

    def do_3(self, line):
        """
        increment y
        """
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(0, self.inc_pos, 0))

    def do_4(self, line):
        """
        decrement y
        """
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(0, -self.inc_pos, 0))

    def do_5(self, line):
        """
        increment z
        """
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(0, 0, self.inc_pos))

    def do_6(self, line):
        """
        decrement z
        """
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(0, 0, -self.inc_pos))

    def do_q(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation.RPY(self.inc_rot, 0, 0),
                                              PyKDL.Vector(0, 0, 0))
    def do_w(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation.RPY(-self.inc_rot, 0, 0),
                                              PyKDL.Vector(0, 0, 0))

    def do_e(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, self.inc_rot, 0),
                                              PyKDL.Vector(0, 0, 0))
    def do_r(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, -self.inc_rot, 0),
                                              PyKDL.Vector(0, 0, 0))

    def do_t(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, self.inc_rot),
                                              PyKDL.Vector(0, 0, 0))
    def do_y(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -self.inc_rot),
                                              PyKDL.Vector(0, 0, 0))
    def do_z(self, line):
        print(self.frame)

    def do_EOF(self, line):
        print("Exit")
        if not rospy.is_shutdown():
            rospy.signal_shutdown('Quit from CMD');
        return True

def main():

    # init ROS
    rospy.init_node('tune_cam_f')
    rospy.loginfo('Tune Camera Frame')

    FrameKeyboardTuner().cmdloop()

if __name__ == '__main__':
    main()
