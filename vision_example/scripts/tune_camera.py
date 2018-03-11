#!/usr/bin/env python

"""
Zihan Chen
2015-03 

reg_file : stores initial hand-eye calibraiton data 
ref_frame: the reference frame in this hand-eye calibration
cam_frame: camera frame name

Script to manually tune camera frame w.r.t Robot
Use case: hand-eye calibration does not quite work ....

 - 1 2 x
 - 3 4 y
 - 5 6 z
 - q w rot x
 - e r rot y
 - t y rot z
"""

import rospy
import rospkg
import tf
import copy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
import tf_conversions.posemath as pm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math
import PyKDL

import readline
import cmd
import threading 


class TuneCamera(cmd.Cmd):
    """Simple command processor example"""

    def __init__(self):
        cmd.Cmd.__init__(self)

        self.prompt = 'prompt: '
        self.intro = "Simple Interface for Defining Line VF"

        # help doc
        self.ruler = '-'

        # ros
        self.br = tf.TransformBroadcaster()

        # data file
        rospack = rospkg.RosPack()
        self.reg_file = rospack.get_path('vision_example') + '/data/cam_reg_wam.txt'
        self.reg_file = rospy.get_param('~reg_file', self.reg_file)
        rospy.loginfo('reg_file  = ' + self.reg_file)

        try:
            f = open(self.reg_file, 'r')
            rot = [float(x) for x in f.readline().split()]
            pos = [float(x) for x in f.readline().split()]
            f.close()
        except IOError:
            rospy.logfatal("Can't open init reg file")
            raise
        
        # cam_frame name
        self.cam_frame = '/camera_frame'
        self.cam_frame = rospy.get_param('~cam_frame', self.cam_frame)
        rospy.loginfo('cam_frame = ' + self.cam_frame)

        # ref_frame name
        self.ref_frame = '/wam/left_camera_frame'
        self.ref_frame = rospy.get_param('~ref_frame', self.ref_frame)
        rospy.loginfo('ref_frame = ' + self.ref_frame)

        # states
        self.frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]),
                                 PyKDL.Vector(pos[0], pos[1], pos[2]))
        rospy.loginfo('init calibration frame = \n' + str(self.frame))

        self.inc_pos = 0.00005
        self.inc_rot = 0.0002

        self.t = threading.Thread(target=self.thread_update_pose)
        self.t.start()

    def thread_update_pose(self):
        # update robot pose
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            tf = pm.toTf(self.frame)
            self.br.sendTransform(tf[0],
                                  tf[1],
                                  rospy.Time.now(),
                                  self.cam_frame, 
                                  self.ref_frame)
            rate.sleep()
            pass

    def do_1(self, line):
        """
        add x 
        """
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(self.inc_pos, 0, 0))

    def do_2(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(-self.inc_pos, 0, 0))

    def do_3(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(0, self.inc_pos, 0))

    def do_4(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(0, -self.inc_pos, 0))

    def do_5(self, line):
        self.frame = self.frame * PyKDL.Frame(PyKDL.Rotation(),
                                              PyKDL.Vector(0, 0, self.inc_pos))

    def do_6(self, line):
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
        
    def do_p(self, line):
        print self.frame.p
        print self.frame.M.GetRPY()
        print self.frame.M.GetQuaternion()
        print 'inc_pos = ' + str(self.inc_pos)
        print 'inc_rot = ' + str(self.inc_rot)

    def do_z(self, line):
        print "Saving data to " + self.reg_file
        self.do_p('')

        # save to file
        f = open(self.reg_file, 'w')
        pos = self.frame.p
        rot = self.frame.M.GetQuaternion()
        f.write(str(rot[0]) + '  ' + str(rot[1]) + '  ' + str(rot[2]) + '  ' + str(rot[3]) + '  \n')
        f.write(str(pos.x()) + '  ' + str(pos.y()) + '  ' + str(pos.z()) + '  \n')
        f.close()

    def do_up(self, line):
        self.inc_pos = self.inc_pos * 2.0
        print 'inc_pos = ' + str(self.inc_pos)
        
    def do_down(self, line):
        self.inc_pos = self.inc_pos / 2.0
        print 'inc_pos = ' + str(self.inc_pos)

    def do_help(self, line):
        print "========== HELP ========="
        print " - 1 2  : pos x            "
        print " - 3 4  : pos y            "
        print " - 5 6  : pos z            "
        print " - q w  : rot x            "
        print " - e r  : rot y            "
        print " - t y  : rot z            "
        print " - p    : print calib frame"
        print " - z    : save to reg file "
        print " - up   : inc x 2          "
        print " - down : inc / 2          "

    def do_EOF(self, line):
        print "Exit"
        if not rospy.is_shutdown():
            rospy.signal_shutdown('Quit from CMD');
        return True

def main():

    # init ROS
    rospy.init_node('tune_cam_f')
    rospy.loginfo('Tune Camera Frame')
    
    try:
        tc = TuneCamera()
        tc.cmdloop()
    except Exception, e:
        print str(e)
        rospy.logfatal('Failed to init TuneCamera')

if __name__ == '__main__':
    main()
