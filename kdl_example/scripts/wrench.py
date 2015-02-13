#!/usr/bin/env python
import roslib
roslib.load_manifest('kdl_example')
import rospy
import PyKDL


"""
Demos how to transfrom wrench (Force/Torque) readings from
Frame B to Frame A

w_b: wrench in Frame B
w_a: wrench in Frame A
F_a_b: Frame B w.r.t. Frame A

Reference:
- Craig Eq. 5.105
- See kdl source code frames.inl 
"""

def main():
    w_b = PyKDL.Wrench()
    F_a_b = PyKDL.Frame()
    w_a = F_a_b * w_b 

if __name__ == '__main__':
    main()
