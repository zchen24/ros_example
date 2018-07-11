#!/usr/bin/env python

"""
Showes how to use argparse and remove ros additional arguments. 
"""

import argparse
import rospy

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-mm', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    print(args.mm)
