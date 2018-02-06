#!/usr/bin/env python

from PyKDL import *
from math import pi


"""
Zihan Chen 2018-02-05

- Create a serial link robot
- Forward kinematics solver
- TODO: IK solver
"""

robot = Chain()

inert = RigidBodyInertia(0, Vector(0, 0, 0), RotationalInertia(0.1, 0.1, 0.1, 0.0, 0.0, 0.0))
robot.addSegment(Segment(Joint(), Frame(Rotation().RotX(-pi/2), Vector(0, 0, 0.05)), inert))

# J1 rot x -90 deg, L1y = -0.040, L1z = 0.12875
inert = RigidBodyInertia(0, Vector(0, 0, 0), RotationalInertia(0.1, 0.1, 0.1, 0.0, 0.0, 0.0))
robot.addSegment(Segment(Joint(Joint.RotZ), Frame(Rotation().RotX(-pi/2), Vector(0, -0.05, -0.10)), inert))


fkSolver = ChainFkSolverPos_recursive(robot)

jnts = JntArray(1)
jnts[0] = pi/3

ee_frame = Frame()
fkSolver.JntToCart(jnts, ee_frame)
print('end effector = ' + str(ee_frame))

