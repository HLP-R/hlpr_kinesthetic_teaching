#!/usr/bin/env python

import rospy
from hlpr_manipulation_utils.manipulator import Gripper
from wpi_jaco_msgs.srv import GravComp

"""
jaco_arm.py

Simple wrapper that abstracts out the arm class so that other arms
can use kinesthetic_interaction
"""

class Arm():

    GRAVITY_COMP_SERVICE = "/jaco_arm/grav_comp" 

    def __init__(self):
        # Setup gravity compensation
        rospy.logwarn("Waiting for gravity compensation service")
        rospy.wait_for_service(Arm.GRAVITY_COMP_SERVICE)
        self.gravity_comp = rospy.ServiceProxy(Arm.GRAVITY_COMP_SERVICE, GravComp)
        rospy.logwarn("Gravity compenstation service loaded")

        # Initialize the gripper
        self.gripper = Gripper()
 
