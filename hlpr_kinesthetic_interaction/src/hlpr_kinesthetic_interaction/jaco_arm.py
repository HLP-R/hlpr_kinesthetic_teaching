#!/usr/bin/env python

import rospy
from hlpr_manipulation_utils.manipulator import Gripper
from wpi_jaco_msgs.srv import GravComp
from hlpr_kinesthetic_interaction.kinesthetic_interaction import KinestheticInteraction

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
        self.toggle_gravity_comp = rospy.ServiceProxy(Arm.GRAVITY_COMP_SERVICE, GravComp)
        rospy.logwarn("Gravity compenstation service loaded")

        # Initialize the gripper
        self.gripper = Gripper()

    def gravity_comp(self, toggle, ft_mode):
        if ft_mode == KinestheticInteraction.TORQUE_MODE:
            return self.toggle_gravity_comp(toggle)
        else:
            rospy.logwarn("There is currently no force mode API for 6dof. Nothing will happen. Please use torque mode")
            return False
