#!/usr/bin/env python

import rospy
from hlpr_manipulation_utils.manipulator import Gripper
from wpi_jaco_msgs.srv import GravComp
from kinova_msgs.srv import Start, Stop

"""
jaco_arm.py

Simple wrapper that abstracts out the arm class so that other arms
can use kinesthetic_interaction
"""

class Arm():

    GRAVITY_COMP_SERVICE = "/jaco_arm/grav_comp" 
    ENABLE_7DOF_GRAVITY_COMP_SERVICE = "/j2s7s300_driver/in/start_gravity_comp"
    DISABLE_7DOF_GRAVITY_COMP_SERVICE = "/j2s7s300_driver/in/stop_gravity_comp"

    def __init__(self, is7DOF = False):
        # Setup gravity compensation
        rospy.logwarn("Waiting for gravity compensation service")
        if (is7DOF):
            rospy.wait_for_service(Arm.ENABLE_7DOF_GRAVITY_COMP_SERVICE)
            rospy.wait_for_service(Arm.DISABLE_7DOF_GRAVITY_COMP_SERVICE)
            self.enableGravComp = rospy.ServiceProxy(Arm.ENABLE_7DOF_GRAVITY_COMP_SERVICE, Start)
            self.disableGravComp = rospy.ServiceProxy(Arm.DISABLE_7DOF_GRAVITY_COMP_SERVICE, Stop)
            self.gravity_comp = self.setGravityComp
        else:
            rospy.wait_for_service(Arm.GRAVITY_COMP_SERVICE)
            self.gravity_comp = rospy.ServiceProxy(Arm.GRAVITY_COMP_SERVICE, GravComp)
        rospy.logwarn("Gravity compenstation service loaded")

        # Initialize the gripper
        self.gripper = Gripper()

    def setGravityComp(self, toggle):
        if toggle:
            return self.enableGravComp()
        else:
            return self.disableGravComp()
 
