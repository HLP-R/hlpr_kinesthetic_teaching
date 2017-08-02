#!/usr/bin/env python

import rospy
from hlpr_manipulation_utils.manipulator import Gripper
from kinova_msgs.srv import Start, Stop
from hlpr_kinesthetic_interaction.kinesthetic_interaction import KinestheticInteraction

"""
jaco_7dof_arm.py

Simple wrapper that abstracts out the arm class so that other arms
can use kinesthetic_interaction
"""

class Arm():

    ENABLE_7DOF_GRAVITY_COMP_SERVICE = "/j2s7s300_driver/in/start_gravity_comp"
    DISABLE_7DOF_GRAVITY_COMP_SERVICE = "/j2s7s300_driver/in/stop_gravity_comp"
    ENABLE_7DOF_FORCE_SERVICE = "/j2s7s300_driver/in/start_force_control"
    DISABLE_7DOF_FORCE_SERVICE = "/j2s7s300_driver/in/stop_force_control"

    def __init__(self):
        # Setup gravity compensation
        rospy.logwarn("Waiting for gravity compensation service")
        rospy.wait_for_service(Arm.ENABLE_7DOF_GRAVITY_COMP_SERVICE)
        rospy.wait_for_service(Arm.DISABLE_7DOF_GRAVITY_COMP_SERVICE)
        rospy.wait_for_service(Arm.ENABLE_7DOF_FORCE_SERVICE)
        rospy.wait_for_service(Arm.DISABLE_7DOF_FORCE_SERVICE)

        # Store the services
        self.enable_grav_comp = rospy.ServiceProxy(Arm.ENABLE_7DOF_GRAVITY_COMP_SERVICE, Start)
        self.disable_grav_comp = rospy.ServiceProxy(Arm.DISABLE_7DOF_GRAVITY_COMP_SERVICE, Stop)
        self.enable_force = rospy.ServiceProxy(Arm.ENABLE_7DOF_FORCE_SERVICE, Start)
        self.disable_force = rospy.ServiceProxy(Arm.DISABLE_7DOF_FORCE_SERVICE, Stop)
        rospy.logwarn("Gravity compenstation service loaded")

        # Initialize the gripper
        self.gripper = Gripper()

    def gravity_comp(self, toggle, ft_mode):
        if ft_mode == KinestheticInteraction.TORQUE_MODE:
            if toggle:
                return self.enable_grav_comp()
            else:
                return self.disable_grav_comp()
        elif ft_mode == KinestheticInteraction.FORCE_MODE:
            if toggle:
                return self.enable_force()
            else:
                return self.disable_force()
        else:
            rospy.logerr("Passed in unsupported ft mode: %s. Nothing will happen" % ft_mode)
            return False
 
