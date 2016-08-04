#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState
from hlpr_kinesthetic_interaction.srv import *



if __name__ == "__main__":

    rospy.init_node("kinesthetic_interaction_state")
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    with sm:

        smach.StateMachine.add('TRIGGER_KINESTHETIC_INTERACTION',
                               ServiceState('kinesthetic_interaction',
                                            KinestheticInteract,
                                            request = KinestheticInteractRequest(True)),
                               transitions={'succeeded':'succeeded'})

    outcome = sm.execute()
    if (outcome):
        rospy.loginfo("Successfully enabled kinesthetic interaction")
    else:
        rospy.logerror("Was not able to enable kinesthetic interaction")
