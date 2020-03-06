#!/usr/bin/env python

import rospy
import os
import sys
from hlpr_kinesthetic_teaching_api.kinesthetic_teaching_api import KTInterface
from std_srvs.srv import Empty
from hlpr_manipulation_utils.manipulator import Gripper
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt

def playback(bagfile):
    rospy.init_node("kt_bagfile_playback")
    #load KT Interface
    k = KTInterface("~/test_bagfiles",ArmMoveIt(default_planner="PRMStarkConfigDefault"), Gripper())
    #load bagfile (give name of bagfile with no extension)
    k.load_bagfile("~/test_bagfiles/"+ bagfile + ".bag", False)
    rospy.sleep(2.0)
    #begin at starting position
    k.move_to_keyframe(k.segment_pointer)
    #play recorded motion
    k.move_to_end()
    k.stop_tf_threads()
    
if __name__== "__main__":

    playback("test")
