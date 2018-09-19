#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import os
from hlpr_kinesthetic_teaching_api.kinesthetic_teaching_api import KTInterface
from std_srvs.srv import Empty
from std_msgs.msg import String

if os.environ["ROBOT_NAME"]=="2d_arm":
    from hlpr_2d_arm_sim.sim_arm_moveit import Gripper2D, Planner2D
else:
    from hlpr_manipulation_utils.manipulator import Gripper
    from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt

if __name__=="__main__":
    rospy.init_node("kt_api_testing")
    pub = rospy.Publisher('log_KTframe', String, queue_size=10)

    print "Please enter a bagfile name."
    filename = raw_input()
    if filename == "":
        print "No filename provided, exiting..."
        exit()

    if os.environ["ROBOT_NAME"]=="2d_arm":
        k = KTInterface("~/test_bagfiles",Planner2D("/sim_arm/joint_state", "/sim_arm/move_arm"), Gripper2D("/sim_arm/gripper_state","/sim_arm/gripper_command"),False)
    else:
        k = KTInterface("~/test_bagfiles",ArmMoveIt(eef_frame='right_ee_link'), Gripper(),False)

    freezer = rospy.ServiceProxy('freeze_frames', Empty)
        
    rospy.sleep(0.5)
    k.release_arm()


    print k.planner.group[0].get_end_effector_link()
    
    print "-"*60

    valid = False
    first_is_joints = None
    while not rospy.is_shutdown():
        print "Type 'j' to save starting pose as a joint pose"
        print "Type 'e' to save starting pose as an eef pose"
        r = raw_input()
        if r=='j':
            first_is_joints = True
            break
        elif r=='e':
            first_is_joints = False
            break


    print "Move the arm to the desired starting pose and hit enter"
    raw_input()
    k.start(filename, is_joints=first_is_joints)

    pause_tf_record = False
    while not rospy.is_shutdown():
        
        print "="*25 + "Robot: " + os.environ["ROBOT_NAME"] + "="*25
        print "Current frames: "
        for s in k.segments:
            if k.segment_pointer==s:
                pref = " >"
            else:
                pref = "  "
                
            print pref, s
        print
        
        if k.is_joints:
            mode = "joint keyframe"
        else:
            mode = "eef keyframe"
        print "In {} mode. TF recording is {}.".format(mode, not pause_tf_record)
        print "Press enter to grab a keyframe; type 'd' to delete."
        print "'n' -> move to the next keyframe"
        print "'p' -> move to previous keyframe"
        print "'h' -> move to current keyframe"
        print
        print "Type 's' to move to the start; 'e' to move to the end."
        print "Type 'r' to release the arm and 'l' to lock the arm."
        print "Type 'o' to open gripper and 'c' to close."
        print "Type 'j' to toggle joint keyframe mode."
        print "Type 'q' to write to a bag and quit."
        print "Type 'a' to toggle freezing of tf."
        print "-"*60
        k.print_current_pose()
        print "-"*60

        
        r = raw_input()
        if r == 'h':
            k.move_to_keyframe(k.segment_pointer)
        elif r=='d':
            k.remove_current_frame()
        elif r=='n':
            k.move_forward()
        elif r=='p':
            k.move_backward()
        elif r=='j':
            k.is_joints = not k.is_joints
        elif r=='s':
            k.move_to_start()
        elif r=='e':
            k.move_to_end()
        elif r=='r':
            k.release_arm()
        elif r=='l':
            k.lock_arm()
        elif r=='c':
            pub.publish("Close gripper")
            k.close_gripper()           
        elif r=='o':
            pub.publish("Open gripper")
            k.open_gripper()
        elif r=='a':
            try:
                freezer()
            except rospy.ServiceException as e:
                print "Couldn't freeze frames; is freeze frame started from hlpr_manipulation utils?"
        elif r=='q':
            break
        
        elif r=='1':
            pub.publish("Recorded keyframe: Step 1 Reaching")
            k.write_kf(r)
        elif r=='2':
            pub.publish("Recorded keyframe: Step 2 Grasping")
            k.write_kf(r)
        elif r=='3':
            pub.publish("Recorded keyframe: Step 3 Transport")
            k.write_kf(r)
        elif r=='4':
            pub.publish("Recorded keyframe: Step 4 Pouring/Release")
            k.write_kf(r)
        elif r=='5':
            pub.publish("Recorded keyframe: Step 5 Return")
            k.write_kf(r)
        elif r=='6':
            pub.publish("Recorded keyframe: Step 6 Release")
            k.write_kf(r)
        else:
            pub.publish("Recorded keyframe")
            k.write_kf(r)
            
        
    k.end()
    k.stop_tf_threads()
