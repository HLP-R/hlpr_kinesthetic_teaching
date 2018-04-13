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
from hlpr_kinesthetic_teaching_api.kinesthetic_teaching_api import KTInterface


if __name__=="__main__":
    rospy.init_node("kt_api_testing")
    k = KTInterface("~/test_bagfiles")

    rospy.sleep(0.5)
    k.start("test")
    while not rospy.is_shutdown():
        
        print "-"*60
        print "Current frames: "
        for s in k.segments:
            if k.at_keyframe_target(segment_pointer):
                pref = "**"
            else:
                pref = "  "
                
            print pref, s

        if k.is_joints:
            mode = "joint keyframe"
        else:
            mode = "eef keyframe"
        print "In {} mode.".format(mode)
        print "Press enter to grab a keyframe; type 'd' to delete."
        print "Type 'n' to move to the next keyframe; 'p' to move to previous."
        print "Type 's' to move to the start; 'e' to move to the end."
        print "Type 'r' to release the arm and 'l' to lock the arm."
        print "Type 'o' to open gripper and 'c' to close."
        print "Type 'j' to toggle joint keyframe mode."
        print "Type 'q' to write to a bag and quit."
        r = raw_input()
        if r == '':
            k.write_kf()
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
            k.close_gripper()
        elif r=='o':
            k.open_gripper()
        elif r=='q':
            break
        
    k.end()
