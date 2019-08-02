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

import argparse
import os
import rospy

from hlpr_kinesthetic_teaching_api.kinesthetic_teaching_api import KTInterface
from hlpr_manipulation_utils.srv import FreezeFrame, FreezeFrameRequest

if os.environ["ROBOT_NAME"]=="2d_arm":
    from hlpr_2d_arm_sim.sim_arm_moveit import Gripper2D, Planner2D
else:
    from hlpr_manipulation_utils.manipulator import Gripper
    from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt

default_save_dir = os.path.normpath(os.path.expanduser("~/test_bagfiles"))

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def save():
    has_name = False
    while not has_name:
        print "Please enter a bagfile name to save to."
        filename = raw_input()

        if filename[-4:]==".pkl":
            raise NotImplementedError("can't save to pkl yet")
        # elif filename[-4:]==".bag":
        #     filename = filename
        else:
            filename = filename+".bag"

        full_save_path = os.path.join(default_save_dir, filename)
        if os.path.isfile(full_save_path):
            print "This file already exists. Do you want to overwrite it (y/n)?"
            resp = raw_input()
            if resp.lower() == 'y':
                save_file = filename
                has_name = True
            elif resp.lower() == 'n':
                pass
            else:
                print "Invalid response."
        else:
            has_name = True
            save_file = filename

    print("I will save the file to " + full_save_path)
    k.write_bagfile(save_file)


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--tidy", help="clear the terminal after each keyframe", action="store_true")
    args = parser.parse_args()
    if args.tidy:
        print("using tidy mode")
        tidy = args.tidy
    else:
        tidy = False

    rospy.init_node("kt_api_testing", disable_signals=True)

    load_file = None
    filename = None

    if tidy:
        clear_screen()
    # where to load
    has_action = False
    action = None
    while action is None:
        print("Would you like to [o]pen a bag file or create a [n]ew one?")
        provided_action = raw_input()
        if provided_action in ["o", "n"]:
            action = provided_action
            break
        else:
            print("Please type either 'o' or 'n'.\n---")

    if action == "o":
        while load_file is None:
            print("enter name of bag file to open:")
            filename = raw_input()

            if filename[-4:]==".pkl":
                raise NotImplementedError("pkl file exists with this name")
            else:
                filename = filename + ".bag"

            full_load_path = os.path.join(default_save_dir, filename)
            if os.path.isfile(full_load_path):
                has_name = True
                load_file = full_load_path
                print("Loading from {}".format(full_load_path))
            else:
                print("the file {} does not exist.\n---".format(full_load_path))
    elif action == "n":
        print("new creating new ")
    else:
        # should never get here
        assert(False)


    if os.environ["ROBOT_NAME"]=="2d_arm":
        k = KTInterface(default_save_dir,Planner2D("/sim_arm/joint_state", "/sim_arm/move_arm"), Gripper2D("/sim_arm/gripper_state","/sim_arm/gripper_command"))
    else:
        k = KTInterface(default_save_dir,ArmMoveIt(), Gripper())

    freezer = rospy.ServiceProxy('freeze_frames', FreezeFrame)
        
    rospy.sleep(0.5)
    k.release_arm()

    print "end effector link:" + k.planner.group[0].get_end_effector_link()
    print "-"*60


    if load_file is not None:
        k.load_bagfile(load_file)
        print "Press enter to move to the first pose in the loaded bagfile. Type 'n' to prevent this."
        resp = raw_input()
        if resp.lower() != 'n':
            k.move_to_start()
            k.move_to_keyframe(k.segment_pointer)
        k.initialize(is_joints=False)

    else:
        valid = False
        is_joints = None
        while True:
            if tidy:
                clear_screen()
            print "Type 'j' to save starting pose as a joint pose"
            print "Type 'e' to save starting pose as an eef pose"
            try:
                r = raw_input()
            except KeyboardInterrupt:
                exit()
            if r=='j':
                is_joints = True
                break
            elif r=='e':
                is_joints = False
                break

        print "Move the arm to the desired starting pose and hit enter"
        raw_input()

        k.initialize(is_joints=is_joints)
        k.record_keyframe()

    pause_tf_record = False
    while not rospy.is_shutdown():
        if tidy:
            clear_screen()
        print "="*25 + "Robot: " + os.environ["ROBOT_NAME"] + "="*25
        print "Current frames: "
        if len(k.segments) > 0:
            assert(k.first is not None)
        s = k.first
        while s is not None:
            if k.segment_pointer==s:
                pref = " >"
            else:
                pref = "  "
                
            print pref, s
            s = s.next_seg
        print
        
        if k.is_joints:
            mode = "joint keyframe"
        else:
            mode = "eef keyframe"
        print "In {} mode. TF recording is {}.".format(mode, not pause_tf_record)
        print "<Enter>: add a keyframe"
        print "'d': delete current keyframe"
        print
        print "'n': move to the next keyframe"
        print "'p': move to previous keyframe"
        print "'s': move to the start"
        print "'e': move to the end"
        print "'fn': force move to the next keyframe"
        print "'fp': force move to the previous keyframe"
        print "'h': move to current keyframe"
        print
        print "'r': release the arm"
        print "'l': lock the arm"
        print "'o': open gripper"
        print "'c': close gripper"
        print "'j': toggle joint keyframe mode"
        print "'a': toggle freezing of tf"
        print "'w': write to a bag"
        print "'q': quit (with optional write)"
        print "-"*60
        k.print_current_pose()
        print "-"*60

        try:
            r = raw_input()
        except KeyboardInterrupt:
            print "Quitting..."
            exit()
            break
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
            k.close_gripper()
        elif r=='o':
            k.open_gripper()
        elif r=='a':
            try:
                freezer(FreezeFrameRequest.TOGGLE)
            except rospy.ServiceException as e:
                print "Couldn't freeze frames; is freeze frame started from hlpr_manipulation utils?"
        elif r=='w':
            save()
        elif r=='q':
            exit_cli = False
            exit_loop = False
            while not exit_loop:
                print("would you like to save before exiting? [y]es/[n]o/[c]ancel")
                resp = raw_input()
                if resp not in ["y", "n", "c"]:
                    continue
                else:
                    exit_loop = True

                if resp == "y":
                    save()
                    exit_loop = True
                    exit_cli = True
                elif resp == "n":
                    exit_loop = True
                    exit_cli = True
                elif resp == "c":
                    exit_loop = True
                    exit_cli = False
                else:
                    # should not get here
                    assert(False)
            if exit_cli:
                break
        elif r =='fn':
        	success = k.move_forward()
        	if not success and not k.segment_pointer.next_seg is None:
        		k.segment_pointer=k.segment_pointer.next_seg
		elif r=='fp':
			success = k.move_backward()
			if not success and not k.segment_pointer.prev_seg is None:
				k.segment_pointer=k.segment_pointer.prev_seg
        else:
            k.write_kf(r)
        
    k.end()
    k.stop_tf_threads()
