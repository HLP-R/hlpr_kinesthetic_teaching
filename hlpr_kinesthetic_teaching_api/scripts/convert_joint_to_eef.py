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
import rosbag
import sys
import os
from shutil import copyfile

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from kinova_msgs.srv import Start, Stop
from hlpr_manipulation_utils.manipulator import Gripper




if __name__=="__main__":
    rospy.init_node("joint_to_eef_converter")
    
    bags = filter(lambda s: ".bag" in s, sys.argv)
    a = ArmMoveIt(planning_frame="j2s7s300_link_base")
    a.group[0].set_end_effector_link("j2s7s300_ee_link")


    for bag_name in bags:
        inbag = rosbag.Bag(bag_name, "r")
        outbag = rosbag.Bag(bag_name[:-4]+"_eef.bag", "w")
        
        for topic, msg, time in inbag.read_messages():
            outbag.write(topic,msg,time)
            
            if "joint_states" in topic:
                joints = dict(zip(msg.name,msg.position))
                joints = {k:joints[k] for k in filter(lambda s: "j2s7s300" in s, joints.keys())}
                eef =  a.get_FK(state = a.state_from_joints(joints))[0].pose
                outbag.write("/eef_pose", eef, time)

                
        inbag.close()
        outbag.close()
    
