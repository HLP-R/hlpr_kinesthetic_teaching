#!/usr/bin/env python

# Copyright (c) 2016, Diligent Droids
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
# * Neither the name of hlpr_simulator nor the names of its
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

# Author: Vivian Chu, vchu@diligentdroids.com

"""
basic_kinesthetic_interaction.py

This extends the kinesthetic_interaction class to do a very simple set of
commands for writing trajectories to a file
"""
import roslib
import rospy

from std_msgs.msg import String
from kinesthetic_interaction import KinestheticInteraction
from hlpr_record_demonstration.demonstration import Demonstration
from hlpr_kinesthetic_interaction.jaco_arm import Arm

class BasicKinestheticInteraction(KinestheticInteraction):

    OK = "OK"
    ERROR_WRITE = "Did not write"
    ERROR_START = "Did not start"
    ERROR_DEMO = "Demonstration not ready. Did the demo server finish loading?" 

    def __init__(self, spin=True, node_name=None, arm_class=Arm, verbose=True, is7DOF=False):

        if(node_name):
            self.node_name = node_name
        else:
            self.node_name = "kinesthetic_interaction"

        # Initialize the node
        rospy.init_node(self.node_name, anonymous=False) # only one at a time

        # Initialize the arm
        self.arm_class = arm_class

        super(BasicKinestheticInteraction, self).__init__(verbose=verbose, is7DOF=is7DOF)

        # Initialize demonstration
        self.demo = None
        self.demo = Demonstration()

        if spin:
            rospy.spin()

    """Simple helper to speak when successful or failed"""
    def _speech_helper(self, flag, success, fail):
        if flag:
            self.speech.say(success)
        else:
            self.speech.say(fail)


    ## all of the functions that need to be filled in with your own behaviors  

    def apply_hand_action(self, cmd, hand):
        rospy.loginfo("Received command: %s", cmd)
        if self.demo is None:
            rospy.logerr(self.ERROR_DEMO)
        else:
            status = self.demo.write_keyframe() # We write a keyframe when the hand opens or closes
            #self._speech_helper(status, self.OK, self.ERROR_WRITE)

    def apply_arm_action(self, cmd, arm):
        rospy.loginfo("Received command: %s", cmd)

    def demonstration_start(self, cmd):
        if self.demo is None:
            rospy.logerr(self.ERROR_DEMO)
        else:
            # Start KF recording
            self.demo.init_demo() # can add custom names if you like in this function
            status = self.demo.start_keyframe()
            self._speech_helper(status, self.OK, self.ERROR_START)

    def demonstration_keyframe(self, cmd):
        if self.demo is None:
            rospy.logerr(self.ERROR_DEMO)
        else:
            status = self.demo.write_keyframe()
            self._speech_helper(status, self.OK, self.ERROR_WRITE)

    def demonstration_end(self, cmd):
        if self.demo is None:
            rospy.logerr(self.ERROR_DEMO)
        else:
            status = self.demo.stop_recording()
            self._speech_helper(status, "That was demo number "+str(self.demo.demo_num), self.ERROR_WRITE)

    def demonstration_start_trajectory(self, cmd):
        if self.demo is None:
            rospy.logerr(self.ERROR_DEMO)
        else:
            # Start trajectory recording
            self.demo.init_demo() # can add custom names if you like in this function
            status = self.demo.start_trajectory()
            self._speech_helper(status, self.OK, self.ERROR_START)
 
    def demonstration_end_trajectory(self, cmd):
        if self.demo is None:
            rospy.logerr(self.ERROR_DEMO)
        else:
            status = self.demo.stop_recording()
            self._speech_helper(status, "That was demo number "+str(self.demo.demo_num), self.ERROR_WRITE)

    
if __name__== "__main__":
    kinesthetic_interaction = BasicKinestheticInteraction()

