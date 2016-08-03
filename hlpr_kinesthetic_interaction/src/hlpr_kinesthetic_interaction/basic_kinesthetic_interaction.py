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

class BasicKinestheticInteraction(KinestheticInteraction):

    def __init__(self):
        # Initialize the node
        super(BasicKinestheticInteraction, self).__init__()
        rospy.spin()


    ## all of the functions that need to be filled in with your own behaviors  

    def apply_hand_action(self, cmd, hand):
        print "hand"

    def apply_arm_action(self, cmd, arm):
        print "arm"

    def demonstration_start(self, cmd):
        print "start"

    def demonstration_keyframe(self, cmd):
        print "keyframe"

    def demonstration_end(self, cmd):
        print "end"

    def demonstration_start_trajectory(self, cmd):
        print cmd
 
    def demonstration_end_trajectory(self, cmd):
        print cmd

    
if __name__== "__main__":
    kinesthetic_interaction = BasicKinestheticInteraction()

