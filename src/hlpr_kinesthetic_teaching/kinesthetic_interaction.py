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
kinesthetic_interaction.py

This is an abstract class that controls speech interactions with the robot.
The basic low-level commands are applied in this class and extensions of
this class allow for extra functionality to be added to each speech command
as well as additional commands
"""

import roslib
import rospy
from abc import ABCMeta, abstractmethod

from std_msgs.msg import String

from hlpr_speech_recognition import speech_listener
from hlpr_speech_synthesis import speech_synthesizer
from hlpr_kinesthetic_teaching.srv import KinestheticInteract
from hlpr_manipulation_utils.manipulator import Gripper
from wpi_jaco_msgs.srv import GravComp

class KinestheticInteraction:

    __metaclass__ = ABCMeta
    # TODO: REMOVE THIS AND PUT SOMEONE GLOBAL?
    RIGHT = 0
    LEFT = 1

    OPEN_HAND = "OPEN_HAND"
    CLOSE_HAND = "CLOSE_HAND"
    OPEN_HAND_LEFT = "OPEN_HAND_LEFT"
    CLOSE_HAND_LEFT = "CLOSE_HAND_LEFT"
    START_GC = "START_GC"
    END_GC = "END_GC"
    KEYFRAME_START = "KEYFRAME_START"
    KEYFRAME = "KEYFRAME"
    KEYFRAME_END = "KEYFRAME_END"
    TRAJ_START =  "TRAJ_START"
    TRAJ_END = "TRAJ_END"
    
    GRAVITY_COMP_SERVICE = "/jaco_arm/grav_comp"

    def __init__(self):

        # Initialize the node
        rospy.init_node("kinesthetic_interaction", anonymous=False) # only one at a time

        # Initialize the listener class for speech and pull out topic
        self.speech_listener = speech_listener.SpeechListener()
        self.sub_topic = self.speech_listener.recog_topic

        # Initialize speech dictionary
        self._init_speech_dictionary()

        # Initialize speech synthesis
        self.speech = speech_synthesizer.SpeechSynthesizer()

        # Set flag for whether we're in kinesthetic mode
        self.active = False # Default is false
    
        # Create a service for kinesthetic mode
        k_service = rospy.Service('kinesthetic_interaction', KinestheticInteract, self.toggleKMode)

        # Get access to the gravity compensation service
        rospy.loginfo("Waiting for gravity compenstation service")
        rospy.wait_for_service(KinestheticInteraction.GRAVITY_COMP_SERVICE)
        self.gravity_comp = rospy.ServiceProxy(KinestheticInteraction.GRAVITY_COMP_SERVICE, GravComp)
        rospy.loginfo("Gravity compenstation service loaded")

        # Initialize the gripper
        self.gripper = Gripper()

        # Initialize callback for speech commands - do at the end to prevent unwanted behavior
        rospy.Subscriber(self.sub_topic, String, self._speechCB, queue_size=1) 

        rospy.loginfo("Finished initializing Kinesthetic Interaction node")
        #rospy.spin()

    def _init_speech_dictionary(self):

        # Use dictionaries to match switch cases to functions
        self.switcher = {

            # Simple test for greetting first
            "GREETING": self._greet,
            "HEAR_CHECK": self._hear_check,
            KinestheticInteraction.OPEN_HAND: self._open_hand,
            KinestheticInteraction.CLOSE_HAND: self._close_hand,
            KinestheticInteraction.OPEN_HAND_LEFT: self._open_hand_left,
            KinestheticInteraction.CLOSE_HAND_LEFT: self._close_hand_left,
            KinestheticInteraction.START_GC: self._start_grav_comp,
            KinestheticInteraction.END_GC: self._end_grav_comp,
            KinestheticInteraction.KEYFRAME_START: self._demonstration_start,
            KinestheticInteraction.KEYFRAME: self._save_keyframe,
            KinestheticInteraction.KEYFRAME_END: self._demonstration_end,
            KinestheticInteraction.TRAJ_START: self._record_all_start,
            KinestheticInteraction.TRAJ_END: self._record_all_end, 
        }

    def toggleKMode(self, req):
        self.active = req.enableKInteract # pull out what the request sent
        rospy.loginfo("Kinesthetic Interaction set to: %s" % self.active)
        return True

    def _speechCB(self, msg):

        # Pull the speech command
        self.last_command = self.speech_listener.get_last_command()

        # Check if we're actively in kinesthetic mode
        if self.active:

            # Get the function from switcher dictionary
            func = self.switcher.get(self.last_command, self._command_not_found)         
       
            # execute the function?
            func()

        else:
            rospy.logwarn("Kinesthetic Mode is inactive currently. Command: %s ignored" % self.last_command)

    def _command_not_found(self):
        rospy.logwarn("Speech command unknown: %s" % self.last_command)

    def add_speech_commands(self, speech_cmd, func):
        self.switcher[speech_cmd] = func

    # These are the commands that are set in advance and correspond to actual robot state changes
    # Extend this class and the functions in the next section 

    def _greet(self):
        self.speech.say("Hello!")
        print "Hello!"

    def _hear_check(self):
        self.speech.say("I heard ya!")
        print "I heard ya!"
        
    def _open_hand(self):
        self.gripper.open()
        self.apply_hand_action(self.last_command, KinestheticInteraction.RIGHT)
        self.speech.say("OK")

    def _close_hand(self):
        self.gripper.close()
        self.apply_hand_action(self.last_command, KinestheticInteraction.RIGHT)
        self.speech.say("OK")

    def _open_hand_left(self):
        self.apply_hand_action(self.last_command, KinestheticInteraction.LEFT)
        self.speech.say("OK")

    def _close_hand_left(self):
        self.apply_hand_action(self.last_command, KinestheticInteraction.LEFT)
        self.speech.say("OK")

    def _start_grav_comp(self):
        response = self.gravity_comp(True)
        if response:
            self.apply_arm_action(self.last_command, KinestheticInteraction.RIGHT)
        else:
            rospy.logerr("Gravity compensation is not active.")
        self.speech.say("OK")

    def _end_grav_comp(self):
        response = self.gravity_comp(False)
        if response:
            self.apply_arm_action(self.last_command, KinestheticInteraction.RIGHT)
        else:
            rospy.logerr("Gravity compensation is still active.")
        self.speech.say("OK")

    def _demonstration_start(self):
        self.demonstration_start(self.last_command)
        self.speech.say("OK")

    def _save_keyframe(self):
        self.demonstration_keyframe(self.last_command)
        self.speech.say("OK")

    def _demonstration_end(self):
        self.demonstration_end(self.last_command)
        self.speech.say("OK")

    def _record_all_start(self):
        self.demonstration_start_trajectory(self.last_command)
        self.speech.say("OK")

    def _record_all_end(self):
        self.demonstration_end_trajectory(self.last_command)
        self.speech.say("OK")

    ## all of the functions that need to be filled in with your own behaviors  

    @abstractmethod
    def apply_hand_action(self, cmd, hand): pass

    @abstractmethod
    def apply_arm_action(self, cmd, arm): pass

    @abstractmethod
    def demonstration_start(self, cmd): pass

    @abstractmethod
    def demonstration_keyframe(self, cmd): pass

    @abstractmethod
    def demonstration_end(self, cmd): pass

    @abstractmethod
    def demonstration_start_trajectory(self, cmd): pass
 
    @abstractmethod
    def demonstration_end_trajectory(self, cmd): pass

    
if __name__== "__main__":
    kinesthetic_interaction = KinestheticInteraction()

