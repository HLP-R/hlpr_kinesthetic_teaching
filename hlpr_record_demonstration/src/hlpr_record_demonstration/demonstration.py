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

'''
Demonstration.py

Simple container class that keeps track of the basics for demonstration recording
'''

import rospy
import os
import time
import actionlib
from std_msgs.msg import Int32
from hlpr_record_demonstration.msg import RecordKeyframeDemoAction, RecordKeyframeDemoGoal, RecordKeyframeDemoResult, RecordKeyframeDemoFeedback
from hlpr_record_demonstration.record_demonstration_action_server import RecordKFDemoAction

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class Demonstration():

    def __init__(self):

        # Setup some basic default variables for storage
        self.DEFAULT_DATA_FOLDER = "data" 
        self.DEFAULT_DIR_LOC = "~/" # Home
        self.DEFAULT_DATA_PREFIX = "demo"

        # Pull from rosparam if loaded
        self.data_folder_name = get_param("data_folder_name", self.DEFAULT_DATA_FOLDER)
        self.data_folder_loc = get_param("data_folder_loc", self.DEFAULT_DIR_LOC)
        self.data_prefix = get_param("data_prefix", self.DEFAULT_DATA_PREFIX)
        
        # Create path to data storage
        self.data_location = os.path.expanduser(os.path.join(self.data_folder_loc,self.data_folder_name))

        # Create directories if needed
        self._ensure_dir(self.data_location)

        # Determine what demo number we're on
        self.demo_num = self._get_num_files(self.data_location)

        # Init empty filename
        self.filename = ""

        # Flag to keep tracking of recording
        self.recording = False
 
        # Initialize the demonstration action server
        self.record_pub = rospy.Publisher('record_demo_frame', Int32, queue_size=1)
        self.record_client = actionlib.SimpleActionClient('record_keyframe_demo', RecordKeyframeDemoAction)
        rospy.logwarn("Waiting for record keyframe demo server to load")
        self.record_client.wait_for_server() 
        rospy.logwarn("Record keyframe demo server loaded")

    """When a new demonstration bag file needs to be written this function
    generates a new name for it
    """
    def init_demo(self, custom_name = None, timestamp = False):

        # Check if we have a custom demo name
        if custom_name == None:
            self.filename = "_".join((self.data_prefix, "%02d" % self.demo_num))
        else:
            self.filename = custom_name
      
        # Check if we want a custom time stamp to the file 
        if (timestamp):
            self.filename += "_"+time.strftime("%Y-%m-%dT%H%M%S") 

        # Append the bag file name
        self.filename += ".bag"
        self.filename = os.path.join(self.data_location, self.filename)

        # Update what demo number we're on
        self.demo_num +=1


    """Start a trajectory demonstration. 
    returns True/False depending on if we actually publish a command
    """
    def start_trajectory(self):
        status = self._send_client_goal(traj=True)
        return status

    """End a demonstration (keyframe or trajectory)
    returns True/False depending on if we actually publish a command
    """
    def stop_recording(self):
        self.record_pub.publish(RecordKFDemoAction.DEMO_END)
        status = self.record_client.wait_for_result()
        if self.recording and status: # check for result
            #print('[Result] State: %d'%(self.record_client.get_state()))
            #print('[Result] Status: %s'%(self.record_client.get_goal_status_text()))
            print('[Result] Num Poses Recorded: %f'%(self.record_client.get_result().num_keyframes))
            self.recording = False
            return True
        else:
            return False
 
    """Start a keyframe demonstration"""
    def start_keyframe(self):
        status = self._send_client_goal()
        return status
    
    """Write a keyframe"""
    def write_keyframe(self):
        if self.recording:
            self.record_pub.publish(RecordKFDemoAction.KF)
        return self.recording

    """Sets up a client goal with the proper bag name"""
    def _send_client_goal(self, traj=False):
        if self.recording: # Already recording
            return False
        goal = RecordKeyframeDemoGoal()
        goal.bag_file_name = self.filename
        goal.trajectory=traj
        self.record_client.send_goal(goal, feedback_cb=self._feedback_cb)
        self.recording = True
        return True
         
    """Simple function that checks how many demos we've written"""
    def _get_num_files(self, dir_path):
        return len(os.listdir(dir_path))

    """Simple function that makes sure the data directory specified exists"""
    def _ensure_dir(self, f): 
        d = os.path.dirname(f)
        if not os.path.exists(d):
            os.makedirs(d)

    def _feedback_cb(self, feedback):
        #print('[Feedback] num keyframes: %f'%feedback.num_keyframes)
        pass
 
        
if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("demonstration_node", anonymous=False)
    rospy.loginfo("Initializing the demonstration node")

    demo = Demonstration()
    demo._init_demo()



