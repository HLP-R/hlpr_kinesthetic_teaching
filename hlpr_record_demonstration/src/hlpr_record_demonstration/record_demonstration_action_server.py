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
# * Neither the name of hlpr_kinesthetic_teaching nor the names of its
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

# Author: Andrea Thomaz, athomaz@diligentdroids.com
# Edited 8/8/16: Vivian Chu, vchu@diligentdroids.com - any topics (yaml)


import rospy
import rospkg
import actionlib
import yaml
import time
import rosbag
import string

from hlpr_record_demonstration.msg import RecordKeyframeDemoAction, RecordKeyframeDemoGoal, RecordKeyframeDemoResult, RecordKeyframeDemoFeedback
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# RecordKFDemoAction:
#  this is an action server that will record a single keyframe demo each time it is initiated.  
#  once the action is started, keyframes are triggered with the message \record_demo_frame 
#  a bag is created with the name passed in, each keyframe writes data to bag, bag closed on last kf

class RecordKFDemoAction(object):

  # Current funcionality
  #    1 = Keyframe demo: Go Here  
  #    2 = Keyframe or Traj demo: End Here
  KF = 0
  DEMO_END = 1

  demo_done = False
  rec_whole_trajectory = False
  as_started = False
  bagfile_name = String()

  keyframe_count = 0
  feedback = RecordKeyframeDemoFeedback()
  result = RecordKeyframeDemoResult()

  def __init__(self):

    # Initialize the note
    rospy.loginfo("Initializing demonstration recording action server")
    rospy.init_node('record_keyframe_demo_action_server')
    self.server = actionlib.SimpleActionServer('record_keyframe_demo', RecordKeyframeDemoAction, execute_cb=self.do_record_keyframe_demo, auto_start=False)

    # Load the topics and their message types from a yaml file
    default_yaml_loc = rospkg.RosPack().get_path('hlpr_record_demonstration') + '/data/topics.yaml'
    default_traj_rate = 100
    self.yaml_file_loc = rospy.get_param("~yaml_loc", default_yaml_loc)
    self.traj_record_rate = rospy.get_param("~traj_record_rate", default_traj_rate)

    # Dictionary to store data in
    self.msg_store = dict()
    self.data_types = dict()

    # Pull what topics we want to record from yaml file
    for data in yaml.load_all(file(self.yaml_file_loc, 'r')):
      self.data_types[data['topic']] = data['msg_type']

    # Flag for whether we're in the middle of recording a bag file
    self.recording = False

    # Setup listeners for all of the topics we want to record
    for topic in self.data_types.keys():
      # Cycle through each topic and add each callback programatically
      listener = rospy.Subscriber(topic, eval(self.data_types[topic]), self.listener_cb, callback_args=topic, queue_size=10)

    # Start the action server
    self.server.start()

    # Add callback for keyframes
    self.listen_record_msgs = rospy.Subscriber('record_demo_frame', Int32, self.keyframe_callback, queue_size=1)
    rospy.loginfo("Finished initializing demonstration recording action server")
   

  # generic subscriber that writes messages to a dictionar
  # WARN: Unclear if this collides... (Might need some locks)
  def listener_cb(self, msg, topic):
    self.msg_store[topic] = msg

  #TBD come up with a convention for trigger values, etc.]
  # Current funcionality
  def keyframe_callback(self, msg):

    print msg
    # Check if we've received a goal to start recording
    if self.recording is True:
      
      if msg.data == RecordKFDemoAction.KF: #rec another keyframe
        self.write_demo_data()
        rospy.loginfo('Recording keyframe: %d', self.keyframe_count)
      elif msg.data == RecordKFDemoAction.DEMO_END: #end message
        self.write_demo_data()
        self.demo_done = True
        self.recording = False
        rospy.loginfo('Finished recording. Number frames: %d', self.keyframe_count)
      elif msg.data != 2: 
        print 'record_demo_frame should be called with 0-3'
        return

  # Write current state to the open bag
  def write_demo_data(self):    
    # Cycle through the different messages stored away
    for topic in self.msg_store:
      self.bag.write(topic, self.msg_store[topic])
    self.keyframe_count += 1

  def _end_bag(self):
    if self.demo_done is True:
      self.bag.close() 
      rospy.loginfo("Rosbag file closed")
     
  #start performing the record_keyframe_demo action
  #goal includes the bagfile name to use for this demo
  def do_record_keyframe_demo(self, goal):

    # Reset some keyframe counters now that we have a new goal
    self.keyframe_count = 0
    self.demo_done = False
    self.bag = rosbag.Bag(goal.bag_file_name, 'w')
    self.rec_whole_trajectory = goal.trajectory #check if we're doing a trajectory
    rospy.loginfo("Starting to record demonstration, bag file is: %s", goal.bag_file_name)

    # Setup the first keyframe or trajectory
    if self.rec_whole_trajectory:
      rospy.loginfo("In trajectory recording mode")

      # Set the default rate
      self.rate = rospy.Rate(self.traj_record_rate)
    else:
      rospy.loginfo('Recording first keyframe now! starting rosbag')
      self.write_demo_data()

    self.recording = True

    while self.recording is True:
      # Check if we've been pre-empted
      if self.server.is_preempt_requested():
        self.result.num_keyframes = self.keyframe_count # Store away keyframes
        self.server.set_preempted(self.result, "Record keyframe demo preempted")  
        self._end_bag()
        rospy.loginfo("Goal was pre-empted")
        return

      # Check if we're doing a trajectory vs. keyframe
      elif self.rec_whole_trajectory is True:
        self.write_demo_data()
        self.rate.sleep()
      
      elif self.keyframe_count > 0: 
        self.feedback.num_keyframes = self.keyframe_count
        self.server.publish_feedback(self.feedback)
        #time.sleep(.5)  #throttle feedback messages 

    # If we've reach here that means self.recording is false
    self._end_bag()
   
    # Store the results
    self.result.num_keyframes = self.keyframe_count
    self.server.set_succeeded(self.result, "Record Keyframe Demo completed successfully")
    return 

if __name__ =='__main__':
  RecordKFDemoAction()
  rospy.spin()


