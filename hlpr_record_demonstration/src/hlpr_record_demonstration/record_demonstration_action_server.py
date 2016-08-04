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

# Author: Andrea Thomaz, athomaz@diligentdroids.com


import rospy
import actionlib
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
#
# ---TBD: Currently this code assumes someone is publishing topics joint_states and eef_pose
#        topics to record should be defined by specific instantiations that inherit from this class?

class RecordKFDemoAction(object):

  # Current funcionality
  #    0 = Start Trajectory Demo 
  #    1 = Keyframe demo: Start Here 
  #    2 = Keyframe demo: Go Here  
  #    3 = Keyframe or Traj demo: End Here
  TRAJ_START = 0
  KF_START = 1
  KF = 2
  DEMO_END = 3

  demo_done = False
  rec_whole_trajectory = False
  as_started = False
  bagfile_name = String()

  current_jt_st = JointState()
  current_eef_st = Pose()
  keyframe_count = 0
  feedback = RecordKeyframeDemoFeedback()
  result = RecordKeyframeDemoResult()

  def __init__(self):
    rospy.loginfo("Initializing demonstration recording action server")
    self.server = actionlib.SimpleActionServer('record_keyframe_demo', RecordKeyframeDemoAction, execute_cb=self.do_record_keyframe_demo, auto_start=False)
    self.server.start()
    self.listen_record_msgs = rospy.Subscriber('record_demo_frame', Int32, self.keyframe_callback)
    self.listen_jt = rospy.Subscriber('joint_states', JointState, self.jt_st_callback)
    self.listen_eef = rospy.Subscriber('eef_pose', Pose, self.eef_callback)
    rospy.loginfo("Finished initializing demonstration recording action server")
    
  #subscriber for joint_state topic, update most recent data
  def jt_st_callback(self, msg):
    self.current_jt_st = msg

  #subscriber for eef_pose topic, update most recent data
  def eef_callback(self, msg):
    self.current_eef_st = msg

  #keyframe message triggers recording of \joint_state and \eef_pose
  #Important...if this callback happens before do_record_keyframe_demo(), it will wait
  #TBD come up with a convention for trigger values, etc.]
  # Current funcionality
  #    0 = Start Trajectory Demo 
  #    1 = Keyframe demo: Start Here 
  #    2 = Keyframe demo: Go Here  
  #    3 = Keyframe or Traj demo: End Here
  def keyframe_callback(self, msg):
    #if the first keyframe message is sent before the do_record_keyframe_demo,  
    #then we should wait, so we know what bag file to open
    while self.as_started == False:
      continue  

    if msg.data == RecordKFDemoAction.KF_START:  #start message keyframe demo
      print 'recording first keyframe now! starting rosbag'
      self.write_demo_data()
    elif msg.data == RecordKFDemoAction.KF: #rec another keyframe
      self.write_demo_data()
    elif msg.data == RecordKFDemoAction.TRAJ_START: #start message traj demo
      print 'starting trajectory demo, starting rosbag'
      self.rec_whole_trajectory = True
    elif msg.data == RecordKFDemoAction.DEMO_END: #end message
      print 'recording last keyframe now!'
      self.write_demo_data()
      self.demo_done = True
      self.rec_whole_trajectory = False
    elif msg.data != 2: 
      print 'record_demo_frame should be called with 0-3'
      return

 
  #write current state to the open bag
  #TBD: don't hard code what data is saved
  def write_demo_data(self):    
    self.bag.write('joint_states', self.current_jt_st)
    self.bag.write('eef_pose', self.current_eef_st)
    self.keyframe_count += 1
  
  #start performing the record_keyframe_demo action
  #goal includes the bagfile name to use for this demo
  def do_record_keyframe_demo(self, goal):
    print 'starting record keyframe demo action, bag file is'
    self.keyframe_count = 0
    self.demo_done = False
    self.write_whole_trajectory = False
    self.bag = rosbag.Bag(goal.bag_file_name, 'w')
    print goal.bag_file_name
    self.rate = rospy.Rate(100)
    self.as_started = True

    while self.demo_done == False:
      if self.server.is_preempt_requested():
        self.result.num_keyframes = self.keyframe_count
        self.server.set_preempted(self.result, "record keyframe demo preempted")  
        if self.demo_done == False:
          self.bag.close()             
          print 'preempted'       
          return
      elif self.rec_whole_trajectory == True:
        self.write_demo_data()       
        self.rate.sleep()
      elif self.keyframe_count > 0: #wait to start feedback after first keyframe 
        self.feedback.num_keyframes = self.keyframe_count
        self.server.publish_feedback(self.feedback)
        time.sleep(.5)  #throttle feedback messages 

    self.bag.close()
    self.as_started = False
    print 'bag file closed'
    self.result.num_keyframes = self.keyframe_count
    self.server.set_succeeded(self.result, "Record Keyframe Demo completed successfully")
    return
    

if __name__ =='__main__':
  rospy.init_node('record_keyframe_demo_action_server')
  RecordKFDemoAction()
  rospy.spin()


