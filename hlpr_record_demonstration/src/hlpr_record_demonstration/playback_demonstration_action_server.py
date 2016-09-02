#! /usr/bin/env python

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
# Edited 8/9/2016: Vivian Chu, vchu@diligentdroids.com - works with EEF

import sys
import copy
import rospy
import actionlib
import time
import rosbag
import string
import os
import moveit_commander

from hlpr_manipulation_utils.manipulator import *
from hlpr_manipulation_utils.arm_moveit import *

from hlpr_record_demonstration.msg import PlaybackKeyframeDemoAction, PlaybackKeyframeDemoGoal, PlaybackKeyframeDemoResult, PlaybackKeyframeDemoFeedback
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# PlaybackKFDemoAction:
# This is an action server that executes the demo stored in the bagfile given in the goal
# The goal should also specify whether or not this is to be an eef_only target, or full arm joints target
# Uses MoveIt to get a plan

class PlaybackKFDemoAction(object):

  feedback = PlaybackKeyframeDemoFeedback()
  result = PlaybackKeyframeDemoResult()

  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.server = actionlib.SimpleActionServer('playback_keyframe_demo', PlaybackKeyframeDemoAction, execute_cb=self.do_playback_keyframe_demo, auto_start=False)
    self.server.start()
    self.arm_planner = ArmMoveIt()
    self.manipulator = Manipulator()

    # Get joints for the arm from the arm group that we want to plan with
    self._arm_joints = self.arm_planner.group[0].get_active_joints()

    # Set number of keyframes we'll execute
    self.KEYFRAME_THRESHOLD = 30

  def sendPlan(self,plannedTra):
    traj_goal = FollowJointTrajectoryGoal()
    traj_goal.trajectory = plannedTra.joint_trajectory
    self.manipulator.arm.smooth_joint_trajectory_client.send_goal(traj_goal)#sendWaypointTrajectory(traj_goal)
    self.manipulator.arm.smooth_joint_trajectory_client.wait_for_result()   
    return self.manipulator.arm.smooth_joint_trajectory_client.get_result() 

  def _get_arm_joint_values(self, msg):

    # Cycle through the active joints and populate
    # a dictionary for those values
    joint_values = dict()
    for joint_name in self._arm_joints:
        # Find that joint name in msg
        idx = msg.name.index(joint_name)
 
        # Populate the joint message in a dictionary
        joint_values[joint_name] = msg.position[idx]
 
    return joint_values


  #start performing the playback_keyframe_demo action
  #goal includes the bagfile name for this demo
  def do_playback_keyframe_demo(self, goal):
    rospy.loginfo("Received playback demo")
    ## input: keyframe demo goal has bagfile name and eef_only bool
    self.start_time = time.time()

    # Check if the bag file is valid
    # Example path if necessary
    bag_path = os.path.expanduser(goal.bag_file_name)
  
    if (not os.path.isfile(bag_path)):
      error_msg = "Playback bagfile does not exist: %s" % bag_path 
      self.server.set_aborted(self.result, error_msg)
      rospy.logerr(error_msg)
      return
    else:
      self.bag = rosbag.Bag(bag_path)

    # Check the number of playback - and warn/stop if we have more than X number of keyframes
    if self.bag.get_message_count() > self.KEYFRAME_THRESHOLD:
      error_msg = "Playback Keyframe Demo aborted due to too many frames: %d" % self.bag.get_message_count()
      self.server.set_aborted(self.result, error_msg)
      rospy.logerr(error_msg)
      return

    keyframe_count = 0
    while not self.server.is_preempt_requested():
 
      for topic, msg, t in self.bag.read_messages(topics=[goal.target_topic]):
        # Check if we need to convert the msg into joint values
        if goal.eef_only:
          # Ask the arm planner to plan for that joint target from current position
          pt = Pose(msg.position, msg.orientation)
          plan = self.arm_planner.plan_poseTargetInput(pt)

        else:
          # Pull out the joint values for the arm from the message
          pts = self._get_arm_joint_values(msg)
          
          # Ask the arm planner to plan for that joint target from current position
          plan = self.arm_planner.plan_jointTargetInput(pts)

        # Check if we have a valid plan
        if plan == None or len(plan.joint_trajectory.points) < 1:
          print "Error: no plan found"
        else:
          rospy.loginfo("Executing Keyframe: %d" % keyframe_count)
          self.sendPlan(plan)
          keyframe_count+=1

      self.bag.close()
      
      self.result.time_elapsed = rospy.Duration.from_sec(time.time() - self.start_time)
      complete_msg = "Playback Keyframe Demo completed successfully"
      self.server.set_succeeded(self.result, complete_msg)
      rospy.loginfo(complete_msg)
      return
    
    self.result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    self.server.set_preempted(self.result, "playback keyframe demo preempted")  
    self.bag.close()      
    print 'preempted'
    return


if __name__ =='__main__':
  rospy.init_node('playback_keyframe_demo_action_server')
  PlaybackKFDemoAction()
  rospy.spin()
