#! /usr/bin/env python

import sys
import copy
import rospy
import actionlib
import time
import rosbag
import string
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg

#this is pointing to the hello world manipulation stuff...so you need to have hello_world in your catkin_ws...needs to point to hlpr_manipulation! sorry!
import manipulation
from manipulation.manipulator import *
from manipulation.arm_moveit import *

from hlpr_kinesthetic_teaching.msg import PlaybackKeyframeDemoAction, PlaybackKeyframeDemoGoal, PlaybackKeyframeDemoResult, PlaybackKeyframeDemoFeedback
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState

# PlaybackKFDemoAction:
# This is an action server that executes the demo stored in the bagfile given in the goal
# The goal should also specify whether or not this is to be an eef_only target, or full arm joints target
# Uses MoveIt to get a plan
# TBD: calling moveit execute on the plan doesn't work, why?
# TBD: also using wpi_jaco stuff through ArmMoveIt and Manipulator doesn't work, why? 

class PlaybackKFDemoAction(object):

  feedback = PlaybackKeyframeDemoFeedback()
  result = PlaybackKeyframeDemoResult()

  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.server = actionlib.SimpleActionServer('playback_keyframe_demo', PlaybackKeyframeDemoAction, execute_cb=self.do_playback_keyframe_demo, auto_start=False)
    self.server.start()
    self.arm_planner = ArmMoveIt()
    self.manipulator = Manipulator()

  def sendPlan(self,plannedTra):
    traj_goal = FollowJointTrajectoryGoal()
    traj_goal.trajectory = plannedTra.joint_trajectory
    self.manipulator.arm.smooth_joint_trajectory_client.send_goal(traj_goal)#sendWaypointTrajectory(traj_goal)
    self.manipulator.arm.smooth_joint_trajectory_client.wait_for_result()   
    return self.manipulator.arm.smooth_joint_trajectory_client.get_result() 


  #start performing the playback_keyframe_demo action
  #goal includes the bagfile name for this demo
  def do_playback_keyframe_demo(self, goal):
    ## input: keyframe demo goal has bagfile name and eef_only bool
    self.start_time = time.time()
    self.bag = rosbag.Bag(goal.bag_file_name)

    while not self.server.is_preempt_requested():
      if goal.eef_only == True:
        for topic, msg, t in self.bag.read_messages(topics=['eef_pose']):
          print msg.data
          self.goto_eefTargetInput(msg.data)
      else:
        for topic, msg, t in self.bag.read_messages(topics=['joint_states']):
          plan = self.arm_planner.plan_jointTargetInput(msg)
          self.sendPlan(plan)

      self.bag.close()
      
      self.result.time_elapsed = rospy.Duration.from_sec(time.time() - self.start_time)
      self.server.set_succeeded(self.result, "Playback Keyframe Demo completed successfully")
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
