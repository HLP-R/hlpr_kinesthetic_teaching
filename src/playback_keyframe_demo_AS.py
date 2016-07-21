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

from my_awesome_code.msg import PlaybackKeyframeDemoAction, PlaybackKeyframeDemoGoal, PlaybackKeyframeDemoResult, PlaybackKeyframeDemoFeedback
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState

# PlaybackKFDemoAction:

class PlaybackKFDemoAction(object):

  feedback = PlaybackKeyframeDemoFeedback()
  result = PlaybackKeyframeDemoResult()

  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.server = actionlib.SimpleActionServer('playback_keyframe_demo', PlaybackKeyframeDemoAction, execute_cb=self.do_playback_keyframe_demo, auto_start=False)
    self.server.start()
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group = [moveit_commander.MoveGroupCommander("arm"), moveit_commander.MoveGroupCommander("gripper"),
                  moveit_commander.MoveGroupCommander("head")]
    
  def goto_jointTargetInput(self,target_joint):
    ## input: target joint angles (JointState) of the robot
    self.group[0].set_planner_id("RRTConnectkConfigDefault")

    print 'setting target joint positions'
    print target_joint
    print 'planning frame:'
    print self.group[0].get_planning_frame()
    self.group[0].set_joint_value_target(target_joint)
    planAns=self.group[0].plan()
    self.group[0].execute(planAns)  

  def goto_eefTargetInput(self,target_eef):
    ## input: target eef (Pose) of the robot
    self.group[0].set_pose_target(target_eef)
    self.group[0].set_planner_id("RRTConnectkConfigDefault")
    planAns=self.group[0].plan()
    self.group[0].execute(planAns)


  #start performing the playback_keyframe_demo action
  #goal includes the bagfile name for this demo
  def do_playback_keyframe_demo(self, goal):
    self.start_time = time.time()
    self.bag = rosbag.Bag(goal.bag_file_name)

    while not self.server.is_preempt_requested():
      if goal.eef_only == True:
        for topic, msg, t in self.bag.read_messages(topics=['eef_pose']):
          print msg.data
          self.goto_eefTargetInput(msg.data)
      else:
        for topic, msg, t in self.bag.read_messages(topics=['joint_states']):
          self.goto_jointTargetInput(msg)

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
