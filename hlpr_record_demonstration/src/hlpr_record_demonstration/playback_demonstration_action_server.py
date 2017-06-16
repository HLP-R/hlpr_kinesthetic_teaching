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
# Edited 8/9/2016: Vivian Chu, vchu@diligentdroids.com - works with EEF

import copy
import os
import string
import sys
import time

import actionlib
import moveit_commander
import rosbag
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from hlpr_manipulation_utils.arm_moveit import *
from hlpr_manipulation_utils.manipulator import *
from hlpr_record_demonstration.msg import (PlaybackKeyframeDemoAction,
                                           PlaybackKeyframeDemoFeedback,
                                           PlaybackKeyframeDemoGoal,
                                           PlaybackKeyframeDemoResult)
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String


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

        self.GRIPPER_THRESHOLD = 0.01 # 1cm threshold for moving

        # Get joints for the arm from the arm group that we want to plan with
        self._arm_joints = self.arm_planner.group[0].get_active_joints()

        # Get the gripper class
        self.gripper = dict()
        self.gripper['right'] = Gripper(prefix='right') # Insert a 2nd gripper here with left if wanted

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

    def gripper_helper(self, gripper_topic, pos):

        # Setup gripper position storage
        self.gripper_pos[gripper_topic] = pos

        # Get right or left
        gripper_side = gripper_topic.split('/')[2].split('_')[0]

        # Check the gripper state initially and set
        self.gripper[gripper_side].set_pos(pos)

        
    #start performing the playback_keyframe_demo action
    #goal includes the bagfile name for this demo
    def do_playback_keyframe_demo(self, goal):
        rospy.loginfo("Received playback demo")
        ## input: keyframe demo goal has bagfile name and eef_only bool
        self.start_time = time.time()

        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Check if the bag file is valid
        # Example path if necessary
        bag_path = os.path.expanduser(goal.bag_file_name)
    
        if (not os.path.isfile(bag_path)):
            error_msg = "Playback bagfile does not exist: %s" % bag_path 
            self.server.set_aborted(self.result, error_msg)
            rospy.logerr(error_msg)
            return
        with rosbag.Bag(bag_path) as bag:
            self.bag = bag

            # Check the number of playback - and warn/stop if we have more than X number of keyframes
            if self.bag.get_message_count() > self.KEYFRAME_THRESHOLD:
                error_msg = "Playback Keyframe Demo aborted due to too many frames: %d" % self.bag.get_message_count()
                self.server.set_aborted(self.result, error_msg)
                rospy.logerr(error_msg)
                return

            keyframe_count = 0

            # Check if we have a gripper topic. If so add it to playback 
            all_topics = self.bag.get_type_and_topic_info().topics.keys()
            playback_topics = [goal.target_topic]
            GRIPPER_TOPIC = 'gripper/stat'
            gripper_topics = [x for x in all_topics if GRIPPER_TOPIC in x] 
            playback_topics.extend(gripper_topics)
            OBJECT_LOCATION_TOPIC = "object_location"
            playback_topics.append(OBJECT_LOCATION_TOPIC)
            gripper_msgs = dict()
            self.gripper_pos = dict()

            while not self.server.is_preempt_requested():

                # Cycle through the topics and messages and store them into list for ordering
                all_messages = dict()
        
                for topic, msg, t in self.bag.read_messages(topics=playback_topics):

                    if topic not in all_messages:
                        all_messages[topic] = []

                    all_messages[topic].append(msg) 
            
                # Pull out the playback topic
                playback_list = all_messages[goal.target_topic]
                for gripper_topic in gripper_topics: # either one or two grippers
                    gripper_msgs[gripper_topic] = all_messages[gripper_topic]

                    # Actually set the gripper value
                    pos = gripper_msgs[gripper_topic][0].requested_position
                    self.gripper_helper(gripper_topic, pos)

                for msg_count in xrange(len(playback_list)):

                    msg = playback_list[msg_count]

                    if goal.zero_marker:
                        zeroMarker = None
                        if not OBJECT_LOCATION_TOPIC in all_messages:
                            rospy.logerr("Playback specified a zero marker but no object locations were found in keyframe #{}".format(msg_count))
                            self.server.set_aborted(text="Object locations missing")
                            return

                        for i, location in enumerate(all_messages[OBJECT_LOCATION_TOPIC][msg_count].objects):
                            if location.label == goal.zero_marker:
                                zeroMarker = location
                                break

                        if not zeroMarker:
                            rospy.logerr("Specified zero marker not found in .bag file in keyframe #{}".format(msg_count))
                            self.server.set_aborted(text="Zero marker not found in .bag file")
                            return

                        try:
                            currentZero = tfBuffer.lookup_transform("map", goal.zero_marker, rospy.Time(0), timeout=rospy.Duration(5)).transform
                        except tf2_ros.LookupException:
                            rospy.logerr("Specified label \"{}\" not found in current scene. Disable locate objects to play back absolute keyframe positions.".format(zeroMarker.label))
                            self.server.set_aborted(text="Specified label not found")
                            return
                        rospy.loginfo("Using zero marker \"{}\" (prob: {:.1%}) with position (x: {:.2f}, y: {:.2f}, z: {:.2f})".format(
                            zeroMarker.label,
                            zeroMarker.probability,
                            zeroMarker.pose.position.x,
                            zeroMarker.pose.position.y,
                            zeroMarker.pose.position.z
                        ))
                        rospy.loginfo("Found corresponding zero marker in current frame (x: {:.2f}, y: {:.2f}, z: {:.2f})".format(
                            currentZero.translation.x,
                            currentZero.translation.y,
                            currentZero.translation.z
                        ))
                    else:
                        rospy.loginfo("No zero marker passed for keyframe #{}".format(msg_count))
                    
                        
                    # Check if we need to convert the msg into joint values
                    if goal.eef_only:
                        # Ask the arm planner to plan for that joint target from current position
                        pt = Pose(msg.position, msg.orientation)

                        if zeroMarker:
                            # Remap the point to the global map frame for adjustment
                            baseLinkToMap = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(5))
                            mapToBaseLink = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(5))
                            ptAbsolute = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=pt), baseLinkToMap)
                                                        
                            # Compensate for new zero marker position
                            ptAbsolute.pose.position.x += (currentZero.translation.x - zeroMarker.pose.position.x)
                            ptAbsolute.pose.position.y += (currentZero.translation.y - zeroMarker.pose.position.y)
                            ptAbsolute.pose.position.z += (currentZero.translation.z - zeroMarker.pose.position.z)
                            
                            pt = tf2_geometry_msgs.do_transform_pose(ptAbsolute, mapToBaseLink).pose
                            
                            rospy.loginfo("Moving EEF to adjusted position: (x: {:.2f}, y: {:.2f}, z: {:.2f})".format(
                                pt.position.x,
                                pt.position.y,
                                pt.position.z
                            ))
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

                    # Execute Gripper if needed
                    for gripper_topic in gripper_topics:
                        pos = gripper_msgs[gripper_topic][msg_count].requested_position
                        if abs(pos - self.gripper_pos[gripper_topic]) > self.GRIPPER_THRESHOLD:
                            # Actually set the gripper value
                            self.gripper_helper(gripper_topic, pos)
                
                self.result.time_elapsed = rospy.Duration.from_sec(time.time() - self.start_time)
                complete_msg = "Playback Keyframe Demo completed successfully"
                self.server.set_succeeded(self.result, complete_msg)
                rospy.loginfo(complete_msg)
                return
            
            self.result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
            self.server.set_preempted(self.result, "playback keyframe demo preempted")
            print 'preempted'
            return


if __name__ =='__main__':
    rospy.init_node('playback_keyframe_demo_action_server')
    PlaybackKFDemoAction()
    rospy.spin()
