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
# Complete refactor 11/10/2016: Vivian Chu, vchu@diligentdroids.com

import actionlib
import cPickle
import os
import rosbag
import rospy
import yaml
import numpy as np

from collections import defaultdict
from hlpr_record_demonstration.playback_plan_object import PlaybackPlanObject
from hlpr_manipulation_utils.manipulator import Manipulator, Gripper
from hlpr_manipulation_utils.arm_moveit import ArmMoveIt
from hlpr_record_demonstration.msg import PlaybackKeyframeDemoAction, PlaybackKeyframeDemoGoal, PlaybackKeyframeDemoResult, PlaybackKeyframeDemoFeedback
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from vector_msgs.msg import GripperStat
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from std_msgs.msg import Bool
from data_logger_bag.msg import LogControl
from hlpr_record_demonstration.kf_tracker import KFTracker
from hlpr_record_demonstration.srv import ChangeKFRequest, ChangeKF

class PlaybackKFDemoAction(object):

    PLAN_OBJ_KEY = 'playback_objects'

    def __init__(self):

        # Setup the actionlib server
        self.server = actionlib.SimpleActionServer('playback_keyframe_demo', PlaybackKeyframeDemoAction, execute_cb=self.do_playback_keyframe_demo, auto_start=False)

        # Start the server
        self.server.start()

        # Load the drivers for the arm and gripper
        self.arm_planner = ArmMoveIt()
        self.manipulator = Manipulator()
        self.gripper = Gripper(prefix='right')

        # Load some thresholds
        self.KEYFRAME_THRESHOLD = rospy.get_param("~keyframe_threshold", 50)
        self.JOINT_THRESHOLD = rospy.get_param("~joint_threshold", 0.1) # total distance all joints have to move at minimum
        self.GRIPPER_MSG_TYPE = rospy.get_param("~gripper_msg_type", 'vector_msgs/GripperStat')
        self.GRIPPER_OPEN_THRESH = rospy.get_param("~gripper_open_thresh", 0.06)
        self.logger_topic = rospy.get_param("~logger_topic", 'data_logger_flag')
        self.logger_control_topic = rospy.get_param("~logger_control_msg_topic", 'C6_Task_Description')

        self._pre_plan = rospy.get_param('~pre_plan', False)
        self.gripper_status_topic = rospy.get_param('~gripper_topic', '/vector/right_gripper/stat')
        self.joint_state_topic = rospy.get_param('~joint_state_topic', '/joint_states')

        # Setup KF Tracker
        rospy.logwarn("Waiting for KF tracker service")
        rospy.wait_for_service(KFTracker.SERVICE_NAME)
        self.kf_tracker = rospy.ServiceProxy(KFTracker.SERVICE_NAME, ChangeKF)
        rospy.logwarn("KF tracker service loaded")

        # Subscribe to the current gripper status
        rospy.Subscriber(self.gripper_status_topic, GripperStat, self._gripper_update, queue_size=1)

        # Subscribe to the current joint states
        rospy.Subscriber(self.joint_state_topic, JointState, self._joint_state_update, queue_size=1)
       
        # Setup some gripper values during playback
        self.GRIPPER_THRESHOLD = 0.01 # 1cm threshold for moving
        self.GRIPPER_SLEEP_TIME = 2.0 # NOTE: not tuned

        # Get joints for the arm from the arm group that we want to plan with
        self._arm_joints = self.arm_planner.group[0].get_active_joints()

        # Display the trajectory publisher
        self.display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        DisplayTrajectory, queue_size=1)

        # Setup publisher for data logginer
        self.data_log_pub = rospy.Publisher(self.logger_topic, Bool, queue_size=5)
        self.log_control_pub = rospy.Publisher(self.logger_control_topic, LogControl, queue_size=5)

        # Store the playback file
        self.playback_file = None

        # Setup the internal result 
        self.result = PlaybackKeyframeDemoResult()

        # Store the current joint states
        self.current_joint_state = dict()

    def _gripper_update(self, msg):
        self.gripper_pos = msg.position

    def _joint_state_update(self, msg):
        self.current_joint_state = self._get_arm_joint_values(msg)

    # Main function that is called everytime playback is called
    def do_playback_keyframe_demo(self, goal):
        rospy.loginfo("Received playback demo goal")

        # Pull out the file that we'll be working with and check if valid
        filename = os.path.expanduser(goal.bag_file_name)
        rospy.loginfo("Receieved file: %s", filename)
        if os.path.isfile(filename):
            self.playback_file = filename
        else:
            error_msg = "Playback file given does not exist: %s" % filename
            self.server.set_aborted(self.result, error_msg)
            rospy.logerr(error_msg)
            return 

        # Setup data logging
        log_control_msg = LogControl()
        log_control_msg.runName = os.path.split(os.path.splitext(self.playback_file)[0])[-1]
        log_control_msg.playback = True
        self.log_control_pub.publish(log_control_msg)
    
        # Check what kind of file we've received (bag vs. pkl)
        if filename.endswith('.bag'):
            # Process the bag file
            (self.data_store, joint_flag) = self._process_bag(filename, goal.target_topic)
            if self.data_store is None:
                return
    
            # Save the created plan in a pkl file in the same directory
            if joint_flag:
                joint_str = 'joint'
            else:
                joint_str = 'EEF'

            pkl_name = '.'.join((os.path.splitext(self.playback_file)[0]+'_'+joint_str, 'pkl'))

            # Check if the file exists
            #if os.path.isfile(pkl_name):
            #    error_msg = "Error: existing pkl file exists: %s" % pkl_name
            #    self.server.set_aborted(self.result, error_msg)
            #    rospy.logerr(error_msg)
            #    return

            # Save the file
            rospy.loginfo("Saving pkl file: %s" % pkl_name)
            self._save_pkl(pkl_name, self.data_store)

        elif filename.endswith('.pkl'):
            self.data_store = self._load_pkl(filename)

        # Check if we're playing the file
        if goal.vis_only:
            plan = self._visualize_plan(self.data_store) 
            complete_msg = "Visualized plan successfully"
        else:
            plan = self._execute_plan(self.data_store)
            self.data_log_pub.publish(False)
            complete_msg = "Executed plan successfully"

        # Return success if we've gotten to this point
        if plan is None:
            error_msg = "Error: Playback unsuccessful" 
            self.server.set_aborted(self.result, error_msg)
            rospy.logerr(error_msg)
            return
            
        self.result.planned_trajectory = plan
        self.server.set_succeeded(self.result, complete_msg)
        rospy.loginfo(complete_msg)
        return

    def _process_bag(self, filename, target_topic):
        '''Heavy lifting process that actually converts bag files to plans'''

        # Open the bag file
        self.bag = rosbag.Bag(filename) 
       
        # Check if number of keyframes - sanity check if accidentally given trajectory 
        if self.bag.get_message_count() > self.KEYFRAME_THRESHOLD:
            error_msg = "Playback Keyframe Demo aborted due to too many frames: %d" % self.bag.get_message_count()
            self.server.set_aborted(self.result, error_msg)
            rospy.logerr(error_msg)
            return

        # Pull out information about the bagfile
        bag_info = yaml.load(self.bag._get_yaml_info())

        # Pull out the number of messages per topic
        total_keyframes = [x['messages'] for x in bag_info['topics']]
        total_keyframes = list(set(total_keyframes)) 

        # Check if keyframes are aligned
        if len(total_keyframes) > 1:
            error_msg = "Provided bag file does not have aligned keyframes. Exiting"
            self.server.set_aborted(self.result, error_msg)
            rospy.logerr(error_msg)
            return 
        else:
            total_keyframes = total_keyframes[0]

        # Pull out the gripper topic
        topics = self.bag.get_type_and_topic_info().topics.keys() 
        gripper_topic = [x['topic'] for x in bag_info['topics'] if x['type'] == self.GRIPPER_MSG_TYPE]
        if len(gripper_topic) == 0:
            rospy.loginfo("No gripper topics detected - will not include gripper")
        else:    
            if len(gripper_topic) > 1:
                rospy.logwarn('More than one gripper topic: %s' % ', '.join(gripper_topic))
                rospy.logwarn('Will only use values from topic: %s' % gripper_topic[0])
            gripper_topic = gripper_topic[0] # We assume there is one - warn otherwise

        # Store the messages
        data_store = defaultdict(dict)
        msg_store = dict()

        # Read the messages - cycle through and store in dictionary
        for topic, msg, t in self.bag.read_messages(): 
            if topic not in msg_store:
                msg_store[topic] = []

            msg_store[topic].append((msg,t))

        # Check what kind of target type (joints vs. EEF)
        target_type = self.bag.get_type_and_topic_info().topics[target_topic].msg_type
        if target_type == 'sensor_msgs/JointState':
            joint_flag = True
        else:
            joint_flag = False

        # We don't store off the original messages because they don't pickle well
        #data_store['msgs'] = msg_store 

        # Create keyframe objects
        data_store[self.PLAN_OBJ_KEY] = []
        for i in xrange(total_keyframes):
            # Pull out the expected topics
            plan_obj = PlaybackPlanObject()

            # Set the keyframe number
            plan_obj.set_keyframe(i)

            # Convert target into Pose or just joint dictionary
            data = msg_store[target_topic][i]
            if joint_flag:
                target = self._get_arm_joint_values(data[0])
            else:
                pos = data[0].position
                rot = data[0].orientation
                target = Pose(Point(pos.x, pos.y, pos.z), Quaternion(rot.x, rot.y, rot.z, rot.w))

            # Store away values
            plan_obj.set_target_val(target)
            plan_obj.set_target_time(data[1])
            plan_obj.set_joint_flag(joint_flag)

            # If gripper topic exists - set
            if len(gripper_topic) != 0:
                gripper_val = msg_store[gripper_topic][i][0].position
                plan_obj.set_gripper_val((gripper_val,msg_store[gripper_topic][i][1]))
           
            data_store[self.PLAN_OBJ_KEY].append(plan_obj) 
 
        # Filter the plan objects and remove objects with targets too close to the same position
        data_store[self.PLAN_OBJ_KEY] = self._check_keyframes(data_store[self.PLAN_OBJ_KEY], joint_flag)
        total_keyframes = len(data_store[self.PLAN_OBJ_KEY])

        # Actually get the plans from the moveit API
        plans = self.arm_planner.plan_targetInputWaypoint([x.target for x in data_store[self.PLAN_OBJ_KEY]], joint_flag, merged=False, current_joints = self.current_joint_state)

        # Check if we were able to create valid plan segments
        if plans is None:
            no_plan_msg = "Failed to generate a plan"
            self.server.set_aborted(self.result, no_plan_msg)
            return
        else:
            data_store['full_plan'] = plans
        
        # Plan with objects - even if not pre_plan, we test if the given keyframes will work
        for i in xrange(total_keyframes):
            plan_obj = data_store[self.PLAN_OBJ_KEY][i]
            plan_obj.plan = plans[i]
        
        return (data_store, joint_flag)

    def _check_keyframes(self, plan_objects, joint_flag):
        # Need to check if the target is similar to last AND
        # check the state of the gripper. If the gripper has changed
        # merge with the previous keyframe

        prev_obj = None
        discard_list = []
        for i in xrange(len(plan_objects)):
            plan_obj = plan_objects[i]
            if prev_obj is None:
                prev_obj = plan_obj
                continue

            total_difference = 0.0
            if joint_flag:
                for joint in plan_obj.target:
                    total_difference += abs(plan_obj.target[joint]-prev_obj.target[joint])

                # Check if the difference is too small - discard if so
                if total_difference < self.JOINT_THRESHOLD:
                    discard_list.append(i)

                    # Check if the gripper needs to be appended to the prior frame
                    if abs(plan_obj.gripper_val[0] - prev_obj.gripper_val[0]) > self.GRIPPER_THRESHOLD:
                        plan_objects[i-1].gripper_val = plan_obj.gripper_val

            else:
                pass # For now - do EEF checks

            prev_obj = plan_obj

        filtered_plan_objects = []
        for i in xrange(len(plan_objects)):
            if i not in discard_list:
                filtered_plan_objects.append(plan_objects[i])
            else:
                rospy.logwarn('Skipped keyframe: %d' %i)

        return filtered_plan_objects


    def _visualize_plan(self, data_store):

        # Pull out the plan from the data
        full_plan = [x.plan for x in data_store[self.PLAN_OBJ_KEY]]
        plan = self._merge_plans(full_plan)
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.arm_planner.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        return plan

    def _merge_plans(self, plans):
        ''' Merge an array of plans into a single plan'''
        if plans is None:
            return None

        if len(plans) < 1:
            return plans # Returns None or empty as well

        # Start merging
        merged_plan = plans[0]
        points = []
        for plan in plans:
            points = self._merge_points(points, plan.joint_trajectory.points)
        merged_plan.joint_trajectory.points = points

        return merged_plan

    def _merge_points(self, points, new_points):
        '''Merge two sets of points and taking into account time'''
        # Check if this is the first set
        if len(points) < 1:
            return new_points

        all_points = points
        # Pull out the last time from current points
        last_point_time = points[-1].time_from_start+rospy.Duration(5.0)
        # Remove the first point of the new_points trajectory
        new_points.pop(0)
        for point in new_points:
            point.time_from_start = point.time_from_start+last_point_time
            all_points = all_points + [point]
        return all_points

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

    def _execute_plan(self, data_store):

        while not self.server.is_preempt_requested():

            # Start recording
            self.data_log_pub.publish(True)

            # Pull out the plan segments
            plan_segments = data_store[self.PLAN_OBJ_KEY]

            # Execute each plan segment
            for plan_obj in plan_segments:
                result = self._send_plan(plan_obj.plan, plan_obj.keyframe_num)
                self._execute_gripper(plan_obj)

                # Check if the result was successful (error code = 0)
                if result.error_code != 0:
                    error_msg = "Trajectory playback unsuccessful for segment: %d" % plan_obj.keyframe_num
                    rospy.logerr(error_msg)
                    return

            # Finished
            return self._merge_plans(data_store['full_plan'])

        # If we were told to stop in the middle of execution
        preempt_msg = "Playback keyframe demo preempted"
        self.server.set_preempted(self.result, preempt_msg)
        rospy.logwarn(preempt_msg) 
        return

    def _send_plan(self, plan, keyframe_count=-1):
        # Check if we have a valid plan
        if plan == None or len(plan.joint_trajectory.points) < 1:
            rospy.logerror("No plan found")
        else:
            rospy.loginfo("Executing Keyframe: %d" % keyframe_count)
            self._kf_helper(num=keyframe_count)
            traj_goal = FollowJointTrajectoryGoal()
            traj_goal.trajectory = plan.joint_trajectory
            self.manipulator.arm.smooth_joint_trajectory_client.send_goal(traj_goal)
            self.manipulator.arm.smooth_joint_trajectory_client.wait_for_result()
            return self.manipulator.arm.smooth_joint_trajectory_client.get_result()

    def _execute_gripper(self, stored_obj):
        if stored_obj.gripper_val is not None:
            pos = stored_obj.gripper_val[0]
            if abs(pos - self.gripper_pos) > self.GRIPPER_THRESHOLD:

                # Change KF
                self._kf_helper(increment=True)

                # Check if gripper position is open or closed
                if pos < self.GRIPPER_OPEN_THRESH:
                    self.gripper.close()
                else:
                    self.gripper.open()

                # Comment this in if we want to go to the actual gripper taught position
                #self.gripper.set_pos(pos)
                rospy.sleep(self.GRIPPER_SLEEP_TIME) # Let gripper open/close

    def _kf_helper(self, num=0, increment=False, decrement=False):
        # Initialize and check what kind of change we want
        req = ChangeKFRequest()
        if increment:
            req.increment = True
        elif decrement:
            req.decrement = True
        else:
            req.kf_num = num

        # Send off
        resp = self.kf_tracker(req)
        if not resp.response:
            rospy.logwarn("Error occurred when setting current KF number")

    def _save_pkl(self, path_name, data):
        '''
        Simple helper that writes data to pkl file

        Input: data 
               path_name - location to and the filename of data
        '''

        cPickle.dump(data, open(path_name, "wb"), cPickle.HIGHEST_PROTOCOL)

    def _load_pkl(self, path_name):
        '''
        Simple helper function that loads a pickle file given a path to it

        input: path to pkl file
        output: loaded pickle file

        '''

        # Load pickle file
        with open(path_name, "rb") as file_path:
           loaded_file = cPickle.load(file_path)

        return loaded_file




if __name__ =='__main__':
  rospy.init_node('playback_keyframe_demo_action_server')
  PlaybackKFDemoAction()
  rospy.spin()




