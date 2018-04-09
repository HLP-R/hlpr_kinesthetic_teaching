#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
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
# * Neither the name of the SIM Lab nor the names of its
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

import os
import yaml
import importlib
import copy

import rospy
import actionlib
import rosbag
import rospkg

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt

#segments are a double-linked list
class KTSegment(object):
    def __init__(self, target, gripper_open, prev_seg = None, is_traj = False,
                 is_joints = True):
        self.is_traj = is_traj
        self.prev_seg = prev_seg
        self.gripper_open = gripper_open
        self.next_seg = None
        self.end = target
        self.plan = None
        self.points = None
        self.is_traj = False
        self.is_joints = is_joints
        self.planner = ArmMoveIt()
        
    def set_prev(self, prev_seg):
        self.prev_seg = prev_seg
        self.prev_seg.next_seg = self
        self.plan=None

    def set_eef_target(self, target):
        self.is_joints = False
        self.plan = None
        self.end = target

    def set_joint_target(self, target):
        self.is_joints = True
        self.plan = None
        self.end = target
        
    def get_end_joints(self):
        if self.is_joints:
            return self.end
        else:
            return self.get_plan().joint_trajectory.points[-1]

    def get_plan(self):
        if self.plan is not None:
            return self.plan
        else:
            return self.make_plan()

    def make_plan(self):
        if self.is_traj:
            rospy.logerr("Planning not yet implemented for trajectory segments")
            return None

        if self.prev is not None:
            start = self.prev_seg.get_end_joints()
        else:
            start = None
            
        self.plan = self.planner.plan_pose(target=self.end,
                                           is_joint_pos = self.is_joints,
                                           starting_config=start)
        return self.plan

    def change_next(self, segment):
        self.next_seg = segment
        segment.prev_seg = self

    #def __repr__(self):
    #    return


class KTPlayback(object):
    MIN_DELTA_T = rospy.Duration(1.0)
    JOINT_TOPIC = "/joint_states"
    EEF_TOPIC = "/eef_pose"
    GRIPPER_TOPIC = "/vector/right_gripper/stat"
    GRIPPER_OPEN_THRESH = 0.06
    JOINT_MOVE_THRESH = 0.01
    
    def __init__(self, is_joints=True):
        self.planner = ArmMoveIt()
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            DisplayTrajectory, queue_size=1)
        self.is_joints=is_joints #TODO: allow mixing EEF-only and joint keyframes

       
    def load_bagfile(self, bagfile):
        bag = rosbag.Bag(bagfile, "r")
        msgs = []
        for topic, msg, time in bag.read_msgs():
            msgs.append((topic, msg, time))

        frames = []

        time = msgs[0][2] #(topic, msg, time)
        while i<len(msgs):
            frame = []
            while msgs[i][2]==time and i<len(msgs):
                frame.append(msgs[i])
                i+=1
            frames.append(frame)
                    
        self.load_frames(frames)
            
    def load_pkl(self, pkl_file):
        with open(pkl_file, "rb") as filename:
            self.segments = cPickle.load(filename)
            
    def load_frames(self, frames):
        frames.sort(key=lambda f:f[2])
        
        #subsample trajectory frames
        self.segments = []
        
        time = None
        last_target = None
        last_segment = None
        for frame in frames:
            #subsample trajectory frames
            new_time = frame[2]
            if time is not None and new_time < time+rospy.Duration(MIN_DELTA_T):
                continue
            else:
                time = new_time
                
            step = {}
            for msg in frame:
                topic = msg[0]
                if topic in step:
                    rospy.logwarn("Multiple messages at time {} for topic {}".format(time, topic))
                else:
                    step[topic]=msg

            if self.is_joints:
                joint_msg = step[self.JOINT_TOPIC]
                arm_joints = self.planner.group[0].get_active_joints()
                target = dict([(joint,
                                joint_msg.position[joint_msg.name.index(joint)])
                               for joint in arm_joints])
            else:
                eef_msg = step[self.EEF_TOPIC]
                target = eef_msg

            if self.GRIPPER_TOPIC in step:
                grip_open = step[self.GRIPPER_TOPIC].position > self.GRIPPER_OPEN_THRESH

            saved = False
            if last_target is None:
                new_segment = KTSegment(target, grip_open,
                                        last_segment, is_traj = False,
                                        is_joints = self.is_joints)
                self.segments.append(new_segment)
                last_segment = new_segment
                saved = True
            else:
                if self.is_joints:
                    dist = 0
                    for joint in target: #changed to max over joints
                        d1 = abs(last_target[joint]-target[joint])
                        if  d1 > dist:
                            dist = d1
                if not self.is_joints or dist > self.JOINT_MOVE_THRESH:
                    new_segment = KTSegment(target, grip_open,
                                            last_segment, is_traj = False,
                                            is_joints = self.is_joints)
                    self.segments.append(new_segment)
                    last_segment = new_segment
                    saved = True
                else:
                    if last_segment is not None:
                        # didn't move, so update gripper; this is how the prev.
                        # version worked
                        last_segment.gripper_open = grip_open

            if not saved:
                rospy.loginfo("Discarded keyframe {} at time {}; not enough movement".format(len(self.segments)-1, time))
            

    def write_pkl(self,pklfile):
        cPickle.dump(self.segments, open(pklfile, "wb"), cPickle.HIGHEST_PROTOCOL)

    def vis_plan(self):
        if self.first is None:
            self.first = self.segments[0]

        next_seg = self.first
        display_trajectory=DisplayTrajectory()
        display_trajectory.trajectory_start = self.planner.robot.get_current_state()
        while next_seg is not None:
            display_trajectory.trajectory.append(next_seg.get_plan())
            next_seg = next_seg.next_seg
        self.display_trajectory_publisher.publish(display_trajectory)

    def playback_plan(self):
        if self.first is None:
            self.first = self.segments[0]

        next_seg = self.first
        plan = self.planner.plan_pose(next_seg.start, is_joint_pos = next_seg.is_eef)
        self.planner.move_robot(plan)
        while next_seg is not None:
            self.planner.move_robot(next_seg.plan)

#right now this doesn't handle moving the gripper for you!
class KTRecord(object):
    def __init__(self, bagfile_dir):
        if not os.path.isdir(os.path.expanduser(bagfile_dir)):
            rospy.logerr("Folder {} does not exist! Please create the folder and try again.".format(os.path.expanduser(bagfile_dir)))
            raise(ValueError)

        self.frames = []

        self.bag_dir = os.path.normpath(os.path.expanduser(bagfile_dir))
        
        default_yaml_loc = (rospkg.RosPack().get_path('hlpr_record_demonstration')
                            +'/data/topics.yaml')
        default_traj_rate = 100
        yaml_file_loc = rospy.get_param("~yaml_loc", default_yaml_loc)
        
        
        self.traj_record_rate = rospy.get_param("~traj_record_rate",
                                                default_traj_rate)

        rospy.loginfo("Setting up topic monitors")
        data_types = {}
        for data in yaml.load_all(file(yaml_file_loc, "r")):
            data_types[data['topic']]=data['msg_type']

        for topic in data_types:
            # Split the message type from the message package
            msg_type = data_types[topic]
            msg_arr = msg_type.split('/')
            msg_pkg = msg_arr[0] + '.msg'

            # Import the message type
            try:
                msg_mod = importlib.import_module(msg_pkg)
            except ImportError:
                rospy.logwarn("Messages for type {} could not be imported because {} wasn't found. Messages of this type will not be recorded".format(msg_type, msg_pkg))
                continue

            try:
                # Cycle through each topic and add each callback programatically
                listener = rospy.Subscriber(topic, eval('msg_mod.'+msg_arr[1]), self.monitor_cb, callback_args=topic, queue_size=10)

            except:
                rospy.logwarn("Messages of type: %s could not be loaded and will not be recorded" % msg_type)
        rospy.loginfo("Topic monitor setup complete")
        
        self.msg_store={}
        self.recording = False
        self.record_traj = False
        
        
    def monitor_cb(self, msg, topic):
        self.msg_store[topic] = msg

    def record_topics(self):
        time = rospy.Time.now()
        frame = []
        for topic in self.msg_store:
            frame.append((topic, copy.deepcopy(self.msg_store[topic]), time))
        self.frames.append(frame)
            
    def clear_frames(self):
        self.frames = []

    def remove_last_frame(self):
        self.frames.pop()

    def write_bagfile(self):
        if len(self.frames)==0:
            rospy.logwarn("No frames recorded! Writing empty bagfile...")
        bag = rosbag.Bag(self.bag_name, "w")
        for frame in self.frames:
            for msg_info in frame:
                topic = msg_info[0]
                time = msg_info[2]
                msg = msg_info[1]
                bag.write(topic, msg, t=time)
        bag.close()

    def write_traj(self):
        rate = rospy.Rate(self.traj_record_rate)#Hz
        while not rospy.is_shutdown and self.record_traj:
            self.record_topics()
            rate.sleep()
        
    def start(self, name, keyframe=True):
        self.clear_frames()
        self.recording = True
        if name[-4:]!=".bag":
            bagname = name+".bag"
        full_bag_path = self.bag_dir+"/"+ bagname
        if os.path.isfile(full_bag_path):
            rospy.logerr("Bagfile exists! Exit with Ctrl-C before 'end' is called to avoid overwriting your data")
        self.bag_name = full_bag_path 
        if keyframe:
            self.record_topics()
        else:
            self.start_traj_record()

    def stop_traj_record(self):
        self.record_traj = False
        self.record_thread.join()

    def start_traj_record(self):
        if not self.recording:
            rospy.logwarn("Cannot start trajectory logging until recording is started.  Start recording and try again.")
        self.record_thread = threading.Thread(target=self.write_traj)
        self.record_traj = True
        self.record_thread.start()
        
    def write_kf(self):
        if not self.recording:
            rospy.logwarn("Cannot write keyframe: recording not started.  Start recording and try again.")
        if self.record_traj:
            rospy.logwarn("Cannot write keyframe while recording trajectory. Stop recording trajectory and try again.")
            return
        self.record_topics()
        
    def end(self):
        if not self.recording:
            rospy.logwarn("Cannot end recording: recording was never started!")
            return
        if self.record_traj == True:
            self.stop_traj_record()
        self.write_bagfile()
        self.recording = False
        self.bag_name = ""
        return self.frames
