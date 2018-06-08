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

import cPickle

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from kinova_msgs.srv import Start, Stop
from hlpr_manipulation_utils.manipulator import Gripper

#segments are a double-linked list
class KTSegment(object):
    JOINT_TOPIC = "joint_states"
    EEF_TOPIC = "eef_pose"
    if os.environ['ROBOT_NAME'] == "poli2":
        GRIPPER_TOPIC = "/gripper/stat"
    else:
        GRIPPER_TOPIC = "/vector/right_gripper/stat"
    GRIPPER_OPEN_THRESH = 0.06
    
    def __init__(self, planner, frames, delta_t,
                 prev_seg = None, next_seg = None,
                 is_traj = False, is_joints = True):
        self.is_traj = is_traj
        self.next_seg = next_seg
        self.plan = None
        self.prev_seg = prev_seg
        if self.prev_seg is not None:
            self.prev_seg.set_next(self)
        if self.next_seg is not None:
            self.next_seg.set_prev(self)
        self.dt = delta_t

        self.is_joints = is_joints
        self.planner = planner
        self.gripper = Gripper()
        self.gripper_open = self.gripper.get_pos() > 0.08
        self.frames = frames

        if self.is_traj == True:
            rospy.logwarn("Trajectory control not implemented; only loading last frame")
        #TODO: load trajectory into part of a plan; note that you'll still need to plan to the starting point of the trajectory, and could run into smoothness issues
        frame = self.frames[-1]

        step = {}
        for msg in frame:
            topic = msg[0]
            if topic in step:
                rospy.logwarn("Multiple messages at time {} for topic {}".format(time, topic))
            else:
                step[topic]=msg[1]

        if self.is_joints:
            if not self.JOINT_TOPIC in step:
                rospy.logwarn("Joint topic {} not found at time {}. Check your bagfile!".format(self.JOINT_TOPIC, time))
                return None

            joint_msg = step[self.JOINT_TOPIC]
            arm_joints = self.planner.group[0].get_active_joints()
            target = dict([(joint,
                            joint_msg.position[joint_msg.name.index(joint)])
                           for joint in arm_joints])
        else:
            if not self.EEF_TOPIC in step:
                rospy.logwarn("EEF topic {} not found at time {}. Check your bagfile!".format(self.EEF_TOPIC, time))
            eef_msg = step[self.EEF_TOPIC]
            target = eef_msg

        if self.GRIPPER_TOPIC in step:
            self.gripper_open = step[self.GRIPPER_TOPIC].position > self.GRIPPER_OPEN_THRESH

        self.end = target
        
    def set_prev(self, prev_seg):
        self.prev_seg = prev_seg
        if self.prev_seg is not None:
            self.prev_seg.next_seg = self
        self.plan=None

    def set_next(self, segment):
        self.next_seg = segment
        if self.next_seg is not None:
            self.next_seg.prev_seg = self
            self.next_seg.plan = None
        
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
            joint_traj = self.get_plan().joint_trajectory
            return dict(zip(joint_traj.joint_names,
                            joint_traj.points[-1].positions))

    def get_end_state(self):
        return self.planner.state_from_joints(self.get_end_joints())

    def get_start_joints(self):
        if self.prev_seg is not None:
            return self.prev_seg.get_end_joints()
        else:
            return None

    def get_start_state(self):
        if self.prev_seg is not None:
            return self.prev_seg.get_end_state()
        else:
            return self.planner.get_current_pose()

    def get_plan(self):
        #always replan if first segment, since robot could be starting from
        #anywhere
        if self.plan is not None and self.prev_seg is not None:
            return self.plan
        else:
            return self.make_plan()

    def get_reversed_plan(self):
        nalp = copy.deepcopy(self.get_plan())
        nalp.joint_trajectory.points = nalp.joint_trajectory.points[::-1]
        ref_time = nalp.joint_trajectory.points[0].time_from_start
        for p in nalp.joint_trajectory.points:
            p.time_from_start = ref_time-p.time_from_start
        return nalp

    def make_plan(self):
        if self.is_traj:
            rospy.logerr("Planning not yet implemented for trajectory segments")
            return None

        if self.prev_seg is not None:
            start = self.prev_seg.get_end_state()
        else:
            start = None
            
        self.plan = self.planner.plan_pose(target=self.end,
                                           is_joint_pos = self.is_joints,
                                           starting_config=start)
        return self.plan

    def __repr__(self):
        def abbr(joints):
            s = sorted(joints.items(), key=lambda j:j[0])
            s = map(lambda p: round(p[1],2),s)
            return s

        
        end = abbr(self.get_end_joints())
        
        if self.prev_seg is not None:
            start = abbr(self.get_start_joints())
        else:
            start = ["    " for e in end]

        
        text = "["+",".join(map(lambda p: "{:<4}->{:<4}".format(p[0],p[1]), zip(start,end)))+"]"

        r = "<KTSegment {}>".format(text)
        return r

#right now this doesn't handle moving the gripper for you!
class KTInterface(object):
    MIN_DELTA_T = rospy.Duration(0.1)
    JOINT_MOVE_THRESH = 0.01
    ARM_RELEASE_SERVICE = '/j2s7s300_driver/in/start_force_control'
    ARM_LOCK_SERVICE = '/j2s7s300_driver/in/stop_force_control'
    
    def __init__(self, save_dir, is_joints=True):
        if not os.path.isdir(os.path.expanduser(save_dir)):
            errstr = "Folder {} does not exist! Please create the folder and try again.".format(os.path.expanduser(save_dir))
            rospy.logerr(errstr)
            raise(ValueError(errstr))

        self.save_dir = os.path.normpath(os.path.expanduser(save_dir))


        if os.environ['ROBOT_NAME'] == "poli2":
            default_yaml_loc = (rospkg.RosPack().get_path('hlpr_kinesthetic_teaching_api')
                            +'/yaml/poli2_topics.yaml')
        else:
            default_yaml_loc = (rospkg.RosPack().get_path('hlpr_kinesthetic_teaching_api')
                            +'/yaml/poli1_topics.yaml')
        

        default_traj_rate = 3
        yaml_file_loc = rospy.get_param("~yaml_loc", default_yaml_loc)
        
        
        self.traj_record_rate = rospy.get_param("~traj_record_rate",
                                                default_traj_rate)

        self.msg_store={}
        self.recording = False
        self.record_traj = False
        self.last_time = None
        self.segment_pointer = None
        self.is_joints = is_joints
        self.first = None
        self.segments = []
        self.planner = ArmMoveIt()
        print self.planner.get_current_pose(simplify = False)
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            DisplayTrajectory, queue_size=1)
        self.gripper = Gripper()
        self.gripper_is_open = None

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

        rospy.loginfo("Waiting for arm services")
        rospy.wait_for_service(self.ARM_RELEASE_SERVICE)
        rospy.wait_for_service(self.ARM_LOCK_SERVICE)
        self.arm_release_srv = rospy.ServiceProxy(self.ARM_RELEASE_SERVICE, Start)
        self.arm_lock_srv = rospy.ServiceProxy(self.ARM_LOCK_SERVICE, Stop)
        rospy.loginfo("Ready to record keyframes.")

        
    def monitor_cb(self, msg, topic):
        self.msg_store[topic] = msg

    def clear_frames(self):
        self.last_time = rospy.Time.now()
        self.segment_pointer = None
        self.first = None
        self.segments = []

    def remove_current_frame(self):
        self.last_time = rospy.Time.now()
        if self.segment_pointer is None:
            rospy.logwarn("No segment pointer.  Have you recorded any keyframes?")
            return

        if self.segment_pointer == self.first:
            self.first = self.segment_pointer.next_seg
            if self.segment_pointer.next_seg is not None:
                self.segment_pointer.next_seg.set_prev(None)
            self.segments.remove(self.segment_pointer)
            self.segment_pointer = self.first
        else:
            self.segment_pointer.next_seg.set_prev(self.segment_pointer.prev_seg)
            self.segments.remove(self.segment_pointer)
            self.segment_pointer = self.segment_pointer.prev_seg

    def write_bagfile(self):
        if len(self.segments)==0:
            rospy.logwarn("No frames recorded! Doing nothing.")
            return
        
        bag = rosbag.Bag(self.save_name, "w")
        if self.first is None:
            self.first = self.segments[0]
        next_seg = self.first

        prev_time = rospy.Time.now()
        while next_seg is not None:
            for frame in next_seg.frames:
                max_time = rospy.Duration(0.0)
                for msg_info in frame:
                    topic = msg_info[0]
                    time = msg_info[2]+prev_time+next_seg.dt
                    if msg_info[2] + next_seg.dt > max_time:
                        max_time = msg_info[2] + next_seg.dt
                    msg = msg_info[1]
                    bag.write(topic, msg, t=time)
            prev_time = prev_time + max_time
            next_seg = next_seg.next_seg
        bag.close()

    def record_keyframe(self):
        dt = rospy.Time.now()-self.last_time
        frame = []
        prev_seg = self.segment_pointer
        if prev_seg is not None:
            next_seg = self.segment_pointer.next_seg
        else:
            next_seg = None
        for topic in self.msg_store:
            frame.append((topic, copy.deepcopy(self.msg_store[topic]), rospy.Duration(0.0)))
        new_seg = KTSegment(self.planner, [frame],
                            dt, prev_seg = prev_seg,
                            next_seg = next_seg,
                            is_traj = False, is_joints=self.is_joints)
        saved = self.insert_segment(new_seg, prev_seg, next_seg)
        if saved:
            self.segment_pointer = new_seg
            if prev_seg is None and self.first is None:
                rospy.logwarn("Setting this keyframe to first frame. This can result in odd behavior if the segment set is not empty.")
                self.first = new_seg

    def record_trajectory(self, as_keyframes = True):
        prev_seg = self.segment_pointer
        next_seg = self.segment_pointer.next_seg
        if prev_seg is not None:
            next_seg = self.segment_pointer.next_seg
        else:
            next_seg = None
        rate = rospy.Rate(self.traj_record_rate)#Hz
        if as_keyframes:
            while not rospy.is_shutdown and self.record_traj:
                self.record_keyframe()
                rate.sleep()
        else:
            dt = rospy.Time.now()-self.last_time
            self.traj_frames = []
            frames = []
            start = rospy.Time.now()
            while not rospy.is_shutdown and self.record_traj:
                frame = []
                for topic in self.msg_store:
                    frame.append((topic, copy.deepcopy(self.msg_store[topic]),
                                  rospy.Time.now()-start))
                frames.append(frame)
                rate.sleep()
            new_seg = KTSegment(self.planner, frames,
                                dt, prev_seg = prev_seg,
                                next_seg = next_seg,
                                is_traj = False, is_joints=self.is_joints)
            self.segments.append(new_seg)
            self.segment_pointer = new_seg
            self.last_time = rospy.Time.now()
            if prev_seg is None and self.first is None:
                rospy.logwarn("Setting this keyframe to first frame. This can result in odd behavior if the segment set is not empty.")
                self.first = new_seg

    def set_pointer(self, segment_idx):
        self.segment_pointer = self.segments[segment_idx]

        
    def start(self, name, start_with_traj=False, is_joints = True):
		self.clear_frames()
		self.is_joints = is_joints
		self.recording = True
		self.last_time = rospy.Time.now()
		self.segment_pointer = None
		if name[-4:]==".pkl":
			filename = name+".bag"
		elif name[-4:]==".bag":
			filename = name
		else:
			filename = name+".bag"
		full_save_path = self.save_dir+"/"+ filename
		if os.path.isfile(full_save_path):
			rospy.logerr("Saving file exists! Exit with Ctrl-C before 'end' is called to avoid overwriting your data")
		self.save_name = full_save_path
		self.release_arm()
		if not start_with_traj:
			self.record_keyframe()
		else:
			self.start_traj_record()

    def stop_traj_record(self):
        self.record_traj = False
        self.record_thread.join()

    def start_traj_record(self):
        if not self.recording:
            rospy.logwarn("Cannot start trajectory logging until recording is started.  Start recording and try again.")
        self.record_thread = threading.Thread(target=self.record_trajectory)
        self.record_traj = True
        self.record_thread.start()
        
    def write_kf(self):
        if not self.recording:
            rospy.logwarn("Cannot write keyframe: recording not started.  Start recording and try again.")
        if self.record_traj:
            rospy.logwarn("Cannot write keyframe while recording trajectory. Stop recording trajectory and try again.")
            return
        self.record_keyframe()
        
    def end(self):
		if not self.recording:
			rospy.logwarn("Cannot end recording: recording was never started!")
			return
		if self.record_traj == True:
			self.stop_traj_record()
		self.write_bagfile()
		self.recording = False
		self.save_name = ""
		return self.segments
    
    def load_bagfile(self, bagfile):
        bag = rosbag.Bag(os.path.expanduser(bagfile), "r")
        msgs = []
        for topic, msg, time in bag.read_messages():
            msgs.append((topic, msg, time))

        frames = []

        time = msgs[0][2] #(topic, msg, time)
        i=0
        while i<len(msgs):
            frame = []
            while i<len(msgs) and msgs[i][2]==time:
                frame.append(msgs[i])
                i+=1
            if i<len(msgs):
                time = msgs[i][2]
            frames.append(frame)
        self.load_frames(frames)

    #moves directly to the keyframe
    def move_to_keyframe(self, segment):
        self.lock_arm()
        first_seg = KTSegment(self.planner, segment.frames,
                              segment.dt,
                              is_traj = False,
                              is_joints = self.is_joints)
        plan = first_seg.get_plan()
        success = self.planner.move_robot(plan)
        if success:
            self.set_gripper(self.segment_pointer.gripper_open)
        else:
            rospy.logerr("Error moving to segment {}.".format(segment))
        return success

    def stop_robot(self):
        pass
            
    def move_forward(self):
		self.lock_arm()
		print("SEGMENT POINTER:",self.segment_pointer)

		if self.segment_pointer is None or self.segment_pointer.next_seg is None:
			rospy.logwarn("No keyframe to move to! Doing nothing.")
			return

		if not self.at_seg_pointer():
			rospy.logwarn("Not at the keyframe we're pointing to. Making plan to get there.")
			success = self.move_to_keyframe(self.segment_pointer)
			if not success:
				rospy.logerr("Error moving to keyframe. Aborting.")
				return

		segment = self.segment_pointer.next_seg
		success = self.planner.move_robot(segment.get_plan())
		if not success:
				rospy.logerr("Error moving to keyframe. Aborting.")
				return
		else:
			self.set_gripper(self.segment_pointer.gripper_open)
			self.segment_pointer = segment

    def move_backward(self):
		self.lock_arm()
		print("SEGMENT POINTER:",self.segment_pointer)
		if self.segment_pointer is None or self.segment_pointer.prev_seg is None:
			rospy.logwarn("No keyframe to move to! Doing nothing.")
			return

		if not self.at_seg_pointer():
			rospy.logwarn("Not at the keyframe we're pointing to. Making plan to get there.")
			success = self.move_to_keyframe(self.segment_pointer)
			if not success:
				rospy.logerr("Error moving to keyframe. Aborting.")
				return
		
		success = self.planner.move_robot(self.segment_pointer.get_reversed_plan())
		if not success:
			rospy.logerr("Error moving to keyframe. Aborting.")
			return
		else:
			rospy.loginfo("Moved to segment.")
			self.set_gripper(self.segment_pointer.gripper_open)
			self.segment_pointer = self.segment_pointer.prev_seg
			print("Moved to segment:",self.segment_pointer.frames)

    def at_seg_pointer(self):
        return self.at_keyframe_target(self.segment_pointer)

    def at_keyframe_target(self, segment):
        end_pose = segment.get_end_joints()
        current_pose = self.planner.get_current_pose(simplify=False)
        matches = [abs(end_pose[j]-current_pose[j])<self.JOINT_MOVE_THRESH for j in end_pose]        
        return all(matches)

    def print_current_pose(self):
        current_pose = self.planner.get_current_pose(simplify=False)
        def abbr(joints):
            s = sorted(joints.items(), key=lambda j:j[0])
            s = map(lambda p: str(round(p[1],2)),s)
            return s

        print "Current pose:", ",".join(abbr(current_pose))
        
    def move_to_start(self):
        self.lock_arm()
        
        if len(self.segments) == 0:
            rospy.logwarn("No segments. Have you recorded any keyframes?")
            return
            
        if self.first is None:
            self.first = self.segments[0]

        if self.segment_pointer is None:
            rospy.logwarn("No segment pointer.  Have you recorded any keyframes?")
            return
        
        if self.segment_pointer == self.first:
            rospy.logwarn("Already at the start. Doing nothing.")
            return

        if not self.at_keyframe_target(self.segment_pointer):
            rospy.logwarn("Not at a keyframe. Moving to the last keyframe")
            
            
            success = self.move_to_keyframe(self.segment_pointer)
            if not success:
                rospy.logerr("Unable to move to last keyframe. Aborting!")
                return

        keyframe = self.segment_pointer
        while keyframe != self.first:
            success = self.planner.move_robot(keyframe.get_reversed_plan())
            
            if success:
                keyframe = keyframe.prev_seg
                self.segment_pointer = keyframe
                self.set_gripper(self.segment_pointer.gripper_open)
            else:
                rospy.logerr("Unable to move to keyframe {}. Aborting!".format(self.segment_pointer))
                return

    def move_to_end(self):
        self.lock_arm()
        
        if len(self.segments) == 0 :
            rospy.logwarn("No segments. Have you recorded any keyframes?")
            return
            
        if self.first is None:
            self.first = self.segments[0]

        if self.segment_pointer is None:
            rospy.logwarn("No segment pointer.  Have you recorded any keyframes?")
            return
        
        if self.segment_pointer.next_seg is None:
            rospy.logwarn("Already at the end. Doing nothing.")
            return

        if not self.at_keyframe_target(self.segment_pointer):
            rospy.logwarn("Not at a keyframe. Moving to the last keyframe")
            self.set_gripper(self.segment_pointer.gripper_open)
            success = self.move_to_keyframe(self.segment_pointer)
            if not success:
                rospy.logerr("Unable to move to last keyframe. Aborting!")
                return

        keyframe = self.segment_pointer
        while keyframe.next_seg is not None:
            success  = self.planner.move_robot(keyframe.get_plan())

            if success:
                keyframe = keyframe.next_seg
                self.segment_pointer = keyframe
                self.set_gripper(keyframe.gripper_open)
            else:
                rospy.logerr("Unable to move to last keyframe. Aborting!")
                return

        success = self.planner.move_robot(keyframe.get_plan())
        if success:
            self.segment_pointer = keyframe
            self.set_gripper(keyframe.gripper_open)
        else:
            rospy.logerr("Unable to move to last keyframe. Aborting!")
            return

    def release_arm(self):
        self.arm_release_srv()

    def lock_arm(self):
        self.arm_lock_srv()

    def open_gripper(self):
        self.gripper.open()
        self.gripper_is_open = True

    def close_gripper(self):
        self.gripper.close()
        self.gripper_is_open = False

    def set_gripper(self, gripper_open, wait = True):
        if gripper_open and (self.gripper_is_open is None or not self.gripper_is_open):
            self.open_gripper()
        if not gripper_open and (self.gripper_is_open is None or self.gripper_is_open):
            self.close_gripper()

        if wait:
            while self.gripper.is_moving() and not rospy.is_shutdown():
                rospy.sleep(0.05)

    def insert_segment(self, new_segment, prev_seg=None, next_seg=None):
        saved = False

        if prev_seg is None:
            self.segments.append(new_segment)
            saved = True
        else:
            target = new_segment.get_end_joints()
            last_target = prev_seg.get_end_joints()
            dist = 0
            for joint in target: #changed to max over joints
                d1 = abs(last_target[joint]-target[joint])
                if  d1 > dist:
                    dist = d1
            if dist > self.JOINT_MOVE_THRESH:
                self.segments.append(new_segment)
                new_segment.set_prev(prev_seg)
                saved = True
            else:
                # didn't move, so update gripper; this is how the prev.
                # version worked
                rospy.logwarn("Robot didn't move but gripper might have changed. Updating previous segment. This matches prior version behavior but might not be the best long-term solution.")
                prev_seg.gripper_open = new_segment.gripper_open

        if saved and next_seg is not None:
            new_segment.set_next(next_seg)

        return saved

    def load_frames(self, frames):
		if len(frames)==0:
			rospy.logwarn("No frames to load! Clearing segments...")

		frames.sort(key=lambda f:f[0][2])

		#subsample trajectory frames
		self.segments = []

		time = frames[0][0][2]
		last_segment = None
		for frame in frames:
			#subsample trajectory frames for now; could store multiple frames
			#in one segment for a trajectory segment
			new_time = frame[0][2]
			if last_segment is not None and new_time < time+self.MIN_DELTA_T:
				dt = (new_time-time).to_sec()
				rospy.logwarn("Skipping frame at time {}. Only {}s since last frame".format(new_time,dt))
				continue
			else:
				time = new_time

			delta_t = new_time-time
			for i in range(len(frame)):
				old_item = frame[i]
				#the following only works for keyframes, not trajectories
				new_item = (frame[i][0],frame[i][1],rospy.Duration(0.0))
				    
			new_segment = KTSegment(self.planner, [frame], delta_t)
			if new_segment is None:
				continue
		
			saved = self.insert_segment(new_segment, prev_seg=last_segment)

			if saved:
				last_segment = new_segment
			else:
				rospy.loginfo("Discarded keyframe {} at time {}; not enough movement".format(len(self.segments)-1, time))
		self.segment_pointer = self.segments[0]
		
	
            

    def write_pkl(self):
        cPickle.dump((self.first, self.segments), open(self.save_name, "wb"), cPickle.HIGHEST_PROTOCOL)      
            
    def load_pkl(self, pkl_file):
        with open(pkl_file, "rb") as filename:
            self.first,self.segments = cPickle.load(filename)

    def vis_plan_reverse(self):
        if len(self.segments)==0:
            rospy.logwarn("No keyframes loaded. Not doing anything.")
            return
        
        if self.first is None:
            self.first = self.segments[0]
            rospy.logwarn("First was not set; this could result in odd behavior.")
        
        #run through the list to find the last segment
        this_seg = self.first
        next_seg = self.first.next_seg
        while next_seg is not None:
            this_seg = next_seg
            next_seg = this_seg.next_seg
        last_seg = this_seg

        #add a segment to get us to the start

        first_seg = KTSegment(self.planner, last_seg.frames, last_seg.dt,
                              is_traj = False,
                              is_joints = self.is_joints)
        plan = first_seg.get_plan()
        
        display_trajectory=DisplayTrajectory()
        display_trajectory.trajectory_start = self.planner.robot.get_current_state()
        prev_time = rospy.Duration(0.0)
        
        display1 = DisplayTrajectory()
        display1.trajectory_start = first_seg.get_start_state()
        display1.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display1)
        rospy.sleep(plan.joint_trajectory.points[-1].time_from_start+rospy.Duration(0.5))

        if plan is None:
            rospy.logerr("Could not find plan for segment {}".format(first_seg))
            return
            
        next_plan = copy.deepcopy(plan)
        for p in plan.joint_trajectory.points:
            p.time_from_start+=prev_time
            
        display_trajectory.trajectory.append(next_plan)
        prev_seg = last_seg
        prev_time = plan.joint_trajectory.points[-1].time_from_start
        
        while prev_seg is not self.first:
            rospy.loginfo("Displaying trajectory {}".format(prev_seg))
            plan = prev_seg.get_reversed_plan()
            #print plan.joint_trajectory.points[0]
            #print plan.joint_trajectory.points[-1]

            display1 = DisplayTrajectory()
            display1.trajectory_start = prev_seg.get_end_state()
            display1.trajectory.append(plan)
            self.display_trajectory_publisher.publish(display1)
            rospy.sleep(plan.joint_trajectory.points[-1].time_from_start+rospy.Duration(0.5))

            if plan is None:
                rospy.logerr("Could not find plan for segment {}".format(next_seg))
                return
            
            next_plan = copy.deepcopy(plan)
            for p in plan.joint_trajectory.points:
                p.time_from_start+=prev_time
                
            display_trajectory.trajectory.append(next_plan)
            prev_seg = prev_seg.prev_seg
            prev_time = plan.joint_trajectory.points[-1].time_from_start

        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5.0)
            
    def vis_plan(self):
        if len(self.segments)==0:
            rospy.logwarn("No keyframes loaded. Not doing anything.")
            return
        
        if self.first is None:
            self.first = self.segments[0]

        next_seg = self.first
        display_trajectory=DisplayTrajectory()
        display_trajectory.trajectory_start = self.planner.robot.get_current_state()
        prev_time = rospy.Duration(0.0)
        while next_seg is not None:
            print next_seg
            plan = next_seg.get_plan()
            #print plan.joint_trajectory.points[0]
            #print plan.joint_trajectory.points[-1]

            display1 = DisplayTrajectory()
            display1.trajectory_start = next_seg.get_start_state()
            display1.trajectory.append(plan)
            self.display_trajectory_publisher.publish(display1)
            rospy.sleep(plan.joint_trajectory.points[-1].time_from_start+rospy.Duration(0.5))

            if plan is None:
                rospy.logerr("Could not find plan for segment {}".format(next_seg))
                return
            
            next_plan = copy.deepcopy(plan)
            for p in plan.joint_trajectory.points:
                p.time_from_start+=prev_time
                
            display_trajectory.trajectory.append(next_plan)
            next_seg = next_seg.next_seg
            prev_time = plan.joint_trajectory.points[-1].time_from_start

        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5.0)

    def playback_plan(self):
        self.move_to_start()
        self.move_to_end()

    def playback_plan_reverse(self):
        self.move_to_end()
        self.move_to_start()
