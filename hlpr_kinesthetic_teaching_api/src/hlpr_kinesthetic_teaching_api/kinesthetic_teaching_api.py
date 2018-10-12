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

import copy
import os
import yaml
import threading

import actionlib
import rosbag
import roslib
import rospkg
import rospy
import tf
import tf2_ros

import cPickle

from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from kinova_msgs.srv import Start, Stop

from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from robotiq_85_msgs.msg import GripperStat

# used by both KTSegment and KTInterface
EEF_PREFIX = 'eef_pose_'

#segments are a double-linked list
class KTSegment(object):
    if os.environ['ROBOT_NAME'] == "2d_arm":
        JOINT_TOPIC = "/sim_arm/joint_state"
    else:
        JOINT_TOPIC = "joint_states"


    if os.environ['ROBOT_NAME'] == "2d_arm":
        EEF_TOPIC = "/sim_arm/eef_pose"
        ARM_FRAME = None
    else:
        EEF_FRAME = "j2s7s300_ee_link"
        ARM_FRAME = "j2s7s300_link_base"

    if os.environ['ROBOT_NAME'] == "poli2":
        GRIPPER_TOPIC = "gripper/stat"
    elif os.environ['ROBOT_NAME'] == "2d_arm":
        GRIPPER_TOPIC = "/sim_arm/gripper_state"
    else:
        GRIPPER_TOPIC = "/vector/right_gripper/stat"

    GRIPPER_OPEN_THRESH = 0.06

    def __init__(self, planner, gripper_interface, frames, delta_t,
                 prev_seg = None, next_seg = None,
                 is_traj = False, is_joints = True, gripper_open=None, rel_frame=None):
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
        self.gripper = gripper_interface
        self.frames = frames
        self.rel_frame = self.ARM_FRAME

        if self.is_traj == True:
            rospy.logwarn("Trajectory control not implemented; only loading last frame")
        #TODO: load trajectory into part of a plan; note that you'll still need to plan to the starting point of the trajectory, and could run into smoothness issues
        frame = self.frames[-1]

        step = {}
        for msg in frame:
            topic = msg[0]
            if topic in step:
                rospy.logwarn("Multiple messages at dt {} for topic {}".format(self.dt, topic))
            else:
                step[topic]=msg[1]

        if self.is_joints:
            if not self.JOINT_TOPIC in step:
                rospy.logwarn("Joint topic {} not found at dt {}. Check your bagfile!".format(self.JOINT_TOPIC, self.dt))

            else:
                joint_msg = step[self.JOINT_TOPIC]
                if self.planner is None:
                    raise RuntimeError("cannot interpret joint keyframe segment because no planner was set.")
                arm_joints = self.planner.group[0].get_active_joints()
                target = dict([(joint,
                                joint_msg.position[joint_msg.name.index(joint)])
                               for joint in arm_joints])
                self.end = target
        else:
            if rel_frame is None:
                target = step[EEF_PREFIX+"relative"]
                self.rel_frame = target.header.frame_id
                self.end = target
            else:
                eef_key = EEF_PREFIX+rel_frame
                if eef_key in step:
                    self.rel_frame = rel_frame
                    target = step[eef_key]
                    self.end = target
                elif EEF_PREFIX+self.ARM_FRAME in step:
                    target = step[EEF_PREFIX+self.ARM_FRAME]
                    self.rel_frame = self.ARM_FRAME
                    self.end = target
                else:
                    rospy.logwarn("EEF topic {} not found at dt {}. Check your bagfile!".format(eef_key, self.dt))
                    rospy.logwarn("Available topics are: " + str(step.keys()))
                    rospy.logwarn("Not setting the end pose or rel_frame. If they are not set manually, errors will occur.")


                
        match = None
        for s in step.keys():
            if self.GRIPPER_TOPIC in s:
                match = s
                break
        if match is not None:
            self.gripper_open = step[match].requested_position > self.GRIPPER_OPEN_THRESH
        else:
            if gripper_open is None:
                if self.gripper is None:
                    raise RuntimeError("you must supply a gripper or gripper_open value")
                gripper_open = self.gripper.get_pos() > self.GRIPPER_OPEN_THRESH
            self.gripper_open = gripper_open

        self.data = step

    def change_frame(self, rel_frame):
        if self.is_joints:
            rospy.logwarn("Can't set frame for joint demo")
            return
        if rel_frame is None:
            rel_frame = self.ARM_FRAME
            self.rel_frame = self.ARM_FRAME
        try:
            eef_msg = self.data[EEF_PREFIX+rel_frame]
        except:
            rospy.logwarn("Error setting frame as {}. Does the frame exist?".format(rel_frame))
            return
        target = eef_msg
        self.end = target
        self.rel_frame = rel_frame

    def freeze_rel_to_arm_frame(self): #undo this by calling change_frame("relative")
        if self.is_joints:
            rospy.logwarn("Can't set frame for joint demo")
            return

        listener = tf.TransformListener()
        try:
            listener.waitForTransform(self.ARM_FRAME, self.end.header.frame_id,
                                      rospy.Time(0), rospy.Duration(5))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            rospy.logerr("Couldn't change from frame {} to arm frame. Keeping original frame. (Error: {})".format(self.end.header.frame_id, e))
            return

        try:
            new_pose = listener.transformPose(self.ARM_FRAME, self.end)
        except rospy.ServiceException as e:
            rospy.logerror("Couldn't change from frame {} to arm frame. Keeping original frame. (Error: {})".format(self.end.header.frame_id, e))
            return
        self.end = new_pose

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
            plan = self.get_plan()
            if plan is None:
                return None
            joint_traj = plan.joint_trajectory
            return dict(zip(joint_traj.joint_names,
                            joint_traj.points[-1].positions))

    def get_end_state(self):
        end_joints = self.get_end_joints()
        if end_joints is None:
            return None
        else:
            return self.planner.state_from_joints(end_joints)

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
        #anywhere or if we're planning in eef space (where the frame could move)
        if self.plan is not None and self.prev_seg is not None and self.is_joints:
            return self.plan
        else:
            return self.make_plan()

    def get_reversed_plan(self):
        plan = self.get_plan()
        if plan is None:
            return None
        nalp = copy.deepcopy(plan)
        nalp.joint_trajectory.points = nalp.joint_trajectory.points[::-1]
        ref_time = nalp.joint_trajectory.points[0].time_from_start
        for p in nalp.joint_trajectory.points:
            p.time_from_start = ref_time-p.time_from_start
        return nalp

    def make_plan(self):
        if self.is_traj:
            rospy.logerr("Planning not yet implemented for trajectory segments")
            return None

        if self.prev_seg is not None and self.is_joints:
            start = self.prev_seg.get_end_state()
        else:
            start = None

        if self.is_joints:
            what_kind = "joint"
        else:
            what_kind = "eef"
            
        rospy.loginfo("Making {} plan.".format(what_kind))
        plan = self.planner.plan_pose(target=self.end,
                                      is_joint_pos = self.is_joints,
                                      starting_config=start)


        
        if plan is None or len(plan.joint_trajectory.points)==0:
            rospy.logwarn("No plan found. Trying again...")
            plan = self.planner.plan_pose(target=self.end,
                                      is_joint_pos = self.is_joints,
                                      starting_config=start)
            if plan is None or len(plan.joint_trajectory.points)==0:
                rospy.logwarn("Still no plan found. Giving up.")
                return None
        self.plan = plan

        return self.plan

    def as_dict(self):
        def abbr(joints):
            s = sorted(joints.items(), key=lambda j:j[0])
            s = map(lambda p: round(p[1],2),s)
            return s

        def abbr_pose(pose):
            s =  [pose.pose.position.x,
                  pose.pose.position.y,
                  pose.pose.position.z,
                  pose.pose.orientation.x,
                  pose.pose.orientation.y,
                  pose.pose.orientation.z,
                  pose.pose.orientation.w]

            s = map(lambda p: round(p,2),s)
            return s

        if self.is_joints:
            end = abbr(self.end)
            header = "joints"
        else:
            end = abbr_pose(self.end)
            if self.rel_frame is None:
                header = self.end.header.frame_id
            else:
                header = self.rel_frame

        return {"type":header, "pose":end}
    
    def __repr__(self):
        arr = self.as_dict()
            
        text = "["+", ".join(map(lambda p: "{:<4}".format(p), arr["pose"]))+"]"

        if self.gripper_open:
            gripper_st = "open"
        else:
            gripper_st = "closed"
            
        r = "<KTSegment {} {} {}>".format(gripper_st, arr["type"], text)
        return r

class HashableKTSegment(KTSegment):

    def __init__(self, **kwargs):
        super(HashableKTSegment, self).__init__(**kwargs)

    def __eq__(self, other):
        return hash(self)==hash(other)
        '''return self.end.pose.position == other.end.pose.position and \
               self.end.pose.orientation == other.end.pose.orientation and \
               self.dt == other.dt and \
               self.is_joints == other.is_joints and \
               self.gripper_open == other.gripper_open and \
               self.is_traj == other.is_traj and \
               self.frames == other.frames'''

    def __hash__(self):
        # print("hashable")
        if self.end.pose.orientation.x < 0:
            return hash((round(self.end.pose.position.x,6), round(self.end.pose.position.y,6),
                         round(self.end.pose.position.z,6), -round(self.end.pose.orientation.x,6),
                         -round(self.end.pose.orientation.y,6), -round(self.end.pose.orientation.z,6),
                         -round(self.end.pose.orientation.w,6), self.is_joints, self.gripper_open))
        else:
            return hash((round(self.end.pose.position.x,6), round(self.end.pose.position.y,6),
                         round(self.end.pose.position.z,6), round(self.end.pose.orientation.x,6),
                         round(self.end.pose.orientation.y,6), round(self.end.pose.orientation.z,6),
                         round(self.end.pose.orientation.w,6), self.is_joints, self.gripper_open))
        

    @classmethod
    def FromKTSegment(self, seg):
        seg.__class__ = HashableKTSegment
        return seg

class KTInterface(object):
    MIN_DELTA_T = rospy.Duration(0.1)
    JOINT_MOVE_THRESH = 0.01  # 0.01radians ~= 0.57degrees
    XYZ_MOVE_THRESH = 0.005  # 5mm
    QUAT_MOVE_THRESH = 0.01 # ???
    ARM_RELEASE_SERVICE = '/j2s7s300_driver/in/start_force_control'
    ARM_LOCK_SERVICE = '/j2s7s300_driver/in/stop_force_control'
    ARM_FRAME = 'j2s7s300_link_base'
    EEF_FRAME = 'j2s7s300_ee_link'

    def __init__(self, save_dir, planner, gripper_interface, physical_arm=None, default_yaml_loc= None):
        rospy.loginfo("Initializing KT interface")
        rospy.loginfo("Getting save directory")
        if not os.path.isdir(os.path.expanduser(save_dir)):
            errstr = "Folder {} does not exist! Please create the folder and try again.".format(os.path.expanduser(save_dir))
            rospy.logerr(errstr)
            raise(ValueError(errstr))

        self.save_dir = os.path.normpath(os.path.expanduser(save_dir))


        if default_yaml_loc is None:
            pkg_dir = rospkg.RosPack().get_path('hlpr_kinesthetic_teaching_api')
            if os.environ['ROBOT_NAME'] == "poli2":
                rospy.loginfo("Getting yaml location for poli2...")
                default_yaml_loc = (pkg_dir+'/yaml/poli2_topics.yaml')
                rospy.loginfo("... location found!")
            else:
                rospy.loginfo("Getting yaml location for poli2...")
                default_yaml_loc = (pkg_dir+'/yaml/poli1_topics.yaml')
                rospy.loginfo("... location found!")

        rospy.loginfo("Getting parameters...")
        default_traj_rate = 3
        yaml_file_loc = rospy.get_param("~yaml_loc", default_yaml_loc)

        using_real_arm = rospy.get_param("~physical_arm", True)

        object_frames = rospy.get_param("~object_frames", "")
        object_frames = object_frames.split(",")
        object_frames = filter(lambda s: len(s)>0, object_frames)

        if os.environ["ROBOT_NAME"]=="2d_arm":
            using_real_arm = False

        if physical_arm is not None:
            using_real_arm = physical_arm

        self.traj_record_rate = rospy.get_param("~traj_record_rate",
                                                default_traj_rate)

        self.msg_store={}
        self.msg_type_lookup={}
        self.recording = False
        self.monitor_tfs = True
        self.record_traj = False
        self.last_time = None
        self.segment_pointer = None
        self.is_joints = None
        self.first = None
        self.segments = []
        self.planner = planner

        rospy.loginfo("Setting up display publisher")
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            DisplayTrajectory, queue_size=1)
        self.gripper = gripper_interface
        self.gripper_is_open = None

        rospy.loginfo("Setting up topic monitors")

        watched_topics = {entry['topic']: entry['msg_type'] for entry in yaml.load_all(file(yaml_file_loc, "r"))}

        for topic, topic_type in watched_topics.items():
            msg_class = roslib.message.get_message_class(topic_type)
            if msg_class is None:
                rospy.logwarn("Messages of type: {} could not be loaded and will not be recorded".format(topic_type))
            else:
                self.msg_type_lookup[topic] = msg_class
                rospy.Subscriber(topic, msg_class, self.monitor_cb, callback_args=topic, queue_size=10)

        rospy.loginfo("Topic monitor setup complete")

        if using_real_arm:
            rospy.loginfo("Waiting for arm services")
            rospy.wait_for_service(self.ARM_RELEASE_SERVICE)
            rospy.wait_for_service(self.ARM_LOCK_SERVICE)
            self.arm_release_srv = rospy.ServiceProxy(self.ARM_RELEASE_SERVICE, Start)
            self.arm_lock_srv = rospy.ServiceProxy(self.ARM_LOCK_SERVICE, Stop)

            self.monitor_frames = {}
            object_frames.append(self.ARM_FRAME)
            for oframe in object_frames:
                self.monitor_frames["{}{}".format(EEF_PREFIX, oframe)] = (oframe, self.EEF_FRAME)
            self.tf_thread = threading.Thread(target=self.monitor_tf_cb)
            self.tf_thread.start()


        else:
            self.arm_release_srv = lambda: None
            self.arm_lock_srv = lambda: None

        rospy.loginfo("Ready to record keyframes.")


    def monitor_tf_cb(self):
        listener = tf.TransformListener()

        while not rospy.is_shutdown() and self.monitor_tfs:
            for topic,links in self.monitor_frames.items():
                try:
                    trans, rot = listener.lookupTransform(links[0], links[1], rospy.Time())
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn_throttle(10, "eef publisher error (printed every 10s): " + str(e))
                    continue
                msg = PoseStamped()
                msg.header.frame_id = links[0]
                msg.pose.position = Point()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = trans[0], trans[1], trans[2]
                msg.pose.orientation = Quaternion()
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rot[0], rot[1], rot[2], rot[3]
                self.msg_store[topic] = msg


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

    def write_bagfile(self, save_name):
        if len(self.segments)==0:
            rospy.logwarn("No frames recorded! Doing nothing.")
            return

        bag = rosbag.Bag(os.path.join(self.save_dir, save_name), "w")
        if self.first is None:
            self.first = self.segments[0]
        current_seg = self.first

        prev_max_time = rospy.Duration(0)
        while current_seg is not None:
            for frame in current_seg.frames:
                max_time = prev_max_time
                for topic, msg, raw_time in frame:
                    this_msg_time = raw_time + current_seg.dt
                    if this_msg_time > max_time:
                        max_time = this_msg_time

                    if topic == "{}{}".format(EEF_PREFIX, current_seg.rel_frame):
                        bag.write("{}relative".format(EEF_PREFIX), msg, t=this_msg_time)
                    bag.write(topic, msg, t=this_msg_time)
            prev_max_time = max_time
            current_seg = current_seg.next_seg
        bag.close()

    def record_keyframe(self, frame_id=None):
        if frame_id == "" or frame_id is None:
            frame_id = self.ARM_FRAME
        elif "{}{}".format(EEF_PREFIX, frame_id) not in self.msg_store:
            rospy.logwarn("Frame {} not found! Using arm frame...".format(frame_id))
            frame_id = self.ARM_FRAME

        dt = rospy.Time.now()-self.last_time
        frame = []
        prev_seg = self.segment_pointer
        if prev_seg is not None:
            next_seg = self.segment_pointer.next_seg
        else:
            next_seg = None
        for topic in self.msg_store:
            frame.append((topic, copy.deepcopy(self.msg_store[topic]), rospy.Duration(0.0)))

        frame.append(("is_joint_kf", Bool(data=self.is_joints), rospy.Duration(0.0)))
            
        new_seg = KTSegment(self.planner, self.gripper, [frame],
                            dt, prev_seg = prev_seg,
                            next_seg = next_seg,
                            is_traj = False, is_joints=self.is_joints, rel_frame=frame_id)
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
            new_seg = KTSegment(self.planner, self.gripper, frames,
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


    def initialize(self, name, is_joints = True):
        self.is_joints = is_joints
        self.recording = True
        self.last_time = rospy.Time.now()

        if len(self.segments)==0:
            self.first = None
            self.segment_pointer = None
        else:
            self.first = self.segments[0]
            self.segment_pointer = self.first
        
        if name[-4:]==".pkl":
            filename = name+".bag"
        elif name[-4:]==".bag":
            filename = name
        else:
            filename = name+".bag"
        full_save_path = self.save_dir+"/"+ filename
        if os.path.isfile(full_save_path):
            rospy.logwarn("Target output file already exists and will be overwritten when you save")
        self.release_arm()


    def stop_traj_record(self):
        self.record_traj = False
        self.record_thread.join()

    def start_traj_record(self):
        if not self.recording:
            rospy.logwarn("Cannot start trajectory logging until recording is started.  Start recording and try again.")
        self.record_thread = threading.Thread(target=self.record_trajectory)
        self.record_traj = True
        self.record_thread.start()

    def write_kf(self, rel_frame):
        if not self.recording:
            rospy.logwarn("Cannot write keyframe: recording not started.  Start recording and try again.")
        if self.record_traj:
            rospy.logwarn("Cannot write keyframe while recording trajectory. Stop recording trajectory and try again.")
            return
        self.record_keyframe(rel_frame)

    def end(self):
        if not self.recording:
            rospy.logwarn("Cannot end recording: recording was never started!")
            return
        if self.record_traj == True:
            self.stop_traj_record()
        self.recording = False
        return self.segments

    def stop_tf_threads(self):
        self.monitor_tfs = False


    def load_bagfile(self, bagfile, load_joints=None):
        self.last_time = rospy.Time.now()
        bag = rosbag.Bag(os.path.expanduser(bagfile), "r")
        msgs = []
        for topic, msg, time in bag.read_messages():
            # the following stupidity is brought to you by https://github.com/ros/ros_comm/issues/769
            msg_clean = None
            msg_class = None
            try:
                msg_class = self.msg_type_lookup(topic)
            except TypeError:
                msg_class = None

            if msg_class is None:
                # fallback attempts
                # a few types we know about:
                topic_mapping = {"_sensor_msgs__JointState": JointState,
                                 "_robotiq_85_msgs__GripperStat": GripperStat,
                                 "_std_msgs__Bool": Bool,
                                 "_geometry_msgs__Pose_stamped": PoseStamped
                                 }

                type_string = str(type(msg)).split(".")[1].split("'")[0]
                
                if topic.startswith(EEF_PREFIX):
                    # rospy.loginfo("Message on topic {} begins with {}, so interpreting it as "
                    #                "a PoseStamped.".format(topic, EEF_PREFIX))

                    msg_class = PoseStamped
                elif type_string in topic_mapping:
                    msg_class = topic_mapping[type_string]
                else:
                    rospy.logwarn("Message on topic {} could not have their type inferred because they are not found "
                                  "in the yaml file. Using provided type {}".format(topic, type(msg)))
                    msg_class = type(msg)

            def get_member_names(obj):
                return filter(lambda s: not s.startswith("_") and not callable(getattr(obj, s)), dir(obj))

            def recursive_copy(from_obj, to_obj):
                from_members = get_member_names(from_obj)
                to_members = get_member_names(to_obj)

                for attr in to_members:
                    try:
                        module = getattr(from_obj, attr).__module__
                    except AttributeError:
                        module = ""

                    if module.startswith("tmp"):
                        recursive_copy(getattr(from_obj, attr), getattr(to_obj, attr))
                    else:
                        setattr(to_obj, attr, getattr(from_obj, attr))

            if msg_class != type(msg) and msg_class is not None:
                # assign the fields in msg into a new instance of msg_class
                new_msg = msg_class()
                recursive_copy(msg, new_msg)
            else:
                new_msg = msg

            # end stupidity

            msgs.append((topic, new_msg, time))

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
        self.load_frames(frames, load_joints)

    #moves directly to the keyframe
    def move_to_keyframe(self, segment):
        self.lock_arm()
        first_seg = KTSegment(self.planner, self.gripper, segment.frames,
                              segment.dt,
                              is_traj = False,
                              is_joints = segment.is_joints)
        plan = first_seg.get_plan()
        success = False
        if plan is not None:
            self.set_gripper(self.segment_pointer.gripper_open)
            success = self.planner.move_robot(plan)
        if not success:
            rospy.logerr("Error moving to segment {}.".format(segment))
        return success

    def stop_robot(self):
        pass

    def move_forward(self):
        self.lock_arm()
        #print("SEGMENT POINTER:",self.segment_pointer)

        if self.segment_pointer is None or self.segment_pointer.next_seg is None:
            rospy.logwarn("No keyframe to move to! Doing nothing.")
            return

        if self.segment_pointer.is_joints and not self.at_seg_pointer():
            rospy.logwarn("Not at the keyframe we're pointing to. Making plan to get there.")
            success = self.move_to_keyframe(self.segment_pointer)
            if not success:
                rospy.logerr("Error moving to keyframe. Aborting.")
                return

        segment = self.segment_pointer.next_seg
        success = False
        plan = segment.get_plan()
        if plan is not None:
            self.set_gripper(segment.gripper_open)
            success = self.planner.move_robot(plan)
        if not success:
                rospy.logerr("Error moving to keyframe. Aborting.")
                return
        else:
            self.segment_pointer = segment

    def move_backward(self):
        self.lock_arm()
        #print("SEGMENT POINTER:",self.segment_pointer)
        if self.segment_pointer is None or self.segment_pointer.prev_seg is None:
            rospy.logwarn("No keyframe to move to! Doing nothing.")
            return

        if self.segment_pointer.is_joints and not self.at_seg_pointer():
            rospy.logwarn("Not at the keyframe we're pointing to. Making plan to get there.")
            success = self.move_to_keyframe(self.segment_pointer)
            if not success:
                rospy.logerr("Error moving to keyframe. Aborting.")
                return


        if self.segment_pointer.is_joints:
            success = self.planner.move_robot(self.segment_pointer.get_reversed_plan())
        else:
            success = self.move_to_keyframe(self.segment_pointer.prev_seg)

            
        if not success:
            rospy.logerr("Error moving to keyframe. Aborting.")
            return
        else:
            rospy.loginfo("Moved to segment.")
            self.set_gripper(self.segment_pointer.gripper_open)
            self.segment_pointer = self.segment_pointer.prev_seg
            #print("Moved to segment:",self.segment_pointer.frames)

    def at_seg_pointer(self):
        return self.at_keyframe_target(self.segment_pointer)

    def at_keyframe_target(self, segment):
        end_pose = segment.get_end_joints()
        if end_pose is None:
            return False
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



        if ((self.segment_pointer==self.first
             and not self.segment_pointer.is_joints)
            or (self.segment_pointer.is_joints
                and not self.at_keyframe_target(self.segment_pointer))):
            rospy.logwarn("Not at a keyframe. Moving to the last keyframe")
            success = self.move_to_keyframe(self.segment_pointer)
            if not success:
                rospy.logerr("Unable to move to last keyframe. Aborting!")
                return

        if self.segment_pointer == self.first:
            rospy.logwarn("Already at the start. Doing nothing.")
            return

        while self.segment_pointer != self.first:
            self.move_backward()

        
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

        while self.segment_pointer.next_seg is not None:
            self.move_forward()

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
        rospy.loginfo("Setting gripper. Open is {}".format(gripper_open))
        if gripper_open and (self.gripper_is_open is None or not self.gripper_is_open):
            self.open_gripper()
        if not gripper_open and (self.gripper_is_open is None or self.gripper_is_open):
            self.close_gripper()

        if wait:
            rospy.sleep(1.0)

    def insert_segment(self, new_segment, prev_seg=None, next_seg=None):
        """
        Create a new segment unless it is very similar to the supplied previous segment.
        """
        saved = False

        if prev_seg is None:
            self.segments.append(new_segment)
            saved = True
        else:
            should_make_new = True
            #only check if both are the same type
            if new_segment.is_joints and prev_seg.is_joints:
                target = new_segment.get_end_joints()
                last_target = prev_seg.get_end_joints()

                dist = 0
                for joint in target: #changed to max over joints
                    d1 = abs(last_target[joint]-target[joint])
                    if  d1 > dist:
                        dist = d1
                should_make_new = dist > self.JOINT_MOVE_THRESH
            elif (not new_segment.is_joints) and (not prev_seg.is_joints):
                # both are EEF keyframes
                target = new_segment.end
                last_target = prev_seg.end
                delta = [target.pose.position.x - last_target.pose.position.x,
                         target.pose.position.y - last_target.pose.position.y,
                         target.pose.position.z - last_target.pose.position.z]
                for axis in delta:
                    should_make_new |= abs(axis) > self.XYZ_MOVE_THRESH
                delta = [target.pose.orientation.x - last_target.pose.orientation.x,
                         target.pose.orientation.y - last_target.pose.orientation.y,
                         target.pose.orientation.z - last_target.pose.orientation.z,
                         target.pose.orientation.w - last_target.pose.orientation.w]
                for axis in delta:
                    should_make_new |= abs(axis) > self.QUAT_MOVE_THRESH
            if should_make_new:
                self.segments.append(new_segment)
                new_segment.set_prev(prev_seg)
                saved = True
            else:
                # didn't move, so update gripper; this is how the prev.
                # version worked
                rospy.logwarn("Robot didn't move but gripper might have changed. "
                              "Updating previous segment. This matches prior version "
                              "behavior but might not be the best long-term solution.")
                prev_seg.gripper_open = new_segment.gripper_open

        if saved and next_seg is not None:
            new_segment.set_next(next_seg)

        return saved

    def load_frames(self, frames, load_joints=None):
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


            if load_joints is None:
                joint_topic = "is_joint_kf"
                load_kf_joints = True
                for topic, msg, _ in frame:
                    if "is_joint_kf" == topic:
                        load_kf_joints = msg.data
                        rospy.loginfo("loaded {} as {}".format(joint_topic, str(msg.data)))
                if load_kf_joints is None:
                    rospy.logwarn("load_joints was not supplied, nor was it found in any of the bagfile messages (on the {} topic)."
                                  "defaulting load_joints True".format(joint_topic))
                    load_kf_joints = True
            else:
                load_kf_joints = load_joints
            delta_t = new_time-time
            #for i in range(len(frame)):
            #    old_item = frame[i]
            #    #the following only works for keyframes, not trajectories
            #    new_item = (frame[i][0],frame[i][1],rospy.Duration(0.0))

            print("load_kf_joints is " + str(load_kf_joints))
            new_segment = KTSegment(self.planner, self.gripper, [frame], delta_t, is_joints=load_kf_joints)
            if new_segment is None:
                continue

            saved = self.insert_segment(new_segment, prev_seg=last_segment)

            if saved:
                last_segment = new_segment
            else:
                rospy.loginfo("Discarded keyframe {} at time {}; not enough movement".format(len(self.segments)-1, time))
        self.segment_pointer = self.segments[0]


    def write_pkl(self, save_name):
        cPickle.dump((self.first, self.segments), open(os.path.join(self.save_dir, save_name), "wb"), cPickle.HIGHEST_PROTOCOL)

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

        first_seg = KTSegment(self.planner, self.gripper, last_seg.frames, last_seg.dt,
                              is_traj = False,
                              is_joints = self.is_joints)

        plan = first_seg.get_plan()

        if plan is None:
            rospy.logerr("Could not find plan for segment {}".format(first_seg))
            return


        display_trajectory=DisplayTrajectory()
        display_trajectory.trajectory_start = self.planner.robot.get_current_state()
        prev_time = rospy.Duration(0.0)

        display1 = DisplayTrajectory()
        display1.trajectory_start = first_seg.get_start_state()
        display1.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display1)
        rospy.sleep(plan.joint_trajectory.points[-1].time_from_start+rospy.Duration(0.5))

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
            plan = next_seg.get_plan()

            if plan is None:
                rospy.logerr("Could not find plan for segment {}".format(next_seg))
                return

            display1 = DisplayTrajectory()
            display1.trajectory_start = next_seg.get_start_state()
            display1.trajectory.append(plan)
            self.display_trajectory_publisher.publish(display1)
            rospy.sleep(plan.joint_trajectory.points[-1].time_from_start+rospy.Duration(0.5))


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
