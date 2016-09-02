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

# Author: Vivian Chu, vchu@diligentdroids.com

"""
moveit_playback_state.py

State that takes a path to the bag file to playback and sends it off to the
playback server. Also requires specification if we're working in EEF space 
or joint space
"""

import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import String
from hlpr_record_demonstration.msg import PlaybackKeyframeDemoAction, PlaybackKeyframeDemoGoal, PlaybackKeyframeDemoResult, PlaybackKeyframeDemoFeedback

class MoveitPlaybackBagState(smach.State):

    def __init__(self, bagfile=None, eef_flag=False, pose_topic="joint_states", bagparam=None):
        # bag_file: the path to bag play to playback
        # eef_flag: True - plan in EEF space, False - plan in joint space
        # pose_topic: the topic that the poses are on (e.g. joint_states, eef_pose)
        smach.State.__init__(self, outcomes=['playback_finished'])
        self.playback_client = actionlib.SimpleActionClient('playback_keyframe_demo', PlaybackKeyframeDemoAction)
        rospy.logwarn("Warning for playback server to load")
        self.playback_client.wait_for_server()
        rospy.logwarn("Playback server loaded")

        # We can either set the bagfile name when creating the state OR through rosparam
        if bagfile is None:
            if bagparam is None:
                # Error out!
                rospy.logerr("Exiting: not a valid state. No given bagfile name or rosparam")
                exit()
            else:
                self._bagfile = rospy.get_param(bagparam)
        else:
            self._bagfile = bagfile

        self._eef_flag = eef_flag
        self._pose_topic = pose_topic

    def execute(self, userdata):

        rospy.loginfo("Sending goal to playback server")

        # Create goal and fill in information
        goal = PlaybackKeyframeDemoGoal()
        goal.bag_file_name = self._bagfile
        goal.eef_only = self._eef_flag
        goal.target_topic = self._pose_topic

        # Send goal to server
        self.playback_client.send_goal(goal)
        rospy.loginfo("Waiting for playback to finish")
        self.playback_client.wait_for_result()
        rospy.loginfo(self.playback_client.get_goal_status_text())

        return 'playback_finished'

"""
Example below on how to call the state

To run - publish a string to the start_playback topic

"""


def transition_cb(userdata, msg):

    rospy.loginfo("Received message to start. Transitioning to playback state.")

    # Need to return false to break out of this state
    return False


def main():

    rospy.init_node("moveit_playback_demonstration")

    ROSBAG_PARAM_NAME = "~bagfile"
    rospy.set_param(ROSBAG_PARAM_NAME, "/home/ddroids/data/demo_01.bag")

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        # Simple monitor state
        smach.StateMachine.add('IDLE', smach_ros.MonitorState("/start_playback", String, transition_cb, output_keys=['bag_file', 'eef_flag','target_topic']), transitions={'invalid': 'PLAYBACK', 'valid':'IDLE', 'preempted':'IDLE'})

        # Example below on how to set the bag file with a specific state init
        #smach.StateMachine.add('PLAYBACK', MoveitPlaybackBagState(bagfile="/home/ddroids/data/demo_01.bag", eef_flag=False, pose_topic='joint_states'), transitions={'playback_finished':'DONE'})
        
        # Example of passing in a rosparam location for the bagfile name to be
        smach.StateMachine.add('PLAYBACK', MoveitPlaybackBagState(bagparam=ROSBAG_PARAM_NAME, eef_flag=False, pose_topic='joint_states'), transitions={'playback_finished':'DONE'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__=="__main__":
    main()


