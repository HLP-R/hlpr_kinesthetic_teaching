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

# This is an example of how to make use of the record_keyframe_demo_action_server class

import rospy
import time
import actionlib
import rosbag
from hlpr_record_demonstration.record_demonstration_action_server import RecordKFDemoAction

#import the messages related to the RecordKeyframeDemoAction
from hlpr_record_demonstration.msg import RecordKeyframeDemoAction, RecordKeyframeDemoGoal, RecordKeyframeDemoResult, RecordKeyframeDemoFeedback
from std_msgs.msg import Int32, String

#in order to record demonstrations, you just publish the right message to this topic
pub = rospy.Publisher('record_demo_frame', Int32, queue_size=10)

#the action server implements a feedback service, if you want to 
#monitor feedback during the recording action make a callback function
#currently the feedback is number of keyframes (or time elapsed in traj demo)
def feedback_call(feedback):
    print('[Feedback] num keyframes: %f'%feedback.num_keyframes)

rospy.init_node('record_keyframe_demo_action_client')

#make your action client
client = actionlib.SimpleActionClient('record_keyframe_demo', RecordKeyframeDemoAction)
client.wait_for_server()

#send a goal to the action client, with the name of the bag_file
goal = RecordKeyframeDemoGoal()
goal.bag_file_name = 'demo_data_0'
goal.trajectory = False
client.send_goal(goal, feedback_cb=feedback_call)

## Example keyframe demo: 

## Send a start here command "1" 
#pub.publish(1) # no longer need

## Then send any number of record keyframe "2" 
frames = 0
while frames < 10: 
  pub.publish(RecordKFDemoAction.KF)
  time.sleep(.1)
  frames += 1

## Send an end here command "3"
pub.publish(RecordKFDemoAction.DEMO_END)

## check the result...
client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
print('[Result] Num Keyframes Recorded: %f'%(client.get_result().num_keyframes))

## uncomment this if you want to print out the bag file you just recorded
##print 'see what the bag file is like:'
##bag = rosbag.Bag(goal.bag_file_name)
##for topic, msg, t in bag.read_messages(topics=['joint_states', 'eef_pose']):
##  print msg
##bag.close()



################

## Example trajectory demo: 

print 'testing a trajectory demo, will record for 1 sec, at 100hz'

# keep using the same action client, just send it new goals each
# time you want to record a new demo
goal.bag_file_name = 'demo_data_1'
client.send_goal(goal, feedback_cb=feedback_call)

#pub.publish(0) # no longer need - sending goal starts
time.sleep(1.0)
pub.publish(RecordKFDemoAction.DEMO_END)
goal.trajectory = True

client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
print('[Result] Num Keyframes Recorded: %f'%(client.get_result().num_keyframes))

## uncomment this if you want to print out the bag file you just recorded
##print 'see what the bag file is like:'
##bag = rosbag.Bag(goal.bag_file_name)
##for topic, msg, t in bag.read_messages(topics=['joint_states', 'eef_pose']):
##  print msg
##bag.close()



