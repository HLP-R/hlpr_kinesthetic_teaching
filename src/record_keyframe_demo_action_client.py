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

import rospy
import time
import actionlib
import rosbag

from hlpr_kinesthetic_teaching.msg import RecordKeyframeDemoAction, RecordKeyframeDemoGoal, RecordKeyframeDemoResult, RecordKeyframeDemoFeedback
from std_msgs.msg import Int32, String

pub = rospy.Publisher('record_demo_frame', Int32, queue_size=10)

demo_num = 0

def feedback_call(feedback):
    print('[Feedback] num keyframes: %f'%feedback.num_keyframes)

rospy.init_node('record_keyframe_demo_action_client')
client = actionlib.SimpleActionClient('record_keyframe_demo', RecordKeyframeDemoAction)
client.wait_for_server()

goal = RecordKeyframeDemoGoal()
goal.start_demo = 1.0

goal.bag_file_name = 'demo_data_'+str(demo_num)
client.send_goal(goal, feedback_cb=feedback_call)


print 'testing a KF demo with lots of frames, goal file name = '
print goal.bag_file_name

pub.publish(1)

frames = 0
while frames < 100: 
  pub.publish(2)
  time.sleep(.1)
  frames += 1

pub.publish(3)
demo_num += 1

client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
print('[Result] Num Keyframes Recorded: %f'%(client.get_result().num_keyframes))

print 'see what the bag file is like:'
bag = rosbag.Bag(goal.bag_file_name)
for topic, msg, t in bag.read_messages(topics=['joint_states', 'eef_pose']):
  print msg
bag.close()


################



goal.bag_file_name = 'demo_data_'+str(demo_num)
client.send_goal(goal, feedback_cb=feedback_call)

raw_input("press any key to start a demo with 3 keyframes")

print 'goal file name = '
print goal.bag_file_name

raw_input("press any key to record keyframe number 1 (start)")
pub.publish(1)

raw_input("press any key to record keyframe number 2 (mid)")
pub.publish(2)

raw_input("press any key to record keyframe number 2 (final)")
pub.publish(3)

demo_num += 1

client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
print('[Result] Num Keyframes Recorded: %f'%(client.get_result().num_keyframes))

print 'see what the bag file is like:'
bag = rosbag.Bag(goal.bag_file_name)
for topic, msg, t in bag.read_messages(topics=['joint_states', 'eef_pose']):
  print msg
bag.close()


####################

print 'testing a trajectory demo'
goal.bag_file_name = 'demo_data_'+str(demo_num)
client.send_goal(goal, feedback_cb=feedback_call)

time.sleep(1.0)
pub.publish(0)
time.sleep(1.0)
pub.publish(3)

demo_num += 1

client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
print('[Result] Num Keyframes Recorded: %f'%(client.get_result().num_keyframes))

print 'see what the bag file is like:'
bag = rosbag.Bag(goal.bag_file_name)
for topic, msg, t in bag.read_messages(topics=['joint_states', 'eef_pose']):
  print msg
bag.close()



