#!/usr/bin/env python

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



