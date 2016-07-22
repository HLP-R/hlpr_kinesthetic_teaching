#!/usr/bin/env python

import rospy
import time
import actionlib
import rosbag

from hlpr_kinesthetic_teaching.msg import PlaybackKeyframeDemoAction, PlaybackKeyframeDemoGoal, PlaybackKeyframeDemoResult, PlaybackKeyframeDemoFeedback
from std_msgs.msg import Int32, String

def feedback_call(feedback):
    print('[Feedback] time elapsed: %f'%feedback.time_elapsed)

rospy.init_node('playback_keyframe_demo_action_client')
client = actionlib.SimpleActionClient('playback_keyframe_demo', PlaybackKeyframeDemoAction)
client.wait_for_server()

time.sleep(1.0)

goal = PlaybackKeyframeDemoGoal()
goal.bag_file_name = 'demo_data_0'
goal.eef_only = False
client.send_goal(goal, feedback_cb=feedback_call)

client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))


