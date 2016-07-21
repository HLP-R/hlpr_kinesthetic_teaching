#!/usr/bin/env python

import rospy
import actionlib
import time
import rosbag
import string

from my_awesome_code.msg import RecordKeyframeDemoAction, RecordKeyframeDemoGoal, RecordKeyframeDemoResult, RecordKeyframeDemoFeedback
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# RecordKFDemoAction:
#  this is an action server that will record a single keyframe demo each time it is initiated.  
#  once the action is started, keyframes are triggered with the message \record_demo_frame 
#  a bag is created with the name passed in, each keyframe writes data to bag, bag closed on last kf
#
# ---TBD: Currently this code assumes someone is publishing topics joint_states and eef_pose
#        topics to record should be defined by specific instantiations that inherit from this class?

class RecordKFDemoAction(object):

  demo_done = False
  rec_whole_trajectory = False
  bagfile_name = String()

  current_jt_st = JointState()
  current_eef_st = Pose()
  keyframe_count = 0
  feedback = RecordKeyframeDemoFeedback()
  result = RecordKeyframeDemoResult()

  def __init__(self):
    self.server = actionlib.SimpleActionServer('record_keyframe_demo', RecordKeyframeDemoAction, execute_cb=self.do_record_keyframe_demo, auto_start=False)
    self.server.start()
    self.listen_record_msgs = rospy.Subscriber('record_demo_frame', Int32, self.keyframe_callback)
    self.listen_jt = rospy.Subscriber('joint_states', JointState, self.jt_st_callback)
    self.listen_eef = rospy.Subscriber('eef_pose', Pose, self.eef_callback)
    
  #subscriber for joint_state topic, update most recent data
  def jt_st_callback(self, msg):
    self.current_jt_st = msg

  #subscriber for eef_pose topic, update most recent data
  def eef_callback(self, msg):
    self.current_eef_st = msg

  #keyframe message triggers recording of \joint_state to \demo_data
  #Important...this callback assumed to run after do_record_keyframe_demo()
  #TBD come up with a convention for trigger values, etc.]
  # Current funcionality
  #    0 = Start Trajectory Demo (doesn't work, bag file gets corrupted for some reason)
  #    1 = Keyframe demo: Start Here 
  #    2 = Keyframe demo: Go Here  
  #    3 = Keyframe or Traj demo: End Here
  def keyframe_callback(self, msg):
    if msg.data == 1:  #start message keyframe demo
      print 'recording first keyframe now! starting rosbag'
      print self.bagfile_name
      self.bag = rosbag.Bag(self.bagfile_name, 'w')
    elif msg.data == 0: #start message traj demo
      print 'starting trajectory demo, starting rosbag'
      print self.bagfile_name
      self.bag = rosbag.Bag(self.bagfile_name, 'w')
      self.rec_whole_trajectory = True
    elif msg.data == 3: #end message
      print 'recording last keyframe now!'
      self.demo_done = True
    elif msg.data != 2: 
      print 'record_demo_frame should be called with 0-3'
      return
   
    self.write_demo_data()
 
    if self.demo_done == True:
      self.bag.close()
      print 'bag file closed'
 
  #write current state to the open bag
  #TBD: don't hard code what data is saved
  def write_demo_data(self):    
    self.bag.write('joint_states', self.current_jt_st)
    self.bag.write('eef_pose', self.current_eef_st)
    self.keyframe_count += 1
  
  #start performing the record_keyframe_demo action
  #goal includes the bagfile name to use for this demo
  def do_record_keyframe_demo(self, goal):
    self.keyframe_count = 0
    self.demo_done = False
    self.write_whole_trajectory = False
    self.bagfile_name = goal.bag_file_name

    while self.demo_done == False:
      if self.server.is_preempt_requested():
        self.result.num_keyframes = self.keyframe_count
        self.server.set_preempted(self.result, "record keyframe demo preempted")  
        if self.demo_done == False:
          self.bag.close()      
        print 'preempted'
        return
      elif self.keyframe_count > 0: #wait to start feedback after first keyframe 
        self.feedback.num_keyframes = self.keyframe_count
        self.server.publish_feedback(self.feedback)
        time.sleep(.5)  #throttle feedback messages 
 
    self.result.num_keyframes = self.keyframe_count
    self.server.set_succeeded(self.result, "Record Keyframe Demo completed successfully")
    return
    

if __name__ =='__main__':
  rospy.init_node('record_keyframe_demo_action_server')
  RecordKFDemoAction()
  rospy.spin()


