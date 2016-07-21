#!/usr/bin/env python

import rospy
import tf 
import time
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState


def eef_pose_pub():
  rospy.init_node('eef_publisher')
  listener = tf.TransformListener()
  pub = rospy.Publisher('eef_pose', Pose, queue_size=10)
  rate = rospy.Rate(100)
  while not rospy.is_shutdown():
    try: 
      trans, rot = listener.lookupTransform('/base_link', '/right_ee_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    msg = Pose()
    msg.position = Point()
    msg.position.x, msg.position.y, msg.position.z = trans[0], trans[1], trans[2]
    msg.orientation = Quaternion() 
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = rot[0], rot[1], rot[2], rot[3]
    pub.publish(msg)
    rate.sleep()

if __name__ =='__main__':
  try:
    eef_pose_pub()
  except rospy.ROSInterruptException:
    pass



