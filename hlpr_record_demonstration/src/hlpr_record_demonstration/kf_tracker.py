#!/usr/bin/env python
# Author: Vivian Chu, vchu@gatech.edu

import rospy
from std_msgs.msg import Int8
from hlpr_record_demonstration.srv import ChangeKF, ChangeKFResponse

""" kf_tracker.py
simple tracker that tracks the current KF for recording purposes
"""
class KFTracker():

    TOPIC_PARAM = "kf_track_topic"
    SERVICE_NAME = "kf_change_service"
    DEFAULT_RATE = 100
    DEFAULT_TOPIC = "kf_tracker/state"
    IDLE_KF = -1

    def __init__(self):

        # Everything is ready, now init the node
        rospy.init_node("kf_tracker")

        # Get the frequency of publishing
        self.publish_rate = rospy.get_param("~kf_rate", self.DEFAULT_RATE)
        self.rate = rospy.Rate(self.publish_rate)

        # Get the topic to publish to
        self.publish_topic = rospy.get_param("~kf_topic", self.DEFAULT_TOPIC)

        # Store the topic name
        rospy.set_param(self.TOPIC_PARAM, self.publish_topic)

        # Setup the publisher
        self.pub = rospy.Publisher(self.publish_topic, Int8, queue_size=10)

        # Setup service to change the current KF number
        self.kf_service = rospy.Service(self.SERVICE_NAME, ChangeKF, self._update_kf)

        # Initialize the state
        self.kf_num = self.IDLE_KF

        rospy.loginfo("Done setting up keyframe tracker")

    def _update_kf(self, req):

        # Check what to do with KF req
        if req.increment:
            self.kf_num+=1
        elif req.decrement:
            self.kf_num-+1
        else:
            self.kf_num = req.kf_num

        rospy.loginfo("Change KF to: %d" % self.kf_num)
        resp = ChangeKFResponse()
        resp.response = True
        return resp

    def _start_publish(self):

        # Continuously publish
        while not rospy.is_shutdown():

            self.pub.publish(self.kf_num)
            self.rate.sleep()

if __name__ == '__main__':
    tracker = KFTracker()
    tracker._start_publish()
