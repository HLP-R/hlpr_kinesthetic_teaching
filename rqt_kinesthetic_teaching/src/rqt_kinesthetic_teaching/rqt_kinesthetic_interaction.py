import os
import signal

import rospy
from hlpr_kinesthetic_interaction.jaco_arm import Arm
from hlpr_kinesthetic_interaction.kinesthetic_interaction import \
    KinestheticInteraction
from hlpr_record_demonstration.demonstration import Demonstration
from object_location.srv import LocationQuery


class TimeoutException(Exception):
    pass

class RQTKinestheticInteraction(KinestheticInteraction):

    def __init__(self, verbose=False, arm_class=Arm):
        self.arm_class = arm_class

        super(RQTKinestheticInteraction, self).__init__(verbose=verbose)

        # Initialize demonstration recorder (might hang if the record demonstration action server hasn't been started)
        signal.alarm(2)
        signal.signal(signal.SIGALRM, self._demonstrationTimeoutHandler)
        self.demonstration = Demonstration()
        self.demonstration.init_demo()
        signal.alarm(0)

        # Define callbacks
        self.start_trajectory_cb = None
        self.start_keyframe_cb = None
        self.add_keyframe_cb = None
        self.end_keyframe_cb = None

        self.object_locator = rospy.ServiceProxy("location_query", LocationQuery)
        self.should_locate_objects = True

    def _demonstrationTimeoutHandler(self, signum, frame):
        msg = "Could not load record keyframe demo server. Run `roslaunch hlpr_record_demonstration start_record_services.launch`."
        rospy.logerr(msg)
        raise TimeoutException(msg)

    def init_demo(self, location = None, timestamp = False):
        self.demonstration.init_demo(
            custom_name=os.path.basename(location),
            new_dir=os.path.dirname(location),
            timestamp=timestamp
        )
        return self.demonstration.filename

    # Override abstract methods
    def apply_hand_action(self, cmd, hand):
        success = self.demonstration.write_keyframe()
        self.add_keyframe_cb(success)

    def apply_arm_action(self, cmd, arm):
        #rospy.loginfo("Received command: %s", cmd)
        pass

    def _locate_objects(self):
        if self.should_locate_objects:
            rospy.loginfo("Waiting for object locator service...")
            self.object_locator()
            rospy.loginfo("Object locator service finished")

    def demonstration_start(self, cmd):
        self._locate_objects()
        success = self.demonstration.start_keyframe()
        self.start_keyframe_cb(success)

    def demonstration_keyframe(self, cmd):
        success = self.demonstration.write_keyframe()
        self.add_keyframe_cb(success)

    def demonstration_end(self, cmd):
        success = self.demonstration.stop_recording()
        self.end_keyframe_cb(success)

    def demonstration_start_trajectory(self, cmd):
        self._locate_objects()
        success = self.demonstration.start_trajectory()
        self.start_trajectory_cb(success)
 
    def demonstration_end_trajectory(self, cmd):
        success = self.demonstration.stop_recording()
        self.end_keyframe_cb(success)
