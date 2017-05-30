import os
import rosbag
import rospy
import rospkg
import signal
import time
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QIcon, QWidget, QMessageBox, QHeaderView, QTreeWidgetItem

from hlpr_record_demonstration.msg import RecordKeyframeDemoAction
from hlpr_record_demonstration.demonstration import Demonstration
from keyframe_bag_interface import ParseException, KeyframeBagInterface

class TimeoutException(Exception):
    pass

class KinestheticTeachingWidget(QWidget):
    """
    Widget for use for kinesthetic teaching demos
    Handles all widget callbacks
    """

    STATUS_DISPLAY_TIME = 3 # seconds

    def __init__(self, context):
        super(KinestheticTeachingWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_kinesthetic_teaching'), 'resource', 'KinestheticTeaching.ui')
        loadUi(ui_file, self)

        self.setObjectName('KTeachingUi')
        self.setWindowTitle(self.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self)

        # Set icons for buttons because they don't persist frm Qt creator for some reason
        self.demoLocationSaveButton.setIcon(QIcon.fromTheme("document-save-as"))
        self.demoLocationButton.setIcon(QIcon.fromTheme("document-open"))
        self.demoFolderLocationButton.setIcon(QIcon.fromTheme("folder"))
        self.playDemoButton.setIcon(QIcon.fromTheme("media-playback-start"))

        # Attach event handlers
        self.demoLocationButton.clicked[bool].connect(self.browseForLocation)
        self.demoFolderLocationButton.clicked[bool].connect(self.browseForFolder)
        self.demoLocationSaveButton.clicked[bool].connect(self.newLocation)
        self.demoLocation.returnPressed.connect(self.loadLocation)
        self.startTrajectoryButton.clicked[bool].connect(self.startTrajectory)
        self.startButton.clicked[bool].connect(self.startKeyframe)
        self.addButton.clicked[bool].connect(self.addKeyframe)
        self.endButton.clicked[bool].connect(self.endKeyframe)
        self.playDemoButton.clicked[bool].connect(self.playDemo)

        # Set sizing options for tree widget headers
        self.playbackTree.header().setStretchLastSection(False)
        self.playbackTree.header().setResizeMode(0, QHeaderView.Stretch)
        self.playbackTree.header().setResizeMode(1, QHeaderView.ResizeToContents)

        self.previousStatusText = None

    def _showWarning(self, title, body):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowTitle(title)
        msg.setText(body)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()
    def _showStatus(self, text):
        self.status.setText(text)
        self.previousStatusText = text
        threading.Timer(self.STATUS_DISPLAY_TIME, self._expireStatus).start()
    def _expireStatus(self):
        if self.status.text() == self.previousStatusText:
            self.status.setText("Ready.")

    def browseForLocation(self):
        location = QFileDialog.getOpenFileName(filter = "*.bag;;*")[0]
        if len(location) == 0:
            return

        self.demoLocation.setText(location)
        self.loadLocation()
    def browseForFolder(self):
        location = QFileDialog.getExistingDirectory()
        if len(location) == 0:
            return

        self.demoLocation.setText(location)
        self.loadLocation()
    def loadLocation(self):
        self.startTrajectoryButton.setEnabled(False)
        self.startButton.setEnabled(False)
        self.addButton.setEnabled(False)
        self.endButton.setEnabled(False)

        location = self.demoLocation.text()
        if os.path.isdir(location):
            locations = [os.path.join(location, f) for f in os.listdir(location) if os.path.isfile(os.path.join(location, f)) and f.split(".")[-1] == "bag"]
        else:
            locations = [location]

        if len(locations) == 0 or len(locations[0]) == 0:
            return
        
        self.keyframeCount.setText("")
        self.playbackTree.clear()
        self._showStatus("Parsing...")
        
        totalFrames = 0
        for location in sorted(locations):
            try:
                parsedData = KeyframeBagInterface().parse(location)
            except (rosbag.bag.ROSBagException, ParseException) as err:
                self._showStatus(str(err))
                rospy.logwarn("[%s] %s", location, str(err))
                self.playbackTree.clear()
                return
            totalFrames += len(parsedData)

            items = []
            for i, keyframe in enumerate(parsedData):
                item = QTreeWidgetItem()
                title = "(#{}) ".format(i) + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(keyframe["time"]))
                item.setText(0, title)
                # Add children
                for topic, data in keyframe["data"].items():
                    topicItem = QTreeWidgetItem()
                    topicItem.setText(0, topic)
                    for attribute, value in data.items():
                        attributeValueItem = QTreeWidgetItem()
                        attributeValueItem.setText(0, attribute)
                        attributeValueItem.setText(1, str(value))
                        topicItem.addChild(attributeValueItem)
                    item.addChild(topicItem)
                items.append(item)
            if len(locations) == 1:
                self.playbackTree.addTopLevelItems(items)
            else:
                item = QTreeWidgetItem()
                item.setText(0, location)
                item.addChildren(items)
                self.playbackTree.addTopLevelItem(item)
        
        if len(locations) == 1:
            self.demoName.setText(os.path.basename(locations[0]))
            self.keyframeCount.setText("{} keyframe(s) loaded".format(totalFrames))
        else:
            self.demoName.setText(os.path.basename(self.demoLocation.text()) + os.path.sep)
            self.keyframeCount.setText("{} keyframe(s) loaded from {} files".format(totalFrames, len(locations)))
        self._showStatus("Parsed {} keyframe(s).".format(totalFrames))
        self.playDemoButton.setEnabled(True)
    
    def _getDemonstrationHandler(self, signum, frame):
        msg = "Could not load record keyframe demo server. Run `rosrun hlpr_record_demonstration record_demonstration_action_server.py`."
        rospy.logerr(msg)
        raise TimeoutException(msg)
    def newLocation(self):
        location = QFileDialog.getSaveFileName(filter = "*.bag;;*")[0]
        if len(location) == 0:
            return
        self.demoLocation.setText(location)
        self.startTrajectoryButton.setEnabled(True)
        self.startButton.setEnabled(True)
        self.addButton.setEnabled(True)
        self.endButton.setEnabled(True)

        try:
            os.remove(location)
            self._showStatus("Deleted existing save file.")
        except OSError:
            pass

        # Initialize the demonstration recorder
        #rospy.init_node("demonstration_node", anonymous=False)
        #rospy.loginfo("Initializing the demonstration node")
        signal.signal(signal.SIGALRM, self._getDemonstrationHandler)
        signal.alarm(2)
        try:
            self.demonstration = Demonstration()
        except TimeoutException as err:
            self._showWarning("Record keyframe demo server unreachable", str(err))
            return
        signal.alarm(0)
        self.demonstration.init_demo(custom_name = os.path.basename(location), new_dir = os.path.dirname(location), timestamp = self.shouldTimestamp.isChecked())
        self.demoLocation.setText(self.demonstration.filename)
        self.demoName.setText(os.path.basename(self.demonstration.filename))
        self.keyframeCount.setText("")
        self.playbackTree.clear()

    def startTrajectory(self):
        success = self.demonstration.start_trajectory()
        if not success:
            self._showWarning("Could not start recording", "Failed to start trajectory recording. A recording is already in progress.")
        else:
            self._showStatus("Trajectory started.")
    def startKeyframe(self):
        success = self.demonstration.start_keyframe()
        if not success:
            self._showWarning("Could not start recording", "Failed to start keyframe recording. A recording is already in progress.")
        else:
            self.keyframeCount.setText("0 keyframe(s) recorded")
            self._showStatus("Keyframe recording started.")
    def addKeyframe(self):
        success = self.demonstration.write_keyframe()
        if not success:
            self._showWarning("Could not record keyframe", "Failed to record keyframe. A recording is not currently in progress.")
        else:
            item = QTreeWidgetItem()
            title = "(#{}) ".format(self.playbackTree.topLevelItemCount()) + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            item.setText(0, title)
            self.playbackTree.addTopLevelItem(item)
            self.playbackTree.scrollToItem(item)
            self.keyframeCount.setText("{} keyframe(s) recorded".format(self.playbackTree.topLevelItemCount()))
            self._showStatus("Keyframe recorded.")
    def endKeyframe(self):
        success = self.demonstration.stop_recording()
        if not success:
            if not self.demonstration.recording:
                text = "Failed to end recording. A recording is currently not in progress."
            else:
                text = "Failed to end recordning for an unknown reason. Check the logs for more information."
            self._showWarning("Could not end recording", text)
        else:
            self._showStatus("Recording saved.")
            self.loadLocation()

    def _playDemoHandler(self, signum, frame):
        msg = "Could not load playback keyframe demo server. Run `rosrun hlpr_record_demonstration playback_demonstration_action_server.py`."
        rospy.logerr(msg)
        raise TimeoutException(msg)
    def playDemo(self):
        location = self.demoLocation.text()
        keyframeBagInterface = KeyframeBagInterface()

        signal.signal(signal.SIGALRM, self._playDemoHandler)
        signal.alarm(2)
        try:
            keyframeBagInterface.playInit()
        except TimeoutException as err:
            self._showWarning("Playback keyframe demo server unreachable", str(err))
            return
        signal.alarm(0)

        self._showStatus("Playing...")
        keyframeBagInterface().play(location, self.playDemoDone)
    def playDemoDone(self, feedback):
        print(feedback)
        pass