import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QIcon, QWidget, QMessageBox

from hlpr_record_demonstration.demonstration import Demonstration

class KinestheticTeachingWidget(QWidget):
    """
    Widget for use for kinesthetic teaching demos
    Handles all widget callbacks
    """

    def __init__(self, context):
        super(KinestheticTeachingWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_kinesthetic_teaching'), 'resource', 'KinestheticTeaching.ui')
        loadUi(ui_file, self)

        self.setObjectName('KTeachingUi')
        self.setWindowTitle(self.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self)

        # Set icons for buttons because they don't persist frm Qt creator for some reason
        self.demoLocationButton.setIcon(QIcon.fromTheme("document-open"))
        self.demoLocationSaveButton.setIcon(QIcon.fromTheme("document-save-as"))
        self.playDemoButton.setIcon(QIcon.fromTheme("media-playback-start"))

        # Attach event handlers
        self.demoLocationButton.clicked[bool].connect(self.browseForLocation)
        self.demoLocationSaveButton.clicked[bool].connect(self.newLocation)
        self.demoLocation.returnPressed.connect(self.loadLocation)
        self.startTrajectoryButton.clicked[bool].connect(self.startTrajectory)
        self.startButton.clicked[bool].connect(self.startKeyframe)
        self.addButton.clicked[bool].connect(self.addKeyframe)
        self.endButton.clicked[bool].connect(self.endKeyframe)

    def browseForLocation(self):
        location = QFileDialog.getOpenFileName()[0]
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
        if len(location) == 0:
            return

        print("Not implemented")
    
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
            print("Deleted existing save file")
        except OSError:
            pass

        # Initialize the demonstration recorder
        #rospy.init_node("demonstration_node", anonymous=False)
        #rospy.loginfo("Initializing the demonstration node")
        self.demonstration = Demonstration()
        self.demonstration.init_demo(custom_name = os.path.basename(location), new_dir = os.path.dirname(location), timestamp = self.shouldTimestamp.isChecked())
        self.demoLocation.setText(self.demonstration.filename)
        self.demoName.setText(os.path.basename(self.demonstration.filename))

    def _showWarning(self, title, body):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowTitle(title)
        msg.setText(body)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()
    def startTrajectory(self):
        success = self.demonstration.start_trajectory()
        if not success:
            self._showWarning("Could not start recording", "Failed to start trajectory recording. A recording is already in progress.")
    def startKeyframe(self):
        success = self.demonstration.start_keyframe()
        if not success:
            self._showWarning("Could not start recording", "Failed to start keyframe recording. A recording is already in progress.")
    def addKeyframe(self):
        success = self.demonstration.write_keyframe()
        if not success:
            self._showWarning("Could not record keyframe", "Failed to record keyframe. A recording is not currently in progress.")
    def endKeyframe(self):
        success = self.demonstration.stop_recording()
        if not success:
            if not self.demonstration.recording:
                text = "Failed to end recording. A recording is currently not in progress."
            else:
                text = "Failed to end recordning for an unknown reason. Check the logs for more information."
            self._showWarning("Could not end recording", text)