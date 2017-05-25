import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QIcon, QWidget

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
        self.playDemoButton.setIcon(QIcon.fromTheme("media-playback-start"))

        # Attach event handlers
        self.demoLocationButton.clicked[bool].connect(self.browseForLocation)
        self.demoLocation.returnPressed.connect(self.loadLocation)

    def browseForLocation(self):
        location = QFileDialog.getOpenFileName()[0]
        if len(location) == 0:
            return

        self.demoLocation.setText(location)
        self.loadLocation()

    def loadLocation(self):
        location = self.demoLocation.text()
        if len(location) == 0:
            return

        print("Loading: " + location)