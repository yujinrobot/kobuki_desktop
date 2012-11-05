import os
from QtCore import Signal, Slot
from QtGui import QFrame
import math

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy

from python_qt_binding import loadUi

# Local resource imports
import detail.common_rc
import detail.wandering_rc
from detail.wandering_frame_ui import Ui_wandering_frame

class WanderingFrame(QFrame):
    def __init__(self, parent=None):
        super(WanderingFrame, self).__init__(parent)
        self._ui = Ui_wandering_frame()

    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    def shutdown(self):
        rospy.loginfo("Kobuki TestSuite: wandering test shutdown")
