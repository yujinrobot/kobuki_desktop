import os
from QtCore import Signal, Slot
from QtGui import QFrame
import math

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy
import kobuki_testsuite
#from kobuki_testsuite import TravelForward

from python_qt_binding import loadUi

# Local resource imports
import detail.common_rc
import detail.climbing_rc
from detail.climbing_frame_ui import Ui_climbing_frame

class ClimbingFrame(QFrame):
    def __init__(self, parent=None):
        super(ClimbingFrame, self).__init__(parent)
        self._ui = Ui_climbing_frame()
        kobuki_testsuite.foo()
        self._motion = kobuki_testsuite.TravelForward('/cmd_vel','/odom')


    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    def shutdown(self):
        rospy.loginfo("Kobuki TestSuite: climbing test shutdown")
        self._motion.shutdown()
        
    ##########################################################################
    # Motion Callbacks
    ##########################################################################

    def _run_finished(self):
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    ##########################################################################
    # Qt Callbacks
    ##########################################################################
    @Slot()
    def on_start_button_clicked(self):
        self._ui.start_button.setEnabled(False)
        self._ui.stop_button.setEnabled(True)
        self._motion.start(0.7, 1.0, self._run_finished)

    @Slot()
    def on_stop_button_clicked(self):
        '''
          Hardcore stoppage - straight to zero.
        '''
        self._motion.stop()
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
