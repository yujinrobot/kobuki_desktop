import os
import math

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy
from rospy import Rate
from kobuki_testsuite import TravelForward

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal,Slot
from python_qt_binding.QtGui import QFrame

# Local resource imports
import detail.common_rc
import detail.climbing_rc
from detail.climbing_frame_ui import Ui_climbing_frame
from qt_gui_py_common.worker_thread import WorkerThread

class ClimbingFrame(QFrame):
    def __init__(self, parent=None):
        super(ClimbingFrame, self).__init__(parent)
        self._ui = Ui_climbing_frame()
        self._motion = TravelForward('/cmd_vel','/odom')
        self._motion_thread = None

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
        self._motion.init(0.7, 1.0)
        self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
        self._motion_thread.start()

    @Slot()
    def on_stop_button_clicked(self):
        '''
          Hardcore stoppage - straight to zero.
        '''
        self._motion.stop()
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
