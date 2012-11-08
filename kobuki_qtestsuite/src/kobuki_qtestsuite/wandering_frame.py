import os
from QtCore import Signal, Slot, pyqtSlot
from QtGui import QFrame
import math

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy
from kobuki_testsuite import SafeWandering

from python_qt_binding import loadUi
import QtCore

# Local resource imports
import detail.common_rc
import detail.wandering_rc
from detail.wandering_frame_ui import Ui_wandering_frame
from qt_gui_py_common.worker_thread import WorkerThread

class WanderingFrame(QFrame):
    def __init__(self, parent=None):
        super(WanderingFrame, self).__init__(parent)
        self._ui = Ui_wandering_frame()
        self._motion = SafeWandering('/cmd_vel','/odom', '/mobile_base/events/bumper', '/mobile_base/events/cliff')
        self._motion_thread = None

    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())

    def shutdown(self):
        rospy.loginfo("Kobuki TestSuite: wandering test shutdown")
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

    @pyqtSlot(float)
    def on_speed_spinbox_valueChanged(self, value):
        # could use value, but easy to set like this
        self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())

    @pyqtSlot(float)
    def on_angular_speed_spinbox_valueChanged(self, value):
        # could use value, but easy to set like this
        self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())
