#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki_desktop/master/kobuki_qtestsuite/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import numpy
import operator
from python_qt_binding.QtCore import Signal, Slot, pyqtSlot
try:  # indigo
    from python_qt_binding.QtGui import QFrame, QVBoxLayout
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QFrame, QVBoxLayout
import math

import rospy
from qt_gui_py_common.worker_thread import WorkerThread
from kobuki_testsuite import Square

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.payload_frame_ui import Ui_payload_frame

##############################################################################
# Classes
##############################################################################

class PayloadFrame(QFrame):
    def __init__(self, parent=None):
        super(PayloadFrame, self).__init__(parent)
        self._gyro_topic_name = '/mobile_base/sensors/imu_data'
        self._ui = Ui_payload_frame()
        self._motion = None
        self._motion_thread = None

    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    def shutdown(self):
        '''
          Used to terminate the plugin
        '''
        rospy.loginfo("Kobuki TestSuite: payload frame shutdown")
        self._stop()

    ##########################################################################
    # Widget Management
    ##########################################################################

    def hibernate(self):
        '''
          This gets called when the frame goes out of focus (tab switch).
          Disable everything to avoid running N tabs in parallel when in
          reality we are only running one.
        '''
        self._stop()

    def restore(self):
        '''
          Restore the frame after a hibernate.
        '''
        pass

    ##########################################################################
    # Motion Callbacks
    ##########################################################################

    def _run_finished(self):
        self._motion_thread = None
        self._motion = None
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    ##########################################################################
    # Qt Callbacks
    ##########################################################################
    @Slot()
    def on_start_button_clicked(self):
        self._ui.start_button.setEnabled(False)
        self._ui.stop_button.setEnabled(True)
        self._motion = Square('/mobile_base/commands/velocity', '/odom', self._gyro_topic_name)
        self._motion.init(self._ui.speed_spinbox.value(), self._ui.distance_spinbox.value())
        self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
        self._motion_thread.start()

    @Slot()
    def on_stop_button_clicked(self):
        self._stop()

    def _stop(self):
        if self._motion:
            self._motion.stop()
        if self._motion_thread:
            self._motion_thread.wait()
            self._motion_thread = None
        if self._motion:
            self._motion = None
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    @pyqtSlot(float)
    def on_speed_spinbox_valueChanged(self, value):
        if self._motion:
            self._motion.init(self._ui.speed_spinbox.value(), self._ui.distance_spinbox.value())

    @pyqtSlot(float)
    def on_distance_spinbox_valueChanged(self, value):
        if self._motion:
            self._motion.init(self._ui.speed_spinbox.value(), self._ui.distance_spinbox.value())

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    #def robot_state_callback(self, data):
    #    if data.state == RobotStateEvent.OFFLINE:
    #        self.stop()

#
