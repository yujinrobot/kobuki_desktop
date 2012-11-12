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
from python_qt_binding.QtGui import QFrame, QVBoxLayout
import math

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy
from qt_gui_py_common.worker_thread import WorkerThread

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.gyro_drift_frame_ui import Ui_gyro_drift_frame

##############################################################################
# Classes
##############################################################################

class GyroDriftFrame(QFrame):
    def __init__(self, parent=None):
        super(GyroDriftFrame, self).__init__(parent)
        self._ui = Ui_gyro_drift_frame()
        #self._motion = Rotate('/cmd_vel')
        #self.robot_state_subscriber = rospy.Subscriber("/mobile_base/events/robot_state", RobotStateEvent, self.robot_state_callback)
        self._motion_thread = None

    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        #self._motion.init(self._ui.angular_speed_spinbox.value())

    def shutdown(self):
        rospy.loginfo("Kobuki TestSuite: gyro drift shutdown")
        #self._motion.shutdown()

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
        #self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
        #self._motion_thread.start()

    @Slot()
    def on_stop_button_clicked(self):
        self.stop()
        
    def stop(self):
        #self._motion.stop()
        if self._motion_thread:
            self._motion_thread.wait()
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        
    @pyqtSlot(float)
    def on_angular_speed_spinbox_valueChanged(self, value):
        pass
        #self._motion.init(self._ui.angular_speed_spinbox.value())

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    #def robot_state_callback(self, data):
    #    if data.state == RobotStateEvent.OFFLINE:
    #        self.stop()

#