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

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy
from qt_gui_py_common.worker_thread import WorkerThread
from kobuki_testsuite import TravelForward

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.cliff_sensor_frame_ui import Ui_cliff_sensor_frame

##############################################################################
# Classes
##############################################################################

class CliffSensorFrame(QFrame):
    STATE_FORWARD = "forward"
    STATE_BACKWARD = "backward"
    STATE_STOPPED = "stopped"
        
    def __init__(self, parent=None):
        super(CliffSensorFrame, self).__init__(parent)
        self._ui = Ui_cliff_sensor_frame()
        self._motion = TravelForward('/mobile_base/commands/velocity','/odom', '/mobile_base/events/cliff')
        self._motion_thread = None
        self._distance = 1.2
        self._state = CliffSensorFrame.STATE_FORWARD
        self._is_alive = False # Used to indicate whether the frame is alive or not (see hibernate/restore methods)
        

    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        self._motion.init(self._ui.speed_spinbox.value(), self._distance)

    def shutdown(self):
        '''
          Used to terminate the plugin
        '''
        rospy.loginfo("Kobuki TestSuite: cliff sensor shutdown")
        self._motion.shutdown()

    ##########################################################################
    # Widget Management
    ##########################################################################
        
    def hibernate(self):
        '''
          This gets called when the frame goes out of focus (tab switch). 
          Disable everything to avoid running N tabs in parallel when in
          reality we are only running one.
        '''
        pass
    
    def restore(self):
        '''
          Restore the frame after a hibernate.
        '''
        pass


    ##########################################################################
    # Motion Callbacks
    ##########################################################################

    def _run_finished(self):
        if self._state == CliffSensorFrame.STATE_STOPPED:
            return
        elif self._state == CliffSensorFrame.STATE_FORWARD:
            self._state = CliffSensorFrame.STATE_BACKWARD
            self._motion.init(-self._motion.speed, 0.2)
            self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
            self._motion_thread.start()
        else:
            self._state = CliffSensorFrame.STATE_FORWARD
            self._motion.init(-self._motion.speed, self._distance)
            self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
            self._motion_thread.start()
        
    ##########################################################################
    # Qt Callbacks
    ##########################################################################
    @Slot()
    def on_start_button_clicked(self):
        self._ui.start_button.setEnabled(False)
        self._ui.stop_button.setEnabled(True)
        self._state = CliffSensorFrame.STATE_FORWARD
        self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
        self._motion_thread.start()

    @Slot()
    def on_stop_button_clicked(self):
        self._state = CliffSensorFrame.STATE_STOPPED
        self.stop()
        
    def stop(self):
        self._motion.stop()
        if self._motion_thread:
            self._motion_thread.wait()
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        
    @pyqtSlot(float)
    def on_speed_spinbox_valueChanged(self, value):
        self._motion.init(self._ui.speed_spinbox.value(), self._distance)

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    #def robot_state_callback(self, data):
    #    if data.state == RobotStateEvent.OFFLINE:
    #        self.stop()

#
