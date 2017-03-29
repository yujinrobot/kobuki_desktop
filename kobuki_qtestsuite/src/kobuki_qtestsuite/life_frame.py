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
#import python_qt_binding.QtCore
from python_qt_binding.QtCore import QObject, Signal, Slot, pyqtSlot, QTimer
try:  # indigo
    from python_qt_binding.QtGui import QFrame, QVBoxLayout
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QFrame, QVBoxLayout
import math

import rospy
from qt_gui_py_common.worker_thread import WorkerThread
from kobuki_testsuite import Rotate

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.life_frame_ui import Ui_life_frame

##############################################################################
# Classes
##############################################################################

class LifeFrame(QFrame):
    STATE_STOPPED = "stopped"
    STATE_RUN = "running"
    STATE_IDLE = "idle"

    def __init__(self, parent=None):
        super(LifeFrame, self).__init__(parent)
        self._ui = Ui_life_frame()
        self._motion = Rotate('/mobile_base/commands/velocity')
        self._motion_thread = None
        self._timer = QTimer()
        #self._timer.setInterval(60000) #60s
        self._timer.setInterval(250) #60s
        self._timer.timeout.connect(self.update_progress_callback)
        self._state = LifeFrame.STATE_STOPPED
        self._is_alive = False # Used to indicate whether the frame is alive or not (see hibernate/restore methods)

    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        self._motion.init(self._ui.angular_speed_spinbox.value())

    def shutdown(self):
        '''
          Used to terminate the plugin
        '''
        rospy.loginfo("Kobuki TestSuite: life frame shutdown")
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

    def start(self):
        self._state = LifeFrame.STATE_RUN
        self._ui.run_progress.reset()
        self._ui.idle_progress.reset()
        self._motion_thread = WorkerThread(self._motion.execute, None)
        self._motion_thread.start()

    def stop(self):
        self._state = LifeFrame.STATE_STOPPED
        self._motion.stop()
        if self._motion_thread:
            self._motion_thread.wait()

    ##########################################################################
    # Qt Callbacks
    ##########################################################################
    @Slot()
    def on_start_button_clicked(self):
        self._ui.start_button.setEnabled(False)
        self._ui.stop_button.setEnabled(True)
        self._timer.start()
        self.start()

    @Slot()
    def on_stop_button_clicked(self):
        self.stop()
        self._timer.stop()
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    @pyqtSlot(float)
    def on_angular_speed_spinbox_valueChanged(self, value):
        self._motion.init(self._ui.angular_speed_spinbox.value())

    ##########################################################################
    # Timer Callbacks
    ##########################################################################

    @Slot()
    def update_progress_callback(self):
        if self._state == LifeFrame.STATE_RUN:
            new_value = self._ui.run_progress.value()+1
            if new_value == self._ui.run_progress.maximum():
                print("  Switching to idle")
                self._motion.stop()
                self._state = LifeFrame.STATE_IDLE
            else:
                self._ui.run_progress.setValue(new_value)
        if self._state == LifeFrame.STATE_IDLE:
            new_value = self._ui.idle_progress.value()+1
            if new_value == self._ui.idle_progress.maximum():
                print("  Switching to run")
                self.start()
            else:
                self._ui.idle_progress.setValue(new_value)


#
