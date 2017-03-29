#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki_desktop/master/kobuki_qtestsuite/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
from python_qt_binding.QtCore import Signal, Slot, pyqtSlot
try:  # indigo
    from python_qt_binding.QtGui import QFrame
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QFrame
import math

import rospy
from kobuki_testsuite import SafeWandering
from kobuki_msgs.msg import BumperEvent

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.wandering_frame_ui import Ui_wandering_frame
from qt_gui_py_common.worker_thread import WorkerThread

##############################################################################
# Classes
##############################################################################

class WanderingFrame(QFrame):
    def __init__(self, parent=None):
        super(WanderingFrame, self).__init__(parent)
        self._ui = Ui_wandering_frame()
        self.bump_subscriber = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_event_callback)
        self._motion = SafeWandering('/mobile_base/commands/velocity','/odom', '/mobile_base/events/bumper', '/mobile_base/events/cliff')
        self._motion_thread = None
        self._is_alive = False # Used to indicate whether the frame is alive or not (see hibernate/restore methods)

    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())

    def shutdown(self):
        '''
          Used to terminate the plugin
        '''
        rospy.loginfo("Kobuki TestSuite: wandering test shutdown")
        self._motion.shutdown()
        self.bump_subscriber.unregister()

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


    def stop(self):
        self._motion.stop()
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

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
        self.stop()

    @pyqtSlot(float)
    def on_speed_spinbox_valueChanged(self, value):
        # could use value, but easy to set like this
        self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())

    @pyqtSlot(float)
    def on_angular_speed_spinbox_valueChanged(self, value):
        # could use value, but easy to set like this
        self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())

    ##########################################################################
    # Ros Callbacks
    ##########################################################################
    def bumper_event_callback(self, msg):
        if msg.state == BumperEvent.PRESSED:
            if msg.bumper == BumperEvent.LEFT:
                self._ui.left_bump_counter_lcd.display(self._ui.left_bump_counter_lcd.intValue()+1)
            elif msg.bumper == BumperEvent.RIGHT:
                self._ui.right_bump_counter_lcd.display(self._ui.right_bump_counter_lcd.intValue()+1)
            else:
                self._ui.centre_bump_counter_lcd.display(self._ui.centre_bump_counter_lcd.intValue()+1)

