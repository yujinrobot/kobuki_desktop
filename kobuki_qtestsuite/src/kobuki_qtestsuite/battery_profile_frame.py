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
    from python_qt_binding.QtGui import QFrame, QVBoxLayout
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QFrame, QVBoxLayout
import math

import rospy
from qt_gui_py_common.worker_thread import WorkerThread
from rqt_plot.plot_widget import PlotWidget
from kobuki_testsuite import Rotate
from kobuki_msgs.msg import RobotStateEvent

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.battery_profile_frame_ui import Ui_battery_profile_frame
from full_size_data_plot import FullSizeDataPlot
from rqt_plot.data_plot import DataPlot

##############################################################################
# Classes
##############################################################################

class BatteryProfileFrame(QFrame):
    def __init__(self, parent=None):
        super(BatteryProfileFrame, self).__init__(parent)
        self._cmd_vel_topic_name = '/mobile_base/commands/velocity'
        self._battery_topic_name = "/mobile_base/sensors/core/battery"
        self._ui = Ui_battery_profile_frame()
        self._motion = None
        self.robot_state_subscriber = None
        self._motion_thread = None
        self._plot_widget = None

    def setupUi(self, cmd_vel_topic_name):
        self._ui.setupUi(self)
        self._cmd_vel_topic_name = cmd_vel_topic_name
        self._plot_layout = QVBoxLayout(self._ui.battery_profile_group_box)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        self.restore()

    def shutdown(self):
        '''
          Used to terminate the plugin
        '''
        rospy.loginfo("Kobuki TestSuite: battery test shutdown")
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
        self._plot_layout.removeWidget(self._plot_widget)
        self._plot_widget = None

    def restore(self):
        '''
          Restore the frame after a hibernate.
        '''
        self._plot_widget = PlotWidget()
        self._plot_widget.setWindowTitle("Battery Profile")
        self._plot_widget.topic_edit.setText(self._battery_topic_name)
        self._plot_layout.addWidget(self._plot_widget)

        self._data_plot = DataPlot(self._plot_widget)
        self._data_plot.set_autoscale(y=False)
        self._data_plot.set_ylim([0, 180])
        self._plot_widget.switch_data_plot_widget(self._data_plot)


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
        if not self._motion:
            self._motion = Rotate(self._cmd_vel_topic_name)
            self._motion.init(self._ui.angular_speed_spinbox.value())
        if not self.robot_state_subscriber:
            self.robot_state_subscriber = rospy.Subscriber("/mobile_base/events/robot_state", RobotStateEvent, self.robot_state_callback)
        rospy.sleep(0.5)
        self._plot_widget._start_time = rospy.get_time()
        self._plot_widget.enable_timer(True)
        try:
            self._plot_widget.remove_topic(self._battery_topic_name)
        except KeyError:
            pass
        self._plot_widget.add_topic(self._battery_topic_name)
        self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
        self._motion_thread.start()

    @Slot()
    def on_stop_button_clicked(self):
        '''
          Hardcore stoppage - straight to zero.
        '''
        self._stop()

    def _stop(self):
        if self._plot_widget:
            self._plot_widget.enable_timer(False) # pause plot rendering
        if self._motion_thread:
            self._motion.stop()
            self._motion_thread.wait()
            self._motion_thread = None
        if self._motion:
            self._motion.shutdown()
            self._motion = None
        if self.robot_state_subscriber:
            self.robot_state_subscriber.unregister()
            self.robot_state_subscriber = None
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    @pyqtSlot(float)
    def on_angular_speed_spinbox_valueChanged(self, value):
        if self._motion:
            self._motion.init(self._ui.angular_speed_spinbox.value())

    ##########################################################################
    # External Slot Callbacks
    ##########################################################################
    @Slot(str)
    def on_cmd_vel_topic_combo_box_currentIndexChanged(self, topic_name):
        '''
          To be connected to the configuration dock combo box (via the main testsuite frame)
        '''
        self._cmd_vel_topic_name = topic_name
        print("DudetteBattery %s" % topic_name)

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def robot_state_callback(self, data):
        if data.state == RobotStateEvent.OFFLINE:
            self.stop()
