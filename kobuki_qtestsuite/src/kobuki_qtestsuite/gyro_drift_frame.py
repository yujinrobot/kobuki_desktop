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
from kobuki_testsuite import ScanToAngle, DriftEstimation
from rqt_plot.plot_widget import PlotWidget

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.gyro_drift_frame_ui import Ui_gyro_drift_frame
from full_size_data_plot import FullSizeDataPlot
from rqt_plot.mat_data_plot import MatDataPlot

##############################################################################
# Classes
##############################################################################

class GyroDriftFrame(QFrame):
    def __init__(self, parent=None):
        super(GyroDriftFrame, self).__init__(parent)
        self._ui = Ui_gyro_drift_frame()
        self._laser_scan_angle_topic_name = '/laser_scan_angle'
        self._gyro_scan_angle_topic_name = '/gyro_scan_angle'
        self._error_scan_angle_topic_name = '/error_scan_angle'
        self._cmd_vel_topic_name = '/cmd_vel'
        self._motion = DriftEstimation(self._laser_scan_angle_topic_name, self._gyro_scan_angle_topic_name, self._error_scan_angle_topic_name, '/cmd_vel','/mobile_base/sensors/imu_data')
        self._scan_to_angle = ScanToAngle('/scan',self._laser_scan_angle_topic_name)
        self._motion_thread = None

    def setupUi(self):
        self._ui.setupUi(self)
        self._plot_layout = QVBoxLayout(self._ui.scan_angle_group_box)
        self._plot_widget = PlotWidget()
        self._plot_widget.setWindowTitle("Error")
        self._plot_layout.addWidget(self._plot_widget)
        self._plot_widget.switch_data_plot_widget(FullSizeDataPlot(self._plot_widget))
        self._plot_widget.data_plot.dynamic_range = True
        self._plot_widget_live = PlotWidget()
        self._plot_widget_live.setWindowTitle("Live Graphs")
        self._plot_layout.addWidget(self._plot_widget_live)
        self._plot_widget_live.switch_data_plot_widget(MatDataPlot(self._plot_widget_live))
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        self._motion.init(self._ui.angular_speed_spinbox.value())

    def shutdown(self):
        rospy.loginfo("Kobuki TestSuite: gyro drift shutdown")
        self._motion.shutdown()
        self._scan_to_angle.shutdown()

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
        self._scan_to_angle.start()
        self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
        self._motion_thread.start()
        rospy.sleep(0.1)
        self._plot_widget.data_plot.reset()
        self._plot_widget._start_time = rospy.get_time()
        self._plot_widget.enable_timer(True)
        try:
            self._plot_widget.remove_topic(self._error_scan_angle_topic_name+'/scan_angle')
            self._plot_widget_live.remove_topic(self._laser_scan_angle_topic_name+'/scan_angle')
            self._plot_widget_live.remove_topic(self._gyro_scan_angle_topic_name+'/scan_angle')
            self._plot_widget_live.remove_topic(self._cmd_vel_topic_name+'/angular/z')
        except KeyError:
            pass
        self._plot_widget_live.add_topic(self._cmd_vel_topic_name+'/angular/z')
        self._plot_widget_live.add_topic(self._laser_scan_angle_topic_name+'/scan_angle')
        self._plot_widget_live.add_topic(self._gyro_scan_angle_topic_name+'/scan_angle')
        self._plot_widget.add_topic(self._error_scan_angle_topic_name+'/scan_angle')
        self._ui.start_button.setEnabled(False)
        self._ui.stop_button.setEnabled(True)

    @Slot()
    def on_stop_button_clicked(self):
        self.stop()
        
    def stop(self):
        self._motion.stop()
        self._scan_to_angle.stop()
        if self._motion_thread:
            self._motion_thread.wait()
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        
    @pyqtSlot(float)
    def on_angular_speed_spinbox_valueChanged(self, value):
        self._motion.init(self._ui.angular_speed_spinbox.value())

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    #def robot_state_callback(self, data):
    #    if data.state == RobotStateEvent.OFFLINE:
    #        self.stop()

#