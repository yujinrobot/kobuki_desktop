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
#from rqt_plot.qwt_data_plot import QwtDataPlot
from rqt_plot.mat_data_plot import MatDataPlot
from qt_gui_py_common.worker_thread import WorkerThread
from rqt_plot.plot_widget import PlotWidget
#from kobuki_testsuite import SafeWandering

# Local resource imports
import detail.common_rc
import detail.text_rc
from detail.battery_profile_frame_ui import Ui_battery_profile_frame

##############################################################################
# Classes
##############################################################################

class BatteryProfileDataPlot(MatDataPlot):
    def __init__(self, parent=None):
        super(BatteryProfileDataPlot, self).__init__(parent)
        
    ######################################
    # Overrides
    ######################################
    def _update_legend(self):
        handles, labels = self._canvas.axes.get_legend_handles_labels()
        if handles:
            hl = sorted(zip(handles, labels), key=operator.itemgetter(1))
            handles, labels = zip(*hl)
        self._canvas.axes.legend(handles, labels, loc='lower right')
        
    def redraw(self):
        ''' 
          We fix the y axis and continually resize the x axis to encapsulate
          the entire domain, range of the battery profile.
          
          @Todo : the domain is simply the data value, we could use
        '''
        self._canvas.axes.grid(True, color='gray')
        # Set axis bounds
        ymin = ymax = None
        xmax = 0
        for curve in self._curves.values():
            data_x, data_y, plot = curve
            if len(data_x) == 0:
                continue

            xmax = max(xmax, data_x[-1])
            self._canvas.axes.set_xbound(lower=0, upper=xmax)

            self._canvas.axes.set_ybound(lower=0, upper=180)

        # Set plot data on current axes
        for curve in self._curves.values():
            data_x, data_y, plot = curve
            plot.set_data(numpy.array(data_x), numpy.array(data_y))

        self._canvas.draw()

class BatteryProfileFrame(QFrame):
    def __init__(self, parent=None):
        super(BatteryProfileFrame, self).__init__(parent)
        self._battery_topic_name = "/mobile_base/sensors/core/battery"
        self._ui = Ui_battery_profile_frame()
        #self._motion = SafeWandering('/cmd_vel','/odom', '/mobile_base/events/bumper', '/mobile_base/events/cliff')
        #self._motion_thread = None

    def setupUi(self):
        self._ui.setupUi(self)
        self._plot_layout = QVBoxLayout(self._ui.battery_profile_group_box)
        self._plot_widget = PlotWidget()
        self._plot_widget.setWindowTitle("Battery Profile")
        self._plot_widget.topic_edit.setText(self._battery_topic_name)
        self._plot_layout.addWidget(self._plot_widget)
        self._plot_widget.switch_data_plot_widget(BatteryProfileDataPlot(self._plot_widget))
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)
        #self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())

    def shutdown(self):
        rospy.loginfo("Kobuki TestSuite: wandering test shutdown")
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
        self._plot_widget._start_time = rospy.get_time()
        self._plot_widget.add_topic(self._battery_topic_name)
        self._ui.start_button.setEnabled(False)
        self._ui.stop_button.setEnabled(True)
        #self._motion_thread = WorkerThread(self._motion.execute, self._run_finished)
        #self._motion_thread.start()

    @Slot()
    def on_stop_button_clicked(self):
        '''
          Hardcore stoppage - straight to zero.
        '''
        #self._motion.stop()
        self._plot_widget.remove_topic(self._battery_topic_name)
        self._ui.start_button.setEnabled(True)
        self._ui.stop_button.setEnabled(False)

    @pyqtSlot(float)
    def on_angular_speed_spinbox_valueChanged(self, value):
        pass
        # could use value, but easy to set like this
        #self._motion.init(self._ui.speed_spinbox.value(), -0.1, self._ui.angular_speed_spinbox.value())
