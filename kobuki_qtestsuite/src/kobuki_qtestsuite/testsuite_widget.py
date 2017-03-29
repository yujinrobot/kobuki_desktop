#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki_desktop/master/kobuki_qtestsuite/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy

from python_qt_binding.QtCore import Signal,Slot
try:  # indigo
    from python_qt_binding.QtGui import QWidget
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QWidget
#from rqt_py_common.extended_combo_box import ExtendedComboBox

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Local resource imports
import detail.common_rc
from detail.testsuite_ui import Ui_testsuite_widget

class TestSuiteWidget(QWidget):

    def __init__(self, parent=None):
        super(TestSuiteWidget, self).__init__(parent)
        self._ui = Ui_testsuite_widget()
        self._tabs = []

    def setupUi(self):
        self._ui.setupUi(self)
        self._tabs = [self._ui.battery_profile_frame,
                      self._ui.gyro_drift_frame,
                      self._ui.payload_frame,
                      self._ui.cliff_sensor_frame,
                      self._ui.life_frame,
                      self._ui.wandering_frame
                       ]
        self._current_tab = self._tabs[self._ui.testsuite_tab_widget.currentIndex()]
        self._ui.configuration_dock.setupUi()
        self._ui.battery_profile_frame.setupUi(self._ui.configuration_dock.cmd_vel_topic_name())
        self._ui.gyro_drift_frame.setupUi()
        self._ui.payload_frame.setupUi()
        self._ui.cliff_sensor_frame.setupUi()
        self._ui.life_frame.setupUi()
        self._ui.wandering_frame.setupUi()
        #self.cmd_vel_publisher = rospy.Publisher(self._ui.configuration_dock.cmd_vel_topic_name(), Twist)
        #self.odom_subscriber = rospy.Subscriber(self._ui.configuration_dock.odom_topic_name(), Odometry, self.odometry_callback)
        ####################
        # Slot Callbacks
        ####################
        self._ui.configuration_dock._ui.cmd_vel_topic_combo_box.currentIndexChanged[str].connect(
                    self._ui.battery_profile_frame.on_cmd_vel_topic_combo_box_currentIndexChanged)

    def shutdown(self):
        self._ui.battery_profile_frame.shutdown()
        self._ui.gyro_drift_frame.shutdown()
        self._ui.payload_frame.shutdown()
        self._ui.cliff_sensor_frame.shutdown()
        self._ui.life_frame.shutdown()
        self._ui.wandering_frame.shutdown()

    ##########################################################################
    # Slot Callbacks
    ##########################################################################
    @Slot(str)
    def on_cmd_vel_topic_combo_box_currentIndexChanged(self, topic_name):
        # This is probably a bit broken, need to test with more than just /cmd_vel so
        # there is more than one option.
        print("Dude")
        self.cmd_vel_publisher = rospy.Publisher(str(self.cmd_vel_topic_combo_box.currentText()), Twist, queue_size=10)

    @Slot(str)
    def on_odom_topic_combo_box_currentIndexChanged(self, topic_name):
        # Need to redo the subscriber here
        pass

    @Slot(int)
    def on_testsuite_tab_widget_currentChanged(self, index):
        self._current_tab.hibernate()
        self._current_tab = self._tabs[self._ui.testsuite_tab_widget.currentIndex()]
        self._current_tab.restore()

