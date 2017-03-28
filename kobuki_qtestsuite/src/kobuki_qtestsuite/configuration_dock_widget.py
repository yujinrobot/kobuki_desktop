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
    from python_qt_binding.QtGui import QDockWidget
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QDockWidget
from rqt_py_common.extended_combo_box import ExtendedComboBox

# Local resource imports
import detail.common_rc
from detail.configuration_dock_ui import Ui_configuration_dock_widget

class ConfigurationDockWidget(QDockWidget):

    def __init__(self, parent=None):
        super(ConfigurationDockWidget, self).__init__(parent)
        
        self._ui = Ui_configuration_dock_widget()
        #self._ui.setupUi(self)

        
    def setupUi(self):
        self._ui.setupUi(self)
        _, _, topic_types = rospy.get_master().getTopicTypes()
        cmd_vel_topics = [ topic[0] for topic in topic_types if topic[1] == 'geometry_msgs/Twist' ]
        self._ui.cmd_vel_topic_combo_box.setItems.emit(sorted(cmd_vel_topics))
        #self.cmd_vel_publisher = rospy.Publisher(str(self.cmd_vel_topic_combo_box.currentText()), Twist)
        odom_topics = [ topic[0] for topic in topic_types if topic[1] == 'nav_msgs/Odometry' ]
        self._ui.odom_topic_combo_box.setItems.emit(sorted(odom_topics))
        #self.odom_subscriber = rospy.Subscriber(str(self.odom_topic_combo_box.currentText()), Odometry, self.odometry_callback)
        core_sensor_topics = [ topic[0] for topic in topic_types if topic[1] == 'kobuki_msgs/SensorState' ]
        self._ui.core_topic_combo_box.setItems.emit(sorted(core_sensor_topics))
        
    def cmd_vel_topic_name(self):
        return str(self._ui.cmd_vel_topic_combo_box.currentText())

    def odom_topic_name(self):
        return str(self._ui.odom_topic_combo_box.currentText())

    def core_sensors_topic_name(self):
        return str(self._ui.core_sensor_topic_combo_box.currentText())

    def battery_topic_name(self):
        return str(self._ui.core_sensor_topic_combo_box.currentText() + '/' + battery)
        
    ##########################################################################
    # Slot Callbacks
    ##########################################################################
    @Slot(str)
    def on_cmd_vel_topic_combo_box_currentIndexChanged(self, topic_name):
        pass
        # This is probably a bit broken, need to test with more than just /cmd_vel so
        # there is more than one option.
        #self.cmd_vel_publisher = rospy.Publisher(str(self.cmd_vel_topic_combo_box.currentText()), Twist)

    @Slot(str)
    def on_odom_topic_combo_box_currentIndexChanged(self, topic_name):
        # Need to redo the subscriber here
        pass

    @Slot(str)
    def on_core_topic_combo_box_currentIndexChanged(self, topic_name):
        pass
