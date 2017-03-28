#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki_desktop/master/kobuki_qtestsuite/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import os
import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
try:  # indigo
    from python_qt_binding.QtGui import QFrame, QWidget
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QFrame, QWidget
from python_qt_binding.QtCore import Signal,Slot
from rqt_py_common.extended_combo_box import ExtendedComboBox
from geometry_msgs.msg import Twist

# Local resource imports
import detail.common_rc
from .testsuite_widget import TestSuiteWidget
from .configuration_dock_widget import ConfigurationDockWidget

class KobukiTestSuite(Plugin):

    def __init__(self, context):
        super(KobukiTestSuite, self).__init__(context)
        # give QObjects reasonable names
        self.setObjectName('Kobuki Test Suite')

        # create QWidget
        self._widget = TestSuiteWidget()
        #self._widget = ConfigurationDockWidget()

        # add widget to the user interface
        context.add_widget(self._widget)
        
        # Custom setup function to make sure promoted widgets all do their setup properly
        self._widget.setupUi()

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    ##########################################################################
    # Slot Callbacks
    ##########################################################################

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a dialog to offer the user a set of configuration
