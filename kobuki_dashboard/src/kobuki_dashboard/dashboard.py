import roslib;roslib.load_manifest('kobuki_dashboard')
import rospy

import diagnostic_msgs

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, MenuDashWidget, IconToolButton
from QtGui import QMessageBox, QAction
from python_qt_binding.QtCore import QSize

from .battery_widget import BatteryWidget
from .led_widget import LedWidget
from .motor_widget import MotorWidget

class KobukiDashboard(Dashboard):
    def setup(self, context):
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

        self._motor_widget = MotorWidget('/mobile_base/commands/motor_power')
        self._laptop_bat = BatteryWidget("Laptop")
        self._kobuki_bat = BatteryWidget("Kobuki")

        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)

    def get_widgets(self):
        leds = [LedWidget('/mobile_base/commands/led1'), LedWidget('/mobile_base/commands/led2')]
        return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context), self._motor_widget], leds, [self._laptop_bat, self._kobuki_bat]]

    def dashboard_callback(self, msg):
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()
        
        laptop_battery_status = {}
        for status in msg.status:
            if status.name == "/Kobuki/Motor State":
                motor_state = int(status.values[0].value)
                self._motor_widget.update_state(motor_state)

            elif status.name == "/Power System/Battery":
                for value in status.values:
                    if value.key == 'Percent':
                        self._kobuki_bat.update_perc(float(value.value))
                        # This should be self._last_dashboard_message_time?
                        # Is it even used graphically by the widget
                        self._kobuki_bat.update_time(float(value.value))
                    elif value.key == "Charging State":
                        if value.value == "Trickle Charging" or value.value == "Full Charging":
                            self._kobuki_bat.set_charging(True)
                        else:
                            self._kobuki_bat.set_charging(False)
            elif status.name == "/Power System/Laptop Battery":
                for value in status.values:
                    laptop_battery_status[value.key]=value.value

        if (laptop_battery_status):
            percentage = float(laptop_battery_status['Charge (Ah)'])/float(laptop_battery_status['Capacity (Ah)'])
            self._laptop_bat.update_perc(percentage*100)
            self._laptop_bat.update_time(percentage*100)
            charging_state = True if float(laptop_battery_status['Current (A)']) > 0.0 else False
            self._laptop_bat.set_charging(charging_state)

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()
