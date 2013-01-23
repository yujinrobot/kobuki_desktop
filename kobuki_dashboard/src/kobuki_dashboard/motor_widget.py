import rospy
from functools import partial

from kobuki_msgs.msg import MotorPower

from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize

class MotorWidget(IconToolButton):
    def __init__(self, topic):
        self._pub = rospy.Publisher(topic, MotorPower) 

        self._off_icon = ['bg-grey.svg', 'ic-motors.svg']
        self._on_icon = ['bg-green.svg', 'ic-motors.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-motors.svg', 'ol-stale.svg']
        self._state = 0

        icons = [self._off_icon, self._on_icon, self._stale_icon]
        super(MotorWidget, self).__init__(topic, icons=icons)
        self.setFixedSize(QSize(40,40))

        self.update_state(2)

        self.clicked.connect(self.toggle)

    def update_state(self, state):
        if state != self._state: 
            super(MotorWidget, self).update_state(state)
            if state != 2:
                self._pub.publish(MotorPower(state))

    def toggle(self):
        if self._state == 1:
            self.update_state(0)
        else:
            self.update_state(1)

    def close(self):
        self._pub.unregister()
 
