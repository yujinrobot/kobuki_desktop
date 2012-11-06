import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy

from python_qt_binding.QtCore import Signal,Slot
from python_qt_binding.QtGui import QWidget
from rqt_py_common.extended_combo_box import ExtendedComboBox

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Local resource imports
import detail.common_rc
import detail.climbing_rc
from detail.testsuite_ui import Ui_testsuite_widget
from configuration_dock_widget import ConfigurationDockWidget
from climbing_frame import ClimbingFrame
from wandering_frame import WanderingFrame

class TestSuiteWidget(QWidget):

    def __init__(self, parent=None):
        super(TestSuiteWidget, self).__init__(parent)
        
        self._ui = Ui_testsuite_widget()

        self._run_thread = None
        self._current_pose = None
        self._starting_pose = None
        self._current_speed = 0.0
        
    def setupUi(self):
        self._ui.setupUi(self)
        self._ui.configuration_dock.setupUi()
        self._ui.climbing_frame.setupUi()
        self._ui.wandering_frame.setupUi()
        #self.cmd_vel_publisher = rospy.Publisher(self._ui.configuration_dock.cmd_vel_topic_name(), Twist)
        #self.odom_subscriber = rospy.Subscriber(self._ui.configuration_dock.odom_topic_name(), Odometry, self.odometry_callback)
    
    def shutdown(self):
        self._ui.climbing_frame.shutdown()
        self._ui.wandering_frame.shutdown()
        
    ##########################################################################
    # Slot Callbacks
    ##########################################################################
    @Slot(str)
    def on_cmd_vel_topic_combo_box_currentIndexChanged(self, topic_name):
        # This is probably a bit broken, need to test with more than just /cmd_vel so
        # there is more than one option.
        self.cmd_vel_publisher = rospy.Publisher(str(self.cmd_vel_topic_combo_box.currentText()), Twist)

    @Slot(str)
    def on_odom_topic_combo_box_currentIndexChanged(self, topic_name):
        # Need to redo the subscriber here
        pass


    def _soft_stop(self):
        rate = rospy.Rate(10)
        while self._current_speed > 0.0:
            self._current_speed -= 0.01
            cmd = Twist()
            cmd.linear.x = self._current_speed
            self.cmd_vel_publisher.publish(cmd)
            rate.sleep()
        self._current_speed = 0.00
        cmd = Twist()
        cmd.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd)
        
    