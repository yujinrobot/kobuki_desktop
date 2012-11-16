/**
 * @file /include/kobuki_factory_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_FACTORY_TEST_QNODE_HPP_
#define KOBUKI_FACTORY_TEST_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <fstream>

#include <ros/ros.h>

#include <QEvent>
#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/Imu.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/VersionInfo.h>
#include <kobuki_msgs/DockInfraRed.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/PowerSystemEvent.h>
#include <kobuki_msgs/DigitalInputEvent.h>
#include <kobuki_msgs/RobotStateEvent.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include "../include/kobuki_factory_test/robots.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_factory_test {

/*****************************************************************************
** Class
*****************************************************************************/

class QNodeRequest : public QEvent {
public:
QNodeRequest(const std::string& title = "", const std::string& text = "")
    : QEvent(QEvent::User), title(title.c_str()), text(text.c_str()) { };

QString title;
QString text;
};


class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  void run();

  enum EvalStep {
    INITIALIZATION,
    GET_SERIAL_NUMBER,
    TEST_DC_ADAPTER,
    TEST_DOCKING_BASE,
    BUTTON_0_PRESSED,
    BUTTON_0_RELEASED,
    BUTTON_1_PRESSED,
    BUTTON_1_RELEASED,
    BUTTON_2_PRESSED,
    BUTTON_2_RELEASED,
    MEASURE_CHARGE_AND_PSD,  // 10
    MEASURE_GYRO_ERROR,
    TEST_DIGITAL_IO_PORTS,
    TEST_ANALOG_INPUT_PORTS,
    TEST_LEDS,
    TEST_SOUNDS,
    TEST_CLIFF_SENSORS,
    TEST_WHEEL_DROP_SENSORS,
    CENTER_BUMPER_PRESSED,
    CENTER_BUMPER_RELEASED,
    POINT_RIGHT_BUMPER,      // 20
    RIGHT_BUMPER_PRESSED,
    RIGHT_BUMPER_RELEASED,
    POINT_LEFT_BUMPER,
    LEFT_BUMPER_PRESSED,
    LEFT_BUMPER_RELEASED,
    PREPARE_MOTORS_TEST,
    TEST_MOTORS_FORWARD,
    TEST_MOTORS_BACKWARD,
    TEST_MOTORS_CLOCKWISE,
    TEST_MOTORS_COUNTERCW,   // 30
    EVAL_MOTORS_CURRENT,
    EVALUATION_COMPLETED,

    EVALUATION_STEPS_COUNT
  };

  /*********************
  ** Logging
  **********************/
  enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
   };

  QStringListModel* loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &format, ...);

Q_SIGNALS:
  void requestMW(QNodeRequest* request);
  void loggingUpdated();
  void addLogLine(const QString& str);
  void rosShutdown();

private:
  int      init_argc;
  char**   init_argv;

  double   frequency;

  ros::Timer timer;
  bool     timer_active;
  EvalStep current_step;

  Robot *   under_test;
  RobotList  evaluated;

  bool      answer_req;      // user input required
  std::string out_file;      // csv output file path

  /*********************
  ** Testing parameters
  **********************/
  int    motor_max_current;
  int    cliff_sensor_tests;
  int    wheel_drop_tests;
  int    power_plug_tests;
  int    min_power_charged;     // tenths of volt
  double measure_charge_time;   // seconds
  double gyro_camera_max_diff;  // radians
  int    ainput_min_threshold;  // analog input minimum threshold, in millivolts
  int    ainput_max_threshold;  // analog input maximum threshold, in millivolts

  /*********************
  ** Publishers
  **********************/
  ros::Publisher cmd_vel_pub;
  ros::Publisher led_1_pub;
  ros::Publisher led_2_pub;
  ros::Publisher sound_pub;
  ros::Publisher output_pub;
  ros::Publisher ext_pwr_pub;

  /*********************
  ** Subscribers
  **********************/
  ros::Subscriber v_info_sub; // version info
  ros::Subscriber s_core_sub; // sensors core
  ros::Subscriber beacon_sub; // dock ir sensor
  ros::Subscriber gyro_sub;   // gyroscope data
  ros::Subscriber button_sub; // buttons events
  ros::Subscriber bumper_sub; // bumpers events
  ros::Subscriber w_drop_sub; // wheels drop events
  ros::Subscriber cliff_sub;  // cliff sensors events
  ros::Subscriber power_sub;  // power system events
  ros::Subscriber input_sub;  // digital input events
  ros::Subscriber robot_sub;  // robot state events
  ros::Subscriber state_sub;  // diagnostics top level state
  ros::Subscriber diags_sub;  // diagnostics

  QStringListModel logging_model;


  /*********************
  ** Callbacks
  **********************/
  void versionInfoCB(const kobuki_msgs::VersionInfo::ConstPtr& msg);
  void sensorsCoreCB(const kobuki_msgs::SensorState::ConstPtr& msg);
  void dockBeaconCB(const kobuki_msgs::DockInfraRed::ConstPtr& msg);
  void gyroscopeCB(const sensor_msgs::Imu::ConstPtr& msg);
  void buttonEventCB(const kobuki_msgs::ButtonEvent::ConstPtr& msg);
  void bumperEventCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);
  void wDropEventCB(const kobuki_msgs::WheelDropEvent::ConstPtr& msg);
  void cliffEventCB(const kobuki_msgs::CliffEvent::ConstPtr& msg);
  void powerEventCB(const kobuki_msgs::PowerSystemEvent::ConstPtr& msg);
  void inputEventCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg);
  void robotEventCB(const kobuki_msgs::RobotStateEvent::ConstPtr& msg);
  void timerEventCB(const ros::TimerEvent& event);
  void robotStatusCB(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
  void diagnosticsCB(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

  /*********************
  ** Other methods
  **********************/
  bool testIMU(bool first_call);
  void testLeds(bool first_call);
  void testSounds(bool first_call);
  bool testAnalogIn(bool first_call);
  bool measureCharge(bool first_call);
  void evalMotorsCurrent(bool first_call);
  void move(double v, double w, double t = 0.0, bool blocking = false);
  bool saveResults();

  void showUserMsg(LogLevel level, const std::string& title, const std::string& format, ...) {
    va_list arguments;
    va_start(arguments, format);

    char text[256];
    vsnprintf(text, 256, format.c_str(), arguments);

    Q_EMIT requestMW(new QNodeRequest(title, text));
    log(level, "%s: %s", title.c_str(), text);
  }

  void hideUserMsg() {
    Q_EMIT requestMW(new QNodeRequest());
  }

  void nbSleep(double t) {
    // Non-blocking (but naively imprecise) sleep
    for (int i = 0; i < int(t*frequency) && ros::ok(); i++) {
      ros::Duration(1.0/frequency).sleep();
      ros::spinOnce();
    }
    ros::Duration(t*frequency - int(t*frequency)).sleep();
  }
};

}  // namespace kobuki_factory_test

#endif /* KOBUKI_FACTORY_TEST_QNODE_HPP_ */
