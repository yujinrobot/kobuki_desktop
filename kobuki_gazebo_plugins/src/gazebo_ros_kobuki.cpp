/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Marcus Liebhardt
 *
 * This work has been inspired by Nate Koenig's Gazebo plugin for the iRobot Create.
 */

#include <cmath>
#include <cstring>
#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <math/gzmath.hh>
#include "kobuki_gazebo_plugins/gazebo_ros_kobuki.h"

namespace gazebo
{

enum {LEFT= 0, RIGHT=1};

GazeboRosKobuki::GazeboRosKobuki() : shutdown_requested_(false)
{
  // Connect with ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_kobuki", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;

  // using the same values as in kobuki_node
  double pose_cov[36] = {0.1, 0, 0, 0, 0, 0,
                          0, 0.1, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 0.2};
  memcpy(&pose_cov, &pose_cov_, sizeof(double[36]));
}

GazeboRosKobuki::~GazeboRosKobuki()
{
//  rosnode_->shutdown();
  shutdown_requested_ = true;
  // Wait for spinner thread to end
//  ros_spinner_thread_->join();

  //  delete spinner_thread_;
//  delete rosnode_;
}

void GazeboRosKobuki::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  model_ = parent;
  if (!model_)
  {
    ROS_ERROR_STREAM("Invalid model pointer! [" << node_name_ << "]");
    return;
  }
  // Get then name of the parent model and use it as node name
  std::string model_name = sdf->GetParent()->GetValueString("name");
  gzdbg << "Plugin model name: " << model_name << "\n";
  nh_ = ros::NodeHandle("");
  // creating a private name pace until Gazebo implements topic remappings
  nh_priv_ = ros::NodeHandle("/" + model_name);
  node_name_ = model_name;

  world_ = parent->GetWorld();

// TODO: use when implementing subs
//  ros_spinner_thread_ = new boost::thread(boost::bind(&GazeboRosKobuki::spin, this));
//
//  this->node_namespace_ = "";
//  if (_sdf->HasElement("node_namespace"))
//    this->node_namespace_ = _sdf->GetElement("node_namespace")->GetValueString() + "/";

  /*
   * Prepare receiving motor power commands
   */
  motor_power_sub_ = nh_priv_.subscribe("commands/motor_power", 10, &GazeboRosKobuki::motorPowerCB, this);
  motors_enabled_ = true;

  /*
   * Prepare joint state publishing
   */
  if (sdf->HasElement("left_wheel_joint_name"))
  {
    left_wheel_joint_name_ = sdf->GetElement("left_wheel_joint_name")->GetValueString();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find left wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("right_wheel_joint_name"))
  {
    right_wheel_joint_name_ = sdf->GetElement("right_wheel_joint_name")->GetValueString();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find right wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return;
  }
  joints_[LEFT] = parent->GetJoint(left_wheel_joint_name_);
  joints_[RIGHT] = parent->GetJoint(right_wheel_joint_name_);
  if (!joints_[LEFT] || !joints_[RIGHT])
  {
    ROS_ERROR_STREAM("Couldn't find specified wheel joints in the model! [" << node_name_ <<"]");
    return;
  }
  joint_state_.header.frame_id = "Joint States";
  joint_state_.name.push_back(left_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);
  joint_state_.name.push_back(right_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  /*
   * Prepare publishing odometry data
   */
  if (sdf->HasElement("wheel_separation"))
  {
    wheel_sep_ = sdf->GetElement("wheel_separation")->GetValueDouble();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("wheel_diameter"))
  {
    wheel_diam_ = sdf->GetElement("wheel_diameter")->GetValueDouble();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel diameter parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("torque"))
  {
    torque_ = sdf->GetElement("torque")->GetValueDouble();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the torque parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

  /*
   * Prepare receiving velocity commands
   */
  if (sdf->HasElement("velocity_command_timeout"))
  {
    cmd_vel_timeout_ = sdf->GetElement("velocity_command_timeout")->GetValueDouble();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  last_cmd_vel_time_ = world_->GetSimTime();
  cmd_vel_sub_ = nh_priv_.subscribe("commands/velocity", 100, &GazeboRosKobuki::cmdVelCB, this);

  /*
   * Prepare cliff sensors
   */
  std::string cliff_sensor_left_name, cliff_sensor_front_name, cliff_sensor_right_name;
  if (sdf->HasElement("cliff_sensor_left_name"))
  {
    cliff_sensor_left_name = sdf->GetElement("cliff_sensor_left_name")->GetValueString();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of left cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("cliff_sensor_front_name"))
  {
    cliff_sensor_front_name = sdf->GetElement("cliff_sensor_front_name")->GetValueString();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of frontal cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("cliff_sensor_right_name"))
  {
    cliff_sensor_right_name = sdf->GetElement("cliff_sensor_right_name")->GetValueString();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of right cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  cliff_sensor_left_ = boost::shared_dynamic_cast<sensors::RaySensor>(
                       sensors::SensorManager::Instance()->GetSensor(cliff_sensor_left_name));
  cliff_sensor_front_ = boost::shared_dynamic_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_front_name));
  cliff_sensor_right_ = boost::shared_dynamic_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_right_name));
  if (!cliff_sensor_left_)
  {
    ROS_ERROR_STREAM("Couldn't find the left cliff sensor in the model! [" << node_name_ <<"]");
    return;
  }
  if (!cliff_sensor_front_)
  {
    ROS_ERROR_STREAM("Couldn't find the frontal cliff sensor in the model! [" << node_name_ <<"]");
    return;
  }
  if (!cliff_sensor_right_)
  {
    ROS_ERROR_STREAM("Couldn't find the right cliff sensor in the model! [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("cliff_detection_threshold"))
  {
    cliff_detection_threshold_ = sdf->GetElement("cliff_detection_threshold")->GetValueDouble();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the cliff detection threshold parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  cliff_sensor_left_->SetActive(true);
  cliff_sensor_front_->SetActive(true);
  cliff_sensor_right_->SetActive(true);
  cliff_event_pub_ = nh_priv_.advertise<kobuki_msgs::CliffEvent>("events/cliff", 1);

  /*
   * Prepare bumper
   */
  std::string bumper_name;
  if (sdf->HasElement("bumper_name"))
  {
    bumper_name = sdf->GetElement("bumper_name")->GetValueString();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of bumper sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  bumper_ = boost::shared_dynamic_cast<sensors::ContactSensor>(
            sensors::SensorManager::Instance()->GetSensor(bumper_name));
  if (!bumper_)
  {
    ROS_ERROR_STREAM("Couldn't find the bumpers in the model! [" << node_name_ <<"]");
    return;
  }
  bumper_->SetActive(true);
  bumper_event_pub_ = nh_priv_.advertise<kobuki_msgs::BumperEvent>("events/bumper", 1);

  prev_update_time_ = world_->GetSimTime();
  ROS_INFO_STREAM("GazeboRosKobuki plugin ready to go! [" << node_name_ << "]");
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosKobuki::OnUpdate, this));
}

void GazeboRosKobuki::motorPowerCB( const kobuki_msgs::MotorPowerPtr &msg)
{
  if ((msg->state == kobuki_msgs::MotorPower::ON) && (!motors_enabled_))
  {
    motors_enabled_ = true;
    ROS_INFO_STREAM("Motors fired up. [" << node_name_ << "]");
  }
  else if ((msg->state == kobuki_msgs::MotorPower::OFF) && (motors_enabled_))
  {
    motors_enabled_ = false;
    ROS_INFO_STREAM("Motors taking a rest. [" << node_name_ << "]");
  }
}

void GazeboRosKobuki::cmdVelCB( const geometry_msgs::TwistConstPtr &msg)
{
  last_cmd_vel_time_ = world_->GetSimTime();
  wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
  wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;
}

void GazeboRosKobuki::OnUpdate()
{
  /*
   * First process ROS callbacks
   */
  ros::spinOnce();

  /*
   * Update current time and time step
   */
  common::Time time_now = world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  /*
   * Joint states
   */
  joint_state_.header.stamp.sec = time_now.sec;
  joint_state_.header.stamp.nsec = time_now.nsec;
  joint_state_.position[LEFT] = joints_[LEFT]->GetAngle(0).Radian();
  joint_state_.velocity[LEFT] = joints_[LEFT]->GetVelocity(0);
  joint_state_.position[RIGHT] = joints_[RIGHT]->GetAngle(0).Radian();
  joint_state_.velocity[RIGHT] = joints_[RIGHT]->GetVelocity(0);
  joint_state_pub_.publish(joint_state_);

  /*
   * Odometry
   */
  odom_.header.stamp.sec = time_now.sec;
  odom_.header.stamp.nsec = time_now.nsec;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";
  odom_tf_.header = odom_.header;
  odom_tf_.child_frame_id = odom_.child_frame_id;

  // Distance travelled by front wheels
  double d1, d2;
  double dr, da;
  d1 = d2 = 0;
  dr = da = 0;
//  if (set_joints_[LEFT])
  d1 = step_time.Double() * (wheel_diam_ / 2) * joints_[LEFT]->GetVelocity(0);
//  if (set_joints_[RIGHT])
  d2 = step_time.Double() * (wheel_diam_ / 2) * joints_[RIGHT]->GetVelocity(0);
  // Can see NaN values here, just zero them out if needed
  if (isnan(d1))
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Kobuki plugin: NaN in d1. Step time: " << step_time.Double()
                             << ", WD: " << wheel_diam_ << ", velocity: " << joints_[LEFT]->GetVelocity(0));
    d1 = 0;
  }
  if (isnan(d2))
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Kobuki plugin: NaN in d2. Step time: " << step_time.Double()
                             << ", WD: " << wheel_diam_ << ", velocity: " << joints_[RIGHT]->GetVelocity(0));
    d2 = 0;
  }
  dr = (d1 + d2) / 2;
  da = (d2 - d1) / wheel_sep_;

  // Compute odometric pose
  odom_pose_[0] += dr * cos( odom_pose_[2] );
  odom_pose_[1] += dr * sin( odom_pose_[2] );
  odom_pose_[2] += da;
  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = da / step_time.Double();

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf::Quaternion qt;
  qt.setEuler(0,0,odom_pose_[2]);
  odom_.pose.pose.orientation.x = qt.getX();
  odom_.pose.pose.orientation.y = qt.getY();
  odom_.pose.pose.orientation.z = qt.getZ();
  odom_.pose.pose.orientation.w = qt.getW();

  memcpy(&odom_.pose.covariance[0], pose_cov_, sizeof(double)*36);
  memcpy(&odom_.twist.covariance[0], pose_cov_, sizeof(double)*36);

  odom_.twist.twist.linear.x = 0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = 0;
  odom_pub_.publish(odom_); // publish odom message
  odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf_.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf_.transform.rotation = odom_.pose.pose.orientation;
  tf_broadcaster_.sendTransform(odom_tf_);

  /*
   * Propagate velocity commands
   * TODO: Check how to simulate disabled motors, e.g. set MaxForce to zero, but then damping is important!
   */
  if (((time_now - last_cmd_vel_time_).Double() > cmd_vel_timeout_) || !motors_enabled_)
  {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }
  joints_[LEFT]->SetVelocity(0, wheel_speed_cmd_[LEFT] / (wheel_diam_ / 2.0));
  joints_[RIGHT]->SetVelocity(0, wheel_speed_cmd_[RIGHT] / (wheel_diam_ / 2.0));
  joints_[LEFT]->SetMaxForce(0, torque_);
  joints_[RIGHT]->SetMaxForce(0, torque_);

  /*
   * Cliff sensors
   */
  // check current state
  cliff_event_.sensor = 0;
  cliff_event_.state = kobuki_msgs::CliffEvent::FLOOR;
  if (cliff_sensor_left_->GetRange(0) >= cliff_detection_threshold_)
  {
    cliff_event_.sensor += kobuki_msgs::CliffEvent::LEFT;
    cliff_event_.state = kobuki_msgs::CliffEvent::CLIFF;
    max_floot_dist_ = cliff_sensor_left_->GetRange(0);
  }
  if (cliff_sensor_front_->GetRange(0) >= cliff_detection_threshold_)
  {
    cliff_event_.sensor += kobuki_msgs::CliffEvent::CENTER;
    cliff_event_.state = kobuki_msgs::CliffEvent::CLIFF;
    if (cliff_sensor_front_->GetRange(0) > max_floot_dist_)
    {
      max_floot_dist_ = cliff_sensor_front_->GetRange(0);
    }
  }
  if (cliff_sensor_right_->GetRange(0) >= cliff_detection_threshold_)
  {
    cliff_event_.sensor += kobuki_msgs::CliffEvent::RIGHT;
    cliff_event_.state = kobuki_msgs::CliffEvent::CLIFF;
    if (cliff_sensor_right_->GetRange(0) > max_floot_dist_)
    {
      max_floot_dist_ = cliff_sensor_right_->GetRange(0);
    }
  }
  // Only publish new message, if something has changed
  if ((cliff_event_.state == kobuki_msgs::CliffEvent::CLIFF)
      && (cliff_event_.sensor != cliff_event_old_.sensor))
  {
//    max_floot_dist_ = static_cast<int>(0.995f / ( tan( static_cast<float>( m_pk.psd[i]) / 76123.0f )
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, max_floot_dist_)); // convert distance back to an AD reading
    cliff_event_pub_.publish(cliff_event_);
    cliff_event_old_ = cliff_event_;
  }
  /*
   * Bumpers
   */
  msgs::Contacts contacts;
  contacts = bumper_->GetContacts();
//  for (int i = 0; i < contacts.contact_size(); ++i)
//  {
//    std::cout << "Collision between[" << contacts.contact(i).collision1()
//              << "] and [" << contacts.contact(i).collision2() << "]\n";
//
//    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
//    {
//      std::cout << j << "  Position:"
//                << contacts.contact(i).position(j).x() << " "
//                << contacts.contact(i).position(j).y() << " "
//                << contacts.contact(i).position(j).z() << "\n";
//      std::cout << "   Normal:"
//                << contacts.contact(i).normal(j).x() << " "
//                << contacts.contact(i).normal(j).y() << " "
//                << contacts.contact(i).normal(j).z() << "\n";
//      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
//    }
//  }

  // In order to simulate the three bumper sensors, a contact is assigned to one of the bumpers
  // depending on its position. Each sensor covers a range of 60 degrees.
  // +90 ... +30: left bumper
  // +30 ... -30: centre bumper
  // -30 ... -90: right bumper
  bumper_event_.state = 0;
  bumper_event_.bumper = 0;
  // flags used for avoiding multiple triggering of the same bumper due to multiple contacts
  bool bumper_left_pressed = false;
  bool bumper_centre_pressed = false;
  bool bumper_right_pressed = false;


  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    if ((contacts.contact(i).position(0).z() >= 0.015)
        && (contacts.contact(i).position(0).z() <= 0.085)) // only consider contacts at the height of the bumper
    {
      math::Pose current_pose = model_->GetWorldPose();
      double robot_heading = current_pose.rot.GetYaw();
      // using the force normals below, since the contact position is given in world coordinates
      // negate normal, because it points from contact to robot centre
      double global_contact_angle = std::atan2(-contacts.contact(i).normal(0).y(), -contacts.contact(i).normal(0).x());
      double relative_contact_angle = global_contact_angle - robot_heading;

//      std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << std::endl;
//                std::cout << "   Position:"
//                          << contacts.contact(i).position(0).x() << " "
//                          << contacts.contact(i).position(0).y() << " "
//                          << contacts.contact(i).position(0).z() << "\n";
//                std::cout << "   Normal:"
//                          << contacts.contact(i).normal(0).x() << " "
//                          << contacts.contact(i).normal(0).y() << " "
//                          << contacts.contact(i).normal(0).z() << "\n";
//      std::cout << "Current robot heading: " << (robot_heading * (180/M_PI)) << std::endl;
//      std::cout << "Global contact angle: " << (contact_angle * (180/M_PI)) << std::endl;
//      std::cout << "Robot contact angle: " << (relative_contact_angle * (180/M_PI)) << std::endl;
      if ((relative_contact_angle <= (M_PI/2)) && (relative_contact_angle > (M_PI/6)))
      {
        if (!bumper_left_pressed)
        {
          bumper_left_pressed = true;
          bumper_event_.state = kobuki_msgs::BumperEvent::PRESSED;
          bumper_event_.bumper += kobuki_msgs::BumperEvent::LEFT;
//          std::cout << "Left bumper pressed." << std::endl;
//          std::cout << "-----------------------------------------" << std::endl;
        }
      }
      else if ((relative_contact_angle <= (M_PI/6)) && (relative_contact_angle >= (-M_PI/6)))
      {
        if (!bumper_centre_pressed)
        {
          bumper_centre_pressed = true;
          bumper_event_.state = kobuki_msgs::BumperEvent::PRESSED;
          bumper_event_.bumper += kobuki_msgs::BumperEvent::CENTER;
//          std::cout << "Centre bumper pressed." << std::endl;
//          std::cout << "-----------------------------------------" << std::endl;
        }
      }
      else if ((relative_contact_angle < (-M_PI/6)) && (relative_contact_angle >= (-M_PI/2)))
      {
        if (!bumper_right_pressed)
        {
          bumper_right_pressed = true;
          bumper_event_.state = kobuki_msgs::BumperEvent::PRESSED;
          bumper_event_.bumper += kobuki_msgs::BumperEvent::RIGHT;
//          std::cout << "Right bumper pressed." << std::endl;
//          std::cout << "-----------------------------------------" << std::endl;
        }
      }
    }
  }
  // Only publish new message, if something has changed
  if ((bumper_event_.state != bumper_event_old_.state)
      || (bumper_event_.bumper != bumper_event_old_.bumper))
  {
    bumper_event_pub_.publish(bumper_event_);
    bumper_event_old_ = bumper_event_;
  }
}


void GazeboRosKobuki::spin()
{
  while(ros::ok() && !shutdown_requested_)
  {
    ros::spinOnce();
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosKobuki);

} // namespace gazebo
