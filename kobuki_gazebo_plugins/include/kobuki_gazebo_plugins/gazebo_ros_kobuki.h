/*
 * Copyright (c) 2012, Yujin Robot.
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
 *
 * This work is based on the Gazebo ROS plugin for the iRobot Create by Nate Koenig.
 */

#ifndef GAZEBO_ROS_KOBUKI_H
#define GAZEBO_ROS_KOBUKI_H

#include "physics/physics.h"
#include "physics/PhysicsTypes.hh"
#include "sensors/SensorTypes.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

namespace gazebo
{
  class GazeboRosKobuki : public ModelPlugin
  {
    public: 
      GazeboRosKobuki();
      virtual ~GazeboRosKobuki();
          
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();
  
    private:

      void UpdateSensors();
      void OnContact(const std::string &name, const physics::Contact &contact);
      void OnCmdVel( const geometry_msgs::TwistConstPtr &msg);


      /// Parameters
      std::string node_namespace_;
      std::string left_wheel_joint_name_;
      std::string right_wheel_joint_name_;
      std::string base_geom_name_;

      /// Separation between the wheels
      float wheel_sep_;

      /// Diameter of the wheels
      float wheel_diam_;

      ///Torque applied to the wheels
      float torque_;


      ros::NodeHandle *rosnode_;
      //ros::Service operating_mode_srv_;
      //ros::Service digital_output_srv_;
  
      ros::Publisher sensor_state_pub_;
      ros::Publisher odom_pub_;
      ros::Publisher joint_state_pub_;
  
      ros::Subscriber cmd_vel_sub_;

      physics::WorldPtr my_world_;
      physics::ModelPtr my_parent_;

      /// Speeds of the wheels
      float *wheel_speed_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      float odom_pose_[3];
      float odom_vel_[3];

      bool set_joints_[2];
      physics::JointPtr joints_[2];
      physics::CollisionPtr base_geom_;

      sensors::RaySensorPtr left_cliff_sensor_;
      sensors::RaySensorPtr right_cliff_sensor_;
      sensors::RaySensorPtr front_cliff_sensor_;

      tf::TransformBroadcaster transform_broadcaster_;
      sensor_msgs::JointState js_;

      turtlebot_node::TurtlebotSensorState sensor_state_;

      void spin();
      boost::thread *spinner_thread_;

      event::ConnectionPtr contact_event_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };
}
#endif
