/*
 * Copyright 2015 Stefan Kohlbrecher, TU Darmstadt
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to exert
 *       forces on a robot, resulting in motion. Based on the
 *       planar_move plugin by Piyush Khandelwal.
 * Author: Stefan Kohlbrecher
 * Date: 06 August 2015
 */

#ifndef GAZEBO_ROS_FORCE_BASED_MOVE_HH
#define GAZEBO_ROS_FORCE_BASED_MOVE_HH

#include <memory>
#include <mutex>
#include <string>
#include <map>
#include <algorithm>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/msg/twist.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace gazebo {

  class GazeboRosForceBasedMove : public ModelPlugin {

    public: 
      GazeboRosForceBasedMove();
      ~GazeboRosForceBasedMove();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      std::shared_ptr<rclcpp::Node> node_;
      void publishOdometry(double step_time);

      tf2::Transform getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const;

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      /// \brief A pointer to the Link, where force is applied
      physics::LinkPtr link_;

      /// \brief The Link this plugin is attached to, and will exert forces on.
      private: std::string link_name_;

      rclcpp::Node::SharedPtr rosnode_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::msg::Odometry odom_;
      std::string tf_prefix_;

      tf2::Transform odom_transform_;

      std::mutex lock_;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      double odometry_rate_;
      double cmd_vel_time_out_;
      bool publish_odometry_tf_;

      // // Custom Callback Queue
      // ros::CallbackQueue queue_;
      // boost::thread callback_queue_thread_;
      // void QueueThread();

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_msg);
      common::Time last_cmd_vel_time_;

      double x_;
      double y_;
      double rot_;
      bool alive_;
      common::Time last_odom_publish_time_;
#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Pose3d last_odom_pose_;
#else
      math::Pose last_odom_pose_;
#endif
      
      double torque_yaw_velocity_p_gain_;
      double force_x_velocity_p_gain_;
      double force_y_velocity_p_gain_;

      double max_x_velocity;
      double max_y_velocity;
      double max_yaw_velocity;

  };

}

#endif /* end of include guard: GAZEBO_ROS_PLANAR_MOVE_HH */
