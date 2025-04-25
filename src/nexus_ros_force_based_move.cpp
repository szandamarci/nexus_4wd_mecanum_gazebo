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

#include <nexus_4wd_mecanum_gazebo/nexus_ros_force_based_move.h>

namespace gazebo 
{

  GazeboRosForceBasedMove::GazeboRosForceBasedMove() {}

  GazeboRosForceBasedMove::~GazeboRosForceBasedMove() {}

  // Load the controller
  void GazeboRosForceBasedMove::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      RCLCPP_INFO(node_->get_logger(), "ForceBasedPlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      RCLCPP_WARN(node_->get_logger(), "ForceBasedPlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      RCLCPP_WARN(node_->get_logger(), "ForceBasedPlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      RCLCPP_WARN(node_->get_logger(), "ForceBasedPlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }
    
    ////////////////////////////////

    torque_yaw_velocity_p_gain_ = 1.0;
    force_x_velocity_p_gain_ = 15.0;
    force_y_velocity_p_gain_ = 15.0;
    
    if (sdf->HasElement("yaw_velocity_p_gain"))
      (sdf->GetElement("yaw_velocity_p_gain")->GetValue()->Get(torque_yaw_velocity_p_gain_));

    if (sdf->HasElement("x_velocity_p_gain"))
      (sdf->GetElement("x_velocity_p_gain")->GetValue()->Get(force_x_velocity_p_gain_));

    if (sdf->HasElement("y_velocity_p_gain"))
      (sdf->GetElement("y_velocity_p_gain")->GetValue()->Get(force_y_velocity_p_gain_));
      
    RCLCPP_INFO_STREAM(node_->get_logger(), "ForceBasedMove using gains: yaw: " << torque_yaw_velocity_p_gain_ <<
                                                 " x: " << force_x_velocity_p_gain_ <<
                                                 " y: " << force_y_velocity_p_gain_ << "\n");

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame")) 
    {
      RCLCPP_WARN(node_->get_logger(), "ForceBasedPlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    } 
    else 
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "robotBaseFrame for force based move plugin: " << robot_base_frame_  << "\n");

    this->link_ = parent->GetLink(robot_base_frame_);

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      RCLCPP_WARN(node_->get_logger(), "ForceBasedPlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }
    cmd_vel_time_out_ = 0.25;
    if (!sdf->HasElement("cmdVelTimeOut"))
    {
      RCLCPP_WARN(node_->get_logger(), "ForceBasedPlugin (ns = %s) missing <cmdVelTimeOut>, "
          "defaults to %f",
          robot_namespace_.c_str(), cmd_vel_time_out_);
    }
    else
    {
      cmd_vel_time_out_ = sdf->GetElement("cmdVelTimeOut")->Get<double>();
    }

    this->publish_odometry_tf_ = true;
    if (!sdf->HasElement("publishOdometryTf")) {
      RCLCPP_WARN(node_->get_logger(), "ForceBasedPlugin Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    } else {
      this->publish_odometry_tf_ = sdf->GetElement("publishOdometryTf")->Get<bool>();
    }

    max_x_velocity = 0.6;
    if (sdf->HasElement("max_x_velocity"))
      (sdf->GetElement("max_x_velocity")->GetValue()->Get(max_x_velocity));

    max_y_velocity = 0.6;
    if (sdf->HasElement("max_y_velocity"))
      (sdf->GetElement("max_y_velocity")->GetValue()->Get(max_y_velocity));

    max_yaw_velocity = 0.5;
    if (sdf->HasElement("max_yaw_velocity"))
      (sdf->GetElement("max_yaw_velocity")->GetValue()->Get(max_yaw_velocity));
 
#if (GAZEBO_MAJOR_VERSION >= 8)
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
#endif
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    odom_transform_.setIdentity();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!node_) 
    {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "ForceBasedPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new rclcpp::Node(robot_namespace_));

    RCLCPP_DEBUG(node_->get_logger(), "OCPlugin (%s) has started!", 
        robot_namespace_.c_str());



    if (publish_odometry_tf_)
      transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // subscribe to the odometry topic
    // ros::SubscribeOptions so =
    //   ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
    //       boost::bind(&GazeboRosForceBasedMove::cmdVelCallback, this, _1),
    //       ros::VoidPtr(), &queue_);

    vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(command_topic_, rclcpp::QoS(10), std::bind(&GazeboRosForceBasedMove::cmdVelCallback, this, std::placeholders::_1));
    odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::QoS(10));

    // start custom queue for diff drive
    // callback_queue_thread_ = 
    //   boost::thread(boost::bind(&GazeboRosForceBasedMove::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosForceBasedMove::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosForceBasedMove::UpdateChild()
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d pose = parent_->WorldPose();

    //If no command received recenlty, set command velocity to zero
    if ((parent_->GetWorld()->SimTime() - last_cmd_vel_time_) > cmd_vel_time_out_) {
      x_ = 0.0;
      y_ = 0.0;
      rot_ = 0.0;
    }

    ignition::math::Vector3d angular_vel = parent_->WorldAngularVel();

    double error = angular_vel.Z() - std::min(rot_, max_yaw_velocity);

    link_->AddTorque(ignition::math::Vector3d(0.0, 0.0, -error * torque_yaw_velocity_p_gain_));

    //float yaw = pose.Rot().Yaw();

    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

    link_->AddRelativeForce(ignition::math::Vector3d((std::min(x_, max_x_velocity) - linear_vel.X())* force_x_velocity_p_gain_,
                                                     (std::min(y_, max_x_velocity) - linear_vel.Y())* force_y_velocity_p_gain_,
                                                      0.0));
#else
    math::Pose pose = parent_->GetWorldPose();

    //If no command received recenlty, set command velocity to zero
    if ((parent_->GetWorld()->GetSimTime() - last_cmd_vel_time_) > cmd_vel_time_out_) {
      x_ = 0.0;
      y_ = 0.0;
      rot_ = 0.0;
    }

    math::Vector3 angular_vel = parent_->GetWorldAngularVel();

    double error = angular_vel.z - std::min(rot_, max_yaw_velocity);

    link_->AddTorque(math::Vector3(0.0, 0.0, -error * torque_yaw_velocity_p_gain_));

    float yaw = pose.rot.GetYaw();

    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    link_->AddRelativeForce(math::Vector3((std::min(x_, max_x_velocity) - linear_vel.x)* force_x_velocity_p_gain_,
                                          (std::min(y_, max_x_velocity) - linear_vel.y)* force_y_velocity_p_gain_,
                                           0.0));
#endif
    //parent_->PlaceOnNearestEntityBelow();
    //parent_->SetLinearVel(math::Vector3(
    //      x_ * cosf(yaw) - y_ * sinf(yaw),
    //      y_ * cosf(yaw) + x_ * sinf(yaw),
    //      0));
    //parent_->SetAngularVel(math::Vector3(0, 0, rot_));

    if (odometry_rate_ > 0.0) {
#if (GAZEBO_MAJOR_VERSION >= 8)
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosForceBasedMove::FiniChild() {
    alive_ = false;
    if (node_){
      vel_sub_.reset();
      odometry_pub_.reset();
      transform_broadcaster_.reset();      
    }
  }

  void GazeboRosForceBasedMove::cmdVelCallback(
      const geometry_msgs::msg::Twist::SharedPtr cmd_msg) 
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;

#if (GAZEBO_MAJOR_VERSION >= 8)
    last_cmd_vel_time_= parent_->GetWorld()->SimTime();
#else
    last_cmd_vel_time_= parent_->GetWorld()->GetSimTime();
#endif
  }

  // void GazeboRosForceBasedMove::QueueThread()
  // {
  //   static const double timeout = 0.01;
  //   while (alive_ && rosnode_->ok()) 
  //   {
  //     queue_.callAvailable(ros::WallDuration(timeout));
  //   }
  // }

  void GazeboRosForceBasedMove::publishOdometry(double step_time)
  {

    auto current_time = node_->now();

    std::string odom_frame = tf2::getFrameId(odometry_frame_);
    std::string base_footprint_frame = tf2::getFrameId(robot_base_frame_);

#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

    odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.X(), linear_vel.Y(), angular_vel.Z(), step_time);
    
    geometry_msgs::msg::Pose pose_msg;
    tf2::toMsg(odom_transform_, pose_msg);
    odom_.pose.pose = pose_msg;

    odom_.twist.twist.angular.z = angular_vel.Z();
    odom_.twist.twist.linear.x  = linear_vel.X();
    odom_.twist.twist.linear.y  = linear_vel.Y();
#else
    math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.x, linear_vel.y, angular_vel.z, step_time);

    geometry_msgs::msg::Pose pose_msg;
    tf2::toMsg(odom_transform_, pose_msg);
    odom_.pose.pose = pose_msg;
    odom_.twist.twist.angular.z = angular_vel.z;
    odom_.twist.twist.linear.x  = linear_vel.x;
    odom_.twist.twist.linear.y  = linear_vel.y;
#endif

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (transform_broadcaster_.get()){
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = current_time;
      transform.header.frame_id = odom_frame;
      transform.child_frame_id = base_footprint_frame;
      transform.transform.translation.x = odom_transform_.getOrigin().x();
      transform.transform.translation.y = odom_transform_.getOrigin().y();
      transform.transform.translation.z = odom_transform_.getOrigin().z();
      transform.transform.rotation.x = odom_transform_.getRotation().x();
      transform.transform.rotation.y = odom_transform_.getOrigin().y();
      transform.transform.rotation.z = odom_transform_.getOrigin().z();
      transform.transform.rotation.w = odom_transform_.getOrigin().w();
      
      transform_broadcaster_->sendTransform(transform);
    }
    
    odom_.pose.covariance[0] = 0.001;
    odom_.pose.covariance[7] = 0.001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    
#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.pose.covariance[35] = 0.01;
    }else{
      odom_.pose.covariance[35] = 100.0;
    }

    odom_.twist.covariance[0] = 0.001;
    odom_.twist.covariance[7] = 0.001;
    odom_.twist.covariance[14] = 0.001;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;

#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.twist.covariance[35] = 0.01;
    }else{
      odom_.twist.covariance[35] = 100.0;
    }

    odometry_pub_->publish(odom_);
  }


  tf2::Transform GazeboRosForceBasedMove::getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const
  {
    tf2::Transform tmp;
    tmp.setIdentity();

    if (std::abs(angular_vel) < 0.0001) {
      //Drive straight
      tmp.setOrigin(tf2::Vector3(static_cast<double>(linear_vel_x*timeSeconds), static_cast<double>(linear_vel_y*timeSeconds), 0.0));
    } else{
        const double distX = linear_vel_x * timeSeconds;
        const double distY = linear_vel_y * timeSeconds;

        if (std::abs(distX) < 0.0001 && std::abs(distY) < 0.0001) {
          //Rotate on spot
          const double angleChange = angular_vel * timeSeconds;
          tmp.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), angleChange));
        } else {
          //Follow circular arc
          const double distChange = std::sqrt(distX * distX + distY * distY);
          const double angleDriveDirection = std::atan2(distY, distX);
          const double angleChange = angular_vel * timeSeconds;

          const double arcRadius = distChange / angleChange;

          tf2::Vector3 endPos = tf2::Vector3(std::sin(angleChange) * arcRadius,
                                    arcRadius - std::cos(angleChange) * arcRadius,
                                    0.0);
          endPos = tf2::quatRotate(tf2::Quaternion(tf2::Vector3(0, 0, 1), angleDriveDirection), endPos);                          

          tmp.setOrigin(endPos);

          tmp.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), angleChange));
        }
    }

    return tmp;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceBasedMove)
}