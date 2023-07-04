// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;

void topic_callback(const nav_msgs::Odometry::ConstPtr & ros1_msg)
{
  auto ros2_msg = std::make_unique<nav_msgs::msg::Odometry>();

  
  ros2_msg->header.stamp.sec = ros1_msg->header.stamp.sec;
  ros2_msg->header.stamp.nanosec = ros1_msg->header.stamp.nsec;
  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  
  
  ros2_msg->pose.pose.position.x = ros1_msg->pose.pose.position.x;
  ros2_msg->pose.pose.position.y = ros1_msg->pose.pose.position.y;
  ros2_msg->pose.pose.position.z = ros1_msg->pose.pose.position.z;

  ros2_msg->pose.pose.orientation.x = ros1_msg->pose.pose.orientation.x;
  ros2_msg->pose.pose.orientation.y = ros1_msg->pose.pose.orientation.y;
  ros2_msg->pose.pose.orientation.z = ros1_msg->pose.pose.orientation.z;
  ros2_msg->pose.pose.orientation.w = ros1_msg->pose.pose.orientation.w;  

  for (int i = 0; i < 36; i++){
    ros2_msg->pose.covariance[i] = ros1_msg->pose.covariance[i];
  }
  ros2_msg->twist.twist.linear.x = ros1_msg->twist.twist.linear.x;
  ros2_msg->twist.twist.linear.y = ros1_msg->twist.twist.linear.y;
  ros2_msg->twist.twist.linear.z = ros1_msg->twist.twist.linear.z;

  ros2_msg->twist.twist.angular.x = ros1_msg->twist.twist.angular.x;
  ros2_msg->twist.twist.angular.y = ros1_msg->twist.twist.angular.y;
  ros2_msg->twist.twist.angular.z = ros1_msg->twist.twist.angular.z;  


  for (int i = 0; i < 36; i++){
    ros2_msg->twist.covariance[i] = ros1_msg->twist.covariance[i];
  }

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{

    static const rmw_qos_profile_t rmw_qos_profile_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    50,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      rmw_qos_profile_default.history,
      rmw_qos_profile_default.depth
    ),
    rmw_qos_profile_default);

  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bridge_talker_odom");
  node->declare_parameter("topic_name", "topic");
  std::string topic_name =  node->get_parameter("topic_name").get_parameter_value().get<std::string>();
  pub = node->create_publisher<nav_msgs::msg::Odometry>(topic_name, qos);

  // ROS 1 node and subscriber
  ros::init(argc, argv, "bridge_listener_odom");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_name, 1, topic_callback);

  ros::spin();

  return 0;
}
