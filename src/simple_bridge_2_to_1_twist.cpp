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

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


ros::Publisher pub;

void topic_callback(const geometry_msgs::msg::Twist::SharedPtr ros2_msg)
{

  geometry_msgs::Twist ros1_msg;

  ros1_msg.linear.x = ros2_msg->linear.x;
  ros1_msg.linear.y = ros2_msg->linear.y;
  ros1_msg.linear.z = ros2_msg->linear.z;

  ros1_msg.angular.x = ros2_msg->angular.x;
  ros1_msg.angular.y = ros2_msg->angular.y;
  ros1_msg.angular.z = ros2_msg->angular.z;

  pub.publish(ros1_msg);
}

int main(int argc, char * argv[])
{

  static const rmw_qos_profile_t rmw_qos_profile_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    30,
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

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bridge_twist");
  node->declare_parameter("topic_name", "topic");
  std::string topic_name =  node->get_parameter("topic_name").get_parameter_value().get<std::string>();

  // ROS 1 node and publisher
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::Twist>(topic_name, 1);

  // ROS 2 susbscriber
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    topic_name, qos, topic_callback);

  rclcpp::spin(node);

  return 0;
}
