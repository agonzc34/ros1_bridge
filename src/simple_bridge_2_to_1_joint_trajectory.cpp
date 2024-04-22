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
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


ros::Publisher pub;

void topic_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr ros2_msg)
{

  trajectory_msgs::JointTrajectory ros1_msg;

  ros1_msg.header.stamp.sec = ros2_msg->header.stamp.sec;
  ros1_msg.header.stamp.nsec = ros2_msg->header.stamp.nanosec;
  ros1_msg.header.frame_id = ros2_msg->header.frame_id;

  ros1_msg.joint_names = ros2_msg->joint_names;

  for (size_t i = 0; i < ros2_msg->joint_names.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint point;

    for (size_t j = 0; j < ros2_msg->points[i].positions.size(); j++) {
      ros1_msg.points[i].positions.push_back(ros2_msg->points[i].positions[j]);
    }
    for (size_t j = 0; j < ros2_msg->points[i].velocities.size(); j++) {
      ros1_msg.points[i].velocities.push_back(ros2_msg->points[i].velocities[j]);
    }
    for (size_t j = 0; j < ros2_msg->points[i].accelerations.size(); j++) {
      ros1_msg.points[i].accelerations.push_back(ros2_msg->points[i].accelerations[j]);
    }
    for (size_t j = 0; j < ros2_msg->points[i].effort.size(); j++) {
      ros1_msg.points[i].effort.push_back(ros2_msg->points[i].effort[j]);
    }
    ros1_msg.points[i].time_from_start.sec = ros2_msg->points[i].time_from_start.sec;
    ros1_msg.points[i].time_from_start.nsec = ros2_msg->points[i].time_from_start.nanosec;
  }

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
  auto node = rclcpp::Node::make_shared("bridge_joint_trajectory");
  node->declare_parameter("topic_name", "topic");
  std::string topic_name =  node->get_parameter("topic_name").get_parameter_value().get<std::string>();

  // ROS 1 node and publisher
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  pub = n.advertise<trajectory_msgs::JointTrajectory>(topic_name, 1);

  // ROS 2 susbscriber
  auto sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    topic_name, qos, topic_callback);

  rclcpp::spin(node);

  return 0;
}
