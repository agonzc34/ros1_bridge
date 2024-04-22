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
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr pub;

void copy_trajectory_point(
  const trajectory_msgs::JointTrajectoryPoint & pt1,
  trajectory_msgs::msg::JointTrajectoryPoint & pt2)
{
  pt2.positions = pt1.positions;
  pt2.velocities = pt1.velocities;
  pt2.accelerations = pt1.accelerations;
  pt2.effort = pt1.effort;
  pt2.time_from_start.sec = pt1.time_from_start.sec;
  pt2.time_from_start.nanosec = pt1.time_from_start.nsec;
}

void topic_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & ros1_msg)
{
  auto ros2_msg = std::make_unique<control_msgs::msg::JointTrajectoryControllerState>();
  
  ros2_msg->header.stamp.sec = ros1_msg->header.stamp.sec;
  ros2_msg->header.stamp.nanosec = ros1_msg->header.stamp.nsec;
  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  
  for (size_t i = 0; i < ros1_msg->joint_names.size(); i++) {
    ros2_msg->joint_names.push_back(ros1_msg->joint_names[i]);
  }

  copy_trajectory_point(ros1_msg->desired, ros2_msg->desired);
  copy_trajectory_point(ros1_msg->actual, ros2_msg->actual);
  copy_trajectory_point(ros1_msg->error, ros2_msg->error);

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{

    static const rmw_qos_profile_t rmw_qos_profile_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    200,
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
  auto node = rclcpp::Node::make_shared("bridge_joint_trajectory_controller_state");
  node->declare_parameter("topic_name", "topic");
  std::string topic_name =  node->get_parameter("topic_name").get_parameter_value().get<std::string>();
  pub = node->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(topic_name, qos);

  // ROS 1 node and subscriber
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_name, 1, topic_callback);

  ros::spin();

  return 0;
}
