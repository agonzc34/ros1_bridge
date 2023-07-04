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
#include "sensor_msgs/CameraInfo.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub;

void topic_callback(const sensor_msgs::CameraInfo::ConstPtr & ros1_msg)
{
  auto ros2_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();

  ros2_msg->header.stamp.sec = ros1_msg->header.stamp.sec;
  ros2_msg->header.stamp.nanosec = ros1_msg->header.stamp.nsec;
  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  
  ros2_msg->height = ros1_msg->height;
  ros2_msg->width = ros1_msg->width;
  ros2_msg->distortion_model = ros1_msg->distortion_model;
  ros2_msg->d = ros1_msg->D;

  for(int i=0; i < 9; i++)
  {
    ros2_msg->k[i] = ros1_msg->K[i];
    ros2_msg->r[i] = ros1_msg->R[i];
  }

  for(int i=0; i < 12; i++)
  {
    ros2_msg->p[i] = ros1_msg->P[i];
  }

  ros2_msg->binning_x = ros1_msg->binning_x;
  ros2_msg->binning_y = ros1_msg->binning_y;

  ros2_msg->roi.x_offset = ros1_msg->roi.x_offset;
  ros2_msg->roi.y_offset = ros1_msg->roi.y_offset;
  ros2_msg->roi.height = ros1_msg->roi.height;
  ros2_msg->roi.width = ros1_msg->roi.width;
  ros2_msg->roi.do_rectify = ros1_msg->roi.do_rectify;

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{

    static const rmw_qos_profile_t rmw_qos_profile_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    30,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
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
  auto node = rclcpp::Node::make_shared("bridge_camera_info");
  node->declare_parameter("topic_name", "topic");
  std::string topic_name =  node->get_parameter("topic_name").get_parameter_value().get<std::string>();
  pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name, qos);

  // ROS 1 node and subscriber
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_name, 1, topic_callback);

  ros::spin();

  return 0;
}
