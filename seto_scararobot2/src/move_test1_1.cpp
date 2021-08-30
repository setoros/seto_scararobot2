// Copyright 2020-2021 SETOUCHI ROS STUDY GROUP
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

#include <string>
#include <cmath>
#include <iostream>
#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("move_test1_1");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  rclcpp::WallRate loop_rate(50);

  sensor_msgs::msg::JointState scara_arm;
  scara_arm.name.push_back("base_to_arm1");
  scara_arm.name.push_back("arm1_to_arm2");
  scara_arm.name.push_back("end_joint");
  scara_arm.position.push_back(0.0);
  scara_arm.position.push_back(0.0);
  scara_arm.position.push_back(0.0);

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  auto counter = 0.0;
  auto joint_value = 0.0;
  while (rclcpp::ok()) {
    counter += 0.1;
    joint_value = std::sin(counter);

    for (size_t i = 0; i < scara_arm.name.size(); ++i) {
      scara_arm.position[i] = joint_value;
    }

    scara_arm.header.stamp = clock->now();

    joint_state_pub->publish(scara_arm);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}