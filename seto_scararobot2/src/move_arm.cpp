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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr node = nullptr;
geometry_msgs::msg::Point arm_position;
bool is_reveived_point = false;

void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg){
   arm_position.x = msg->x;
   arm_position.y = msg->y;
   is_reveived_point = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("move_arm");
  auto joint_state_pub= node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  auto arm_state_pub = node->create_publisher<std_msgs::msg::String>("arm_states", 10);
  auto sub_ = node->create_subscription<geometry_msgs::msg::Point>(
      "arm_positions", 100, std::bind(topic_callback, std::placeholders::_1));

  rclcpp::WallRate loop_rate(100);

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  sensor_msgs::msg::JointState scara_arm;
  std_msgs::msg::String arm_state;

  scara_arm.name.push_back("base_to_arm1");
  scara_arm.name.push_back("arm1_to_arm2");
  scara_arm.name.push_back("end_joint");
  scara_arm.position.push_back(0.0);
  scara_arm.position.push_back(0.0);
  scara_arm.position.push_back(0.0);
  scara_arm.header.stamp = clock->now();
  joint_state_pub->publish(scara_arm);


  while (rclcpp::ok()) {
    scara_arm.position[0] = arm_position.x;
    scara_arm.position[1] = arm_position.y;
    scara_arm.position[2] = 0.0;
    scara_arm.header.stamp = clock->now();
    joint_state_pub->publish(scara_arm);
    if(is_reveived_point == true)
    {
      // ダイナミクセルから座標を取得して情報を送る
      // 未実装
      // arm_state.data = "Waiting";
      // arm_state.data = "Moving";
      arm_state.data = "Goal";
      arm_state_pub->publish(arm_state);
      is_reveived_point = false;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}