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
#include <math.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "builtin_interfaces/msg/time.hpp"
using namespace std::chrono_literals;

class MoveTest1 : public rclcpp::Node
{
public:
  size_t count = 0;
  explicit MoveTest1(const std::string & topic_name)
  : Node("move_test1")
  {
    scara_arm.name.resize(3);
    scara_arm.name[0] = "base_to_arm1";
    scara_arm.name[1] = "arm1_to_arm2";
    scara_arm.name[2] = "end_joint";
    scara_arm.position.resize(3);
    // タイマー実行されるイベントハンドラー関数
    auto publish_message =
      [this]() -> void  // ラムダ式による関数オブジェクトの定義
      {
        scara_arm.header.stamp = clock->now();
        scara_arm.position[0] = ((3.141592/2) * sin((float(count % 314) / 50.0)));
        scara_arm.position[1] = ((3.141592/3) * sin((float(count % 157) / 25.0))) + (3.141592/3);
        scara_arm.position[2] = 0.0;
        scara_arm_pub->publish(scara_arm);
        count++;
      };

    // joint_stateトピックの送信設定
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    scara_arm_pub = create_publisher<sensor_msgs::msg::JointState>(topic_name, qos);
    // publish_messageの50ミリ秒周期でのタイマー実行
    timer_ = create_wall_timer(50ms, publish_message);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr scara_arm_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState scara_arm;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
};

int main(int argc, char * argv[])
{
  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // joint_state_publisherノードの生成とスピン開始
  auto node = std::make_shared<MoveTest1>("joint_states");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}