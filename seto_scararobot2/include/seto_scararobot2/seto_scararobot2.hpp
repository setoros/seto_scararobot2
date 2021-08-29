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

#ifndef SetoScaraRobot2__SetoScaraRobot2_HPP_
#define SetoScaraRobot2__SetoScaraRobot2_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <std_msgs/msg/string.hpp>

namespace seto_scara
{

class SetoScaraRobot2 : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  geometry_msgs::msg::TransformStamped tf_odom;
  rclcpp::Clock ros_clock;
  rclcpp::Time last_cmd_vel_time;

  enum Step
  {
    Initialize,
    WaitBeadsInfo,
    SearchSetBeads,
    CatchBeads,
    WaitCatchBeads,
    EndEffectorforCatch,
    WaitEndEffectorforCatch,
    SetBeads,
    WaitSetBeads,
    EndEffectorforSet,
    WaitEndEffectorforSet,
    CheckFinishTask
  };

  const std::string MSG_ARM_WAITING    = "Waiting";
  const std::string MSG_ARM_MOVING    = "Moving";
  const std::string MSG_ARM_GOAL    = "Goal";
  const std::string MSG_ENDEFFECTOR_WAITING    = "Waiting";
  const std::string MSG_ENDEFFECTOR_MOVING    = "Moving";
  const std::string MSG_ENDEFFECTOR_GOAL    = "Goal";
  const std::string MSG_ENDEFFECTOR_GRUB    = "grub";
  const std::string MSG_ENDEFFECTOR_RELEASE    = "release";
  int step; //スカラロボットの状態変数
  std_msgs::msg::String received_beads_positions; //受け取ったビーズ情報
  std_msgs::msg::String received_arm_states; //受け取ったアームの状態
  std::vector<int> arm_positions; // 受け取ったビーズの位置をアームの座標系に変換したものを保存
  geometry_msgs::msg::Point arm_position; //送信用のアーム情報

  std_msgs::msg::String move_endeffector;
  std_msgs::msg::String received_endeffector_states; //受け取ったエンドエフェクタの状態

  std_msgs::msg::String string_msg;
  geometry_msgs::msg::Point point_msg;


  //アームの状態変数
  bool is_arm_waiting;
  bool is_arm_moving;
  bool is_arm_goal;

  bool is_endeffector_waiting;
  bool is_endeffector_moving;
  bool is_endeffector_goal;
  
  //色番号
  //beads_setterのカラーコードと一致させておく
  const int SPACE = 0;
  const int COLOR1 = 1;
  const int COLOR2 = 2;
  const int COLOR3 = 3;
  

  std::vector<std::vector<int>> color_first = {
    {0, 8}, {1, 8}, {2, 8},
    {0, 9}, {1, 9}, {2, 9},
    {0, 10}, {1, 10}, {2, 10},
    {0, 11}, {1, 11}, {2, 11},
    {0, 12}, {1, 12}, {2, 12},
    {0, 13}, {1, 13}, {2, 13},
    {0, 14}, {1, 14}, {2, 14},
    {0, 15}, {1, 15}, {2, 15},
    {0, 16}, {1, 16}, {2, 16},
    {0, 17}, {1, 17}, {2, 17},
    {0, 18}, {1, 18}, {2, 18},
    {0, 19}, {1, 19}, {2, 19},
    {0, 20}, {1, 20}, {2, 20},
    {0, 21}, {1, 21}, {2, 21},
    {0, 22}, {1, 22}, {2, 22},
    {0, 23}, {1, 23}, {2, 23},
    {0, 24}, {1, 24}, {2, 24},
    {0, 25}, {1, 25}, {2, 25},
    {0, 26}, {1, 26}, {2, 26},
    {0, 27}, {1, 27}, {2, 27},
    {0, 28}, {1, 28}, {2, 28},
  };

  std::vector<std::vector<int>> color_second = {
    {3, 8}, {4, 8}, {5, 8},
    {3, 9}, {4, 9}, {5, 9},
    {3, 10}, {4, 10}, {5, 10},
    {3, 11}, {4, 11}, {5, 11},
    {3, 12}, {4, 12}, {5, 12},
    {3, 13}, {4, 13}, {5, 13},
    {3, 14}, {4, 14}, {5, 14},
    {3, 15}, {4, 15}, {5, 15},
    {3, 16}, {4, 16}, {5, 16},
    {3, 17}, {4, 17}, {5, 17},
    {3, 18}, {4, 18}, {5, 18},
    {3, 19}, {4, 19}, {5, 19},
    {3, 20}, {4, 20}, {5, 20},
    {3, 21}, {4, 21}, {5, 21},
    {3, 22}, {4, 22}, {5, 22},
    {3, 23}, {4, 23}, {5, 23},
    {3, 24}, {4, 24}, {5, 24},
    {3, 25}, {4, 25}, {5, 25},
    {3, 26}, {4, 26}, {5, 26},
    {3, 27}, {4, 27}, {5, 27},
    {3, 28}, {4, 28}, {5, 28},
  };

  std::vector<std::vector<int>> color_third = {
    {6, 8}, {7, 8}, {8, 8},
    {6, 9}, {7, 9}, {8, 9},
    {6, 10}, {7, 10}, {8, 10},
    {6, 11}, {7, 11}, {8, 11},
    {6, 12}, {7, 12}, {8, 12},
    {6, 13}, {7, 13}, {8, 13},
    {6, 14}, {7, 14}, {8, 14},
    {6, 18}, {7, 15}, {8, 15},
    {6, 16}, {7, 16}, {8, 16},
    {6, 17}, {7, 17}, {8, 17},
    {6, 18}, {7, 18}, {8, 18},
    {6, 19}, {7, 19}, {8, 19},
    {6, 20}, {7, 20}, {8, 20},
    {6, 21}, {7, 21}, {8, 21},
    {6, 22}, {7, 22}, {8, 22},
    {6, 23}, {7, 23}, {8, 23},
    {6, 24}, {7, 24}, {8, 24},
    {6, 25}, {7, 25}, {8, 25},
    {6, 26}, {7, 26}, {8, 26},
    {6, 27}, {7, 27}, {8, 27},
    {6, 28}, {7, 28}, {8, 28},
  };

  //オフセット位置(mm)
  const int OFFSET_X = 20;
  const int OFFSET_Y = 20;

  const double DISTANCE = 5; //ビーズ間の距離（mm）

  int set_count;

  void beadsPositionsCallback(const std_msgs::msg::String::UniquePtr msg);
  void armStatesCallback(const std_msgs::msg::String::UniquePtr msg);
  void endeffectorStatesCallback(const std_msgs::msg::String::UniquePtr msg);

  void update();
  // void publishString();
  // void publishPoint();
  void init();
  void reset_state();
  void convertBeadstoArmPositions();

protected:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_beads_positions;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_arm_states;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_endeffector_states;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Point>::SharedPtr pub_arm_position;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_move_endeffector;

public:
  explicit SetoScaraRobot2(const std::string & name);
  ~SetoScaraRobot2();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
};

}  // namespace seto_scara

#endif  // SETO_SCARAROBOT2__SETO_SCARAROBOT2_HPP_
