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

#include "seto_scararobot2/seto_scararobot2.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include <chrono>
#include <memory>
#include <string>

namespace seto_scara
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

SetoScaraRobot2::SetoScaraRobot2(const std::string & name)
: LifecycleNode(name),
  ros_clock(RCL_ROS_TIME)
 {}
SetoScaraRobot2::~SetoScaraRobot2() {}

CallbackReturn SetoScaraRobot2::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::chrono_literals;
  
  step = Initialize;

  is_arm_waiting = false;
  is_arm_moving = false;
  is_arm_goal = false;

  is_endeffector_waiting = false;
  is_endeffector_moving = false;
  is_endeffector_goal = false;

  // Setup subscribers
  sub_beads_positions = create_subscription<std_msgs::msg::String>(
    "beads_positions", 100, std::bind(&SetoScaraRobot2::beadsPositionsCallback, this, std::placeholders::_1));
  sub_arm_states = create_subscription<std_msgs::msg::String>(
    "arm_states", 100, std::bind(&SetoScaraRobot2::armStatesCallback, this, std::placeholders::_1));
  sub_endeffector_states = create_subscription<std_msgs::msg::String>(
    "mes_beadseeres", 100, std::bind(&SetoScaraRobot2::endeffectorStatesCallback, this, std::placeholders::_1));

  // Setup publishers
  pub_arm_position = create_publisher<geometry_msgs::msg::Point>("arm_positions", 100);
  pub_move_endeffector = create_publisher<std_msgs::msg::String>("mes_beadsee", 100);

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  timer = create_wall_timer(100ms, std::bind(&SetoScaraRobot2::update, this));
  timer->cancel();

  RCLCPP_INFO(get_logger(), "Ready");

  return CallbackReturn::SUCCESS;
}

CallbackReturn SetoScaraRobot2::on_activate(const rclcpp_lifecycle::State &)
{
  pub_arm_position->on_activate();
  pub_move_endeffector->on_activate();
  timer->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn SetoScaraRobot2::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer->cancel();

  pub_arm_position->on_deactivate();
  pub_move_endeffector->on_deactivate();

  RCLCPP_INFO(get_logger(), "Connection terminated.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn SetoScaraRobot2::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer.reset();

  pub_arm_position.reset();
  pub_move_endeffector.reset();

  sub_beads_positions.reset();
  sub_arm_states.reset();
  sub_endeffector_states.reset();

  RCLCPP_INFO(get_logger(), "Reset");

  return CallbackReturn::SUCCESS;
}

void SetoScaraRobot2::beadsPositionsCallback(const std_msgs::msg::String::UniquePtr msg)
{
  received_beads_positions = *msg;
  // RCLCPP_INFO(get_logger(), "Received beadsPositions");
}

void SetoScaraRobot2::armStatesCallback(const std_msgs::msg::String::UniquePtr msg)
{
  received_arm_states = *msg;
  // RCLCPP_INFO(get_logger(), "Received armStates");
  if(received_arm_states.data == MSG_ARM_GOAL)
  {
    is_arm_goal = true;
    is_arm_waiting = true;
    is_arm_moving = false;
  }
  else if(received_arm_states.data == MSG_ARM_WAITING)
  {
    is_arm_waiting = true;
    is_arm_moving = false;
  }
  else if(received_arm_states.data == MSG_ARM_MOVING)
  {
    is_arm_waiting = false;
    is_arm_moving = true;
  }
}

void SetoScaraRobot2::endeffectorStatesCallback(const std_msgs::msg::String::UniquePtr msg)
{
  received_endeffector_states = *msg;
  RCLCPP_INFO(get_logger(), "Received endeffectorStates");
  if(received_endeffector_states.data == MSG_ENDEFFECTOR_GOAL)
  {
    is_endeffector_goal = true;
    is_endeffector_waiting = true;
    is_endeffector_moving = false;
  }
  else if(received_endeffector_states.data == MSG_ENDEFFECTOR_WAITING)
  {
    is_endeffector_waiting = true;
    is_endeffector_moving = false;
  }
  else if(received_endeffector_states.data == MSG_ENDEFFECTOR_MOVING)
  {
    is_endeffector_waiting = false;
    is_endeffector_moving = true;
  }
}

void SetoScaraRobot2::update()
{
  switch(step)
  {
    case Initialize:
    {
      init();
      step++;
      RCLCPP_INFO(get_logger(), "WaitBeadsInfo");
      break;
    }

    case WaitBeadsInfo:
    {
      if(received_beads_positions.data != "")
      {
        step++;
        convertBeadstoArmPositions();
        RCLCPP_INFO(get_logger(), "SearchSetBeads");
      }
      break;
    }

    case SearchSetBeads:
    {
      while(arm_positions[set_count] == SPACE)
      {
        if(set_count <= (int)(arm_positions.size() - 1))
        {
          set_count++;
        }
        else
        {
          break;
        }
      }
      if(set_count > (int)(arm_positions.size() - 1))
      {
        RCLCPP_INFO(get_logger(), "CheckFinishTask");
        
        step = CheckFinishTask;
      }
      else
      {
        RCLCPP_INFO(get_logger(), "CatchBeads");
        step++;
      }
      break;
    }

    case CatchBeads:
    {
      if(arm_positions[set_count] == COLOR1)
      {
        arm_position.x = -color_first[color_first.size() - 1][0] * DISTANCE;
        arm_position.y = -color_first[color_first.size() - 1][1] * DISTANCE;
        color_first.pop_back();
        RCLCPP_INFO(get_logger(), "arm_position.x = %lf arm_position.y = %lf", arm_position.x, arm_position.y);
      }
      else if(arm_positions[set_count] == COLOR2)
      {
        arm_position.x = -color_second[color_second.size() - 1][0] * DISTANCE;
        arm_position.y = -color_second[color_second.size() - 1][1] * DISTANCE;
        color_second.pop_back();
        // RCLCPP_INFO(get_logger(), "arm_position.x = %lf arm_position.y = %lf", arm_position.x, arm_position.y);
      }
      else if(arm_positions[set_count] == COLOR3)
      {
        arm_position.x = -color_third[color_third.size() - 1][0] * DISTANCE;
        arm_position.y = -color_third[color_third.size() - 1][1] * DISTANCE;
        color_third.pop_back();
        // RCLCPP_INFO(get_logger(), "arm_position.x = %lf arm_position.y = %lf", arm_position.x, arm_position.y);
      }
      pub_arm_position->publish(arm_position);
      step++;
      RCLCPP_INFO(get_logger(), "WaitCatchBeads");
      break;
    }

    case WaitCatchBeads:
    {
      if(is_arm_goal == true)
      {
        step++;
        RCLCPP_INFO(get_logger(), "EndEffectorforCatch");
      }
      break;
    }

    case EndEffectorforCatch:
    {
      //pub_move_endeffector.publish("Grasp");
      string_msg.data=MSG_ENDEFFECTOR_GRUB;
      pub_move_endeffector->publish(string_msg);
      step++;
      RCLCPP_INFO(get_logger(), "WaitEndEffectorforCatch");
      break;
    }

    case WaitEndEffectorforCatch:
    {
      if(is_endeffector_goal == true)
      {
        reset_state();
        step++;
        RCLCPP_INFO(get_logger(), "SetBeads");
      }
      break;
    }

    case SetBeads:
    {
      arm_position.x = (int)(set_count % (int)sqrt(arm_positions.size())) * DISTANCE + OFFSET_X;
      arm_position.y = (int)(set_count / (int)sqrt(arm_positions.size())) * DISTANCE + OFFSET_Y;
      pub_arm_position->publish(arm_position);
      RCLCPP_INFO(get_logger(), "arm_position.x = %lf arm_position.y = %lf", arm_position.x, arm_position.y);
      step++;
      RCLCPP_INFO(get_logger(), "WaitSetBeads");
      break;
    }

    case WaitSetBeads:
    {
      if(is_arm_goal == true)
      {
        step++;
        RCLCPP_INFO(get_logger(), "EndEffectorforSet");
      }
      break;
    }

    case EndEffectorforSet:
    {
      string_msg.data=MSG_ENDEFFECTOR_RELEASE;
      pub_move_endeffector->publish(string_msg);
      step++;
      RCLCPP_INFO(get_logger(), "WaitEndEffectorforSet");
      break;
    }

    case WaitEndEffectorforSet:
    {
      if(is_endeffector_goal == true)
      {
        step++;
        RCLCPP_INFO(get_logger(), "CheckFinishTask");
      }
      break;
    }
    
    case CheckFinishTask:
    {
      set_count++;
      // ROS_INFO("position:%d", arm_positions.size());
      // ROS_INFO("count:%d", set_count);
      if(set_count >= (int)arm_positions.size())
      {
        step = Initialize;
      }
      else
      {
        reset_state();
        step = SearchSetBeads;
      }
      break;
    }
  }
}

void SetoScaraRobot2::init()
{
  step = Initialize;
  is_arm_waiting = true;
  is_arm_moving = false;
  is_arm_goal = false;
  is_endeffector_waiting = true;
  is_endeffector_moving = false;
  is_endeffector_goal = false;
  received_beads_positions.data = "";
  received_arm_states.data = "";
  received_endeffector_states.data = "";
  move_endeffector.data = "";
  string_msg.data = "";
  arm_positions = {};
  set_count = 0;
}

void SetoScaraRobot2::reset_state()
{
  is_arm_waiting = true;
  is_arm_moving = false;
  is_arm_goal = false;
  is_endeffector_waiting = true;
  is_endeffector_moving = false;
  is_endeffector_goal = false;
}

void SetoScaraRobot2::convertBeadstoArmPositions()
  {
    for (int i = 0; i < (int)received_beads_positions.data.size(); i++)
    {
      if (std::isalpha(static_cast<unsigned char>(received_beads_positions.data.c_str()[i])))
      {
        //アルファベットを１０以上の数字に置き換える処理（未実装）
      }
      else
      {
        arm_positions.push_back(int(received_beads_positions.data.c_str()[i] - '0'));
      }
    }
  }
}  // namespace seto_scara

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<seto_scara::SetoScaraRobot2>("seto_scararobot2");
  // std::cout << "start" << std::endl;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}