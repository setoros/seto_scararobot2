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

#include "ros/ros.h"
#include "ros/time.h"

#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <geometry_msgs/Point.h>

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <unistd.h> // sleep用

// アームの長さを入れる(単位はビーズを置く位置（beads.x ,beads.y）と揃える)
// 現在はmm

float arm1_length = 90.25;
float arm2_length = 90.25;
float minimam_length = 60.00; // 最小目標距離(中心からの長さ)

float arm1_sita = 0.0;
float arm2_sita = 0.0;
float beads_pos_x = 0.0;
float beads_pos_y = 0.0;


#define DYNAMIXEL_JOINT_NUMBER 4
#define INPUT_JOINT_NUMBER 2

std_msgs::String DXJointName[INPUT_JOINT_NUMBER];
std_msgs::Float64 DXJointPos[INPUT_JOINT_NUMBER];

int step;
enum Step
{
    Waiting,
    Moving,
    Goal
};

int step_EE;
enum Step_EE
{
    Release,
    Grub
};

void beadsCallback(const geometry_msgs::Point &beads)
{
	beads_pos_x = (float)beads.x;
	beads_pos_y = (float)beads.y;
}

void mes_beadsee_Callback(const std_msgs::String::ConstPtr& msg)
{  
  std::string a = "release";
  std::string b = "grub";
  std::string mes;

  mes = msg->data.c_str();
  if (mes == a) step_EE = Release;
  if (mes == b) step_EE = Grub;
}




// コールバックがあるとグローバルに読み込み
void dynamixelreadCallback(const sensor_msgs::JointState::ConstPtr& DynamixelJointState)
{
  int i,j;
  for(i=0;i<INPUT_JOINT_NUMBER;i++){
    for(j=0;j<DYNAMIXEL_JOINT_NUMBER;j++){
      if(DXJointName[i].data == DynamixelJointState->name[j]){
        DXJointPos[i].data = DynamixelJointState->position[j];    // ポジション読み出し
      }
    }
  }

}

void calculate_arm_pos(float x,float y)
{
  float arm1_length_2 = arm1_length * arm1_length;
  float arm2_length_2 = arm2_length * arm2_length;
  float x_2 = x * x;
  float y_2 = y * y;

  // 20191215 Micchy追加
  float length_goal;
  length_goal =  sqrt(x_2 + y_2);
  if ( ((arm1_length + arm2_length) <= length_goal)  || (minimam_length >= length_goal) ){
    ROS_ERROR("[ERROR]:Can't reach the goal.");
    ROS_ERROR("[ERROR]:x|%lf, y|%lf,", x, y );
    return;
	}
  if ( x < 0.0 || y < 0.0){
    ROS_ERROR("[ERROR]:Out of range for x or y.");
    ROS_ERROR("[ERROR]:x|%lf, y|%lf,", x, y );
    return;
	}
  // IK計算
  arm1_sita = atan2(y,x) - acosf( (arm1_length_2 - arm2_length_2 + (x_2 + y_2)) / (2 * arm1_length * length_goal) );
  arm2_sita = M_PI - acosf( (arm1_length_2 + arm2_length_2 - (x_2 + y_2)) / (2 * arm1_length * arm2_length) );
  return;
}

int main(int argc, char **argv)
{

  int flag_wait = 0;
  int flag_move = 0;
  int flag_release = 1; // 初っ端動くと困るのでフラグを立てておく
  int flag_grub = 1;    // 初っ端動くと困るのでフラグを立てておく

  ros::init(argc, argv, "move_arm_dynamixel");  // ノードの初期化
  ros::NodeHandle nh; // ノードハンドラ

  float tmp_beads_pos_x , tmp_beads_pos_y ,old_beads_pos_x=0,old_beads_pos_y=0,old_arm1_sita,old_arm2_sita;

  DXJointName[0].data = "base_to_arm1";
  DXJointName[1].data = "arm1_to_arm2";

  step = Waiting;
  step_EE = Release;

  std_msgs::String ArmStatePub;
  std_msgs::String MES_BEADS_ee_RES;

  //パブリッシャの作成 (目標地点の垂れ流し)
  ros::Publisher pub_scara_arm_goal;
  pub_scara_arm_goal = nh.advertise<sensor_msgs::JointState>("/seto_scararobot/goal",1);

  //パブリッシャの作成 (DynamixelWorkBenchへの指示)
  ros::Publisher pub_scara_arm_trajectory;
  pub_scara_arm_trajectory = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);

  //パブリッシャの作成 (状態)
  ros::Publisher pub_arm_state;
  pub_arm_state = nh.advertise<std_msgs::String>("/arm_states",1);

  //パブリッシャの作成 (エンドエフェクタの状態)
  ros::Publisher pub_mes_beadsee_res;
  pub_mes_beadsee_res = nh.advertise<std_msgs::String>("/mes_beadseeres",1);

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_beads;
  sub_beads = nh.subscribe("/arm_positions", 60, beadsCallback);

  // サブスクライバ(実機の状態)
  ros::Subscriber sub_dynamixelread;
  sub_dynamixelread = nh.subscribe("/dynamixel_workbench/joint_states", 60, dynamixelreadCallback);

  //サブスクライバの作成 (エンドエフェクタの指示)
  ros::Subscriber sub_mes_beadsee;
  sub_mes_beadsee = nh.subscribe("/mes_beadsee", 60, mes_beadsee_Callback);

  ros::Rate loop_rate(100);  // 制御周期60Hz

  // 目標(JointState)を生成
  sensor_msgs::JointState scara_arm;
  scara_arm.name.resize(3);
  scara_arm.name[0] = "base_to_arm1";
  scara_arm.name[1] = "arm1_to_arm2";
  scara_arm.name[2] = "end_joint";
  scara_arm.position.resize(3);
  scara_arm.position[0] = 0.0;
  scara_arm.position[1] = 0.0;
  scara_arm.position[2] = 0.0;

  // ポーズ(JointTrajectory)を生成
  trajectory_msgs::JointTrajectory jtp0;
  jtp0.header.frame_id = "base_link";             // ポーズ名（モーション名)
  jtp0.joint_names.resize(4);                     // 関節名をセット
  jtp0.joint_names[0] ="base_to_arm1";
  jtp0.joint_names[1] ="arm1_to_arm2";
  jtp0.joint_names[2] ="ee1_joint";
  jtp0.joint_names[3] ="ee2_joint";

  jtp0.points.resize(2);                          // ポーズは2つ
  jtp0.points[0].positions.resize(4);             // ポーズ→positionsを2個設定  
  jtp0.points[0].velocities.resize(4);            // ポーズ→velocitiesを2個設定  
  jtp0.points[0].accelerations.resize(4);         // ポーズ→accelerationsを2個設定  
  jtp0.points[0].effort.resize(4);                // ポーズ→effortを2個設定  
  jtp0.points[1].positions.resize(4);             // ポーズ→positionsを2個設定  
  jtp0.points[1].velocities.resize(4);            // ポーズ→velocitiesを2個設定  
  jtp0.points[1].accelerations.resize(4);         // ポーズ→accelerationsを2個設定  
  jtp0.points[1].effort.resize(4);                // ポーズ→effortを2個設定  

  // 原点ポーズをセット
  jtp0.points[0].positions[0] = 0.0;
  jtp0.points[0].positions[1] = 0.0;
  jtp0.points[0].positions[2] = -3.14+0.8;          // 0.8は取り付けオフセット
  jtp0.points[0].positions[3] = 0.0;
  jtp0.points[0].time_from_start = ros::Duration(0.0);  //実行時間0.0sec
  jtp0.points[1].positions[0] = 0.0;
  jtp0.points[1].positions[1] = 0.0;
  jtp0.points[1].positions[2] = -3.14+0.8;
  jtp0.points[1].positions[3] = 0.0;
  jtp0.points[1].time_from_start = ros::Duration(1.0);  //実行時間1.0sec
  old_arm1_sita = 0.0;
  old_arm2_sita = 0.0;

  ROS_INFO("seto_scararobot start!");

  while(ros::ok()){

    // ----変更があったときだけpublish -----------------------
    tmp_beads_pos_x = beads_pos_x;
    tmp_beads_pos_y = beads_pos_y;

    // 前回の目標値と変更があれば・・・
    if ((tmp_beads_pos_x != old_beads_pos_x)||(tmp_beads_pos_y != old_beads_pos_y)){
      jtp0.header.stamp = ros::Time::now();
      scara_arm.header.stamp = ros::Time::now();

      calculate_arm_pos(beads_pos_x,beads_pos_y);
      jtp0.points[0].positions[0] = old_arm1_sita;
      jtp0.points[0].positions[1] = old_arm2_sita;
      jtp0.points[0].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
      jtp0.points[0].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

      jtp0.points[1].positions[0] = arm1_sita;
      jtp0.points[1].positions[1] = arm2_sita;
      jtp0.points[1].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
      jtp0.points[1].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

      scara_arm.position[0] = arm1_sita;
      scara_arm.position[1] = arm2_sita;
      scara_arm.position[2] = 0.0;

      //パブリッシュ (joint_trajectry)
      pub_scara_arm_trajectory.publish(jtp0);
      pub_scara_arm_goal.publish(scara_arm);
      step = Moving;
      flag_move = 0;

      old_beads_pos_x = tmp_beads_pos_x;
      old_beads_pos_y = tmp_beads_pos_y;
      old_arm1_sita = arm1_sita;
      old_arm2_sita = arm2_sita;

    }

    ros::spinOnce();   // コールバック関数を呼ぶ

    switch (step) {
      case Waiting:
        if (flag_wait == 0){
          ArmStatePub.data = "Waiting";
          // 現在の状態をPublish                (1回だけ)
          pub_arm_state.publish(ArmStatePub);
        }
        flag_wait = 1;
        break;
      case Moving:
        flag_wait = 0;

        // Moveingの際、目標と現在地の誤差が1度以下ならゴールとみなす
        if(((abs(DXJointPos[0].data - arm1_sita)) < 0.0174533) && ((abs(DXJointPos[1].data - arm2_sita)) < 0.0174533)){
          step = Goal;
          ArmStatePub.data = "Goal";
          pub_arm_state.publish(ArmStatePub);
          step = Waiting;
          flag_move = 0;
          break;
        }
        if (flag_move == 0){        
          ArmStatePub.data = "Moving";
          // 現在の状態をPublish                (1回だけ)
          pub_arm_state.publish(ArmStatePub);
        }
        flag_move = 1;
        break;
      default:
        break;
    }

  // エンドエフェクタの制御ここから


    switch (step_EE) {
      case Release:
        flag_grub = 0;

        if (flag_release == 0){

          MES_BEADS_ee_RES.data = "Moving";
          pub_mes_beadsee_res.publish(MES_BEADS_ee_RES);// 現在の状態をPublish(1回だけ)
          // 処理を書く

          // 下ろす =================================
          jtp0.points[1].time_from_start = ros::Duration(0.5);  //実行時間0.5sec に変更

          jtp0.points[0].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[0].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[0].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
          jtp0.points[0].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

          jtp0.points[1].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[1].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[1].positions[2] = 0+0.8;          // 0.8は取り付けオフセット
          jtp0.points[1].positions[3] = jtp0.points[1].positions[3];
          pub_scara_arm_trajectory.publish(jtp0);

          usleep(500*1000);  // 0.5秒待ち

          // はなす =================================

           jtp0.points[1].time_from_start = ros::Duration(0.2);  //実行時間0.2sec に変更

          jtp0.points[0].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[0].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[0].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
          jtp0.points[0].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

          jtp0.points[1].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[1].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[1].positions[2] = jtp0.points[1].positions[2];
          jtp0.points[1].positions[3] = 0.0;
          pub_scara_arm_trajectory.publish(jtp0);

          usleep(200*1000);  // 0.2秒待ち

          // あげる =================================

          jtp0.points[1].time_from_start = ros::Duration(0.5);  //実行時間0.5sec に変更

          jtp0.points[0].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[0].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[0].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
          jtp0.points[0].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

          jtp0.points[1].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[1].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[1].positions[2] = -3.14+0.8;          // 0.8は取り付けオフセット
          jtp0.points[1].positions[3] = jtp0.points[1].positions[3];
          pub_scara_arm_trajectory.publish(jtp0);

          usleep(500*1000);  // 0.5秒待ち

          jtp0.points[1].time_from_start = ros::Duration(1.0);  //実行時間1.0sec に戻す

          MES_BEADS_ee_RES.data = "Goal";
          pub_mes_beadsee_res.publish(MES_BEADS_ee_RES);// 現在の状態をPublish(1回だけ)          
          MES_BEADS_ee_RES.data = "Waiting";
          pub_mes_beadsee_res.publish(MES_BEADS_ee_RES);// 現在の状態をPublish(1回だけ)
          flag_release = 1;

        }


        break;



      case Grub:
        flag_release = 0;

        if (flag_grub == 0){        
          MES_BEADS_ee_RES.data = "Moving";
          pub_mes_beadsee_res.publish(MES_BEADS_ee_RES);// 現在の状態をPublish(1回だけ)

          // 処理を書く
          usleep(500*1000);   // 0.5秒静止

          // おろす =================================
          jtp0.points[1].time_from_start = ros::Duration(0.5);  //実行時間0.5sec に変更

          jtp0.points[0].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[0].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[0].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
          jtp0.points[0].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

          jtp0.points[1].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[1].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[1].positions[2] = 0+0.8;          // 0.8は取り付けオフセット
          jtp0.points[1].positions[3] = jtp0.points[1].positions[3];
          pub_scara_arm_trajectory.publish(jtp0);

          usleep(500*1000);  // 0.5秒待ち

          // つかむ =================================
          jtp0.points[0].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[0].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[0].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
          jtp0.points[0].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

          jtp0.points[1].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[1].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[1].positions[2] = jtp0.points[1].positions[2];
          jtp0.points[1].positions[3] = -0.5;
          pub_scara_arm_trajectory.publish(jtp0);

          usleep(500*1000);  // 0.5秒待ち


          // グリグリする ==========================

          jtp0.points[1].time_from_start = ros::Duration(0.2);  //実行時間0.2sec に変更

          jtp0.points[1].positions[1] = jtp0.points[1].positions[1] - 0.02;    // 現在の値から動かさない
          pub_scara_arm_trajectory.publish(jtp0);
          usleep(200*1000);  // 0.2秒待ち

          jtp0.points[1].positions[1] = jtp0.points[1].positions[1] + 0.04;    // 現在の値から動かさない
          pub_scara_arm_trajectory.publish(jtp0);
          usleep(200*1000);  // 0.2秒待ち

          jtp0.points[1].positions[1] = jtp0.points[1].positions[1] - 0.04;    // 現在の値から動かさない
          pub_scara_arm_trajectory.publish(jtp0);
          usleep(200*1000);  // 0.2秒待ち

          jtp0.points[1].positions[1] = jtp0.points[1].positions[1] + 0.04;    // 現在の値から動かさない
          pub_scara_arm_trajectory.publish(jtp0);
          usleep(200*1000);  // 0.2秒待ち

          jtp0.points[1].positions[1] = jtp0.points[1].positions[1] - 0.02;    // 現在の値から動かさない

          jtp0.points[1].time_from_start = ros::Duration(0.5);  //実行時間0.5sec に変更

          // あげる =================================
          jtp0.points[0].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[0].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[0].positions[2] = jtp0.points[1].positions[2];    // 現在の値から動かさない
          jtp0.points[0].positions[3] = jtp0.points[1].positions[3];    // 現在の値から動かさない

          jtp0.points[1].positions[0] = jtp0.points[1].positions[0];    // 現在の値から動かさない
          jtp0.points[1].positions[1] = jtp0.points[1].positions[1];    // 現在の値から動かさない
          jtp0.points[1].positions[2] = -3.14+0.8;          // 0.8は取り付けオフセット
          jtp0.points[1].positions[3] = jtp0.points[1].positions[3];
          pub_scara_arm_trajectory.publish(jtp0);

          usleep(500*1000);  // 0.5秒待ち

          jtp0.points[1].time_from_start = ros::Duration(1.0);  //実行時間1.0sec に戻す

          MES_BEADS_ee_RES.data = "Goal";
          pub_mes_beadsee_res.publish(MES_BEADS_ee_RES);// 現在の状態をPublish(1回だけ)          
          MES_BEADS_ee_RES.data = "Waiting";
          pub_mes_beadsee_res.publish(MES_BEADS_ee_RES);// 現在の状態をPublish(1回だけ)

          flag_grub = 1;
        }

        break;
      default:
        break;
    }
    
    loop_rate.sleep();
  }
  return 0;
}


