// ２号機への移植：未着手

// 20191212 kawata氏作成
// 20191215 Micchy修正
// Pointをsubscriveして、joint_trajectryをpublish
// 20200317 Micchy 逆運動学でバグ(x<0,y>0の領域)があったので修正

#include "ros/ros.h"
#include "ros/time.h"

#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <geometry_msgs/Point.h>

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <math.h>

// アームの長さを入れる(単位はビーズを置く位置（beads.x ,beads.y）と揃える)
// 現在はmm

float arm1_length = 90.25;
float arm2_length = 90.25;

float arm1_sita = 0.0;
float arm2_sita = 0.0;
float beads_pos_x = 0.0;
float beads_pos_y = 0.0;

// 現在の状態を保存する変数
std_msgs::String joint_name[2];
std_msgs::Float64 joint_pos[2];
std_msgs::Float64 joint_vel[2];
std_msgs::Float64 joint_eff[2];

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

void beadsCallback(const geometry_msgs::Point &beads)
{
	beads_pos_x = (float)beads.x;
	beads_pos_y = (float)beads.y;
    //ROS_INFO("%lf",beads_pos_x);
    //ROS_INFO("%lf",beads_pos_y);
}

// コールバックがあるとグローバルに読み込み
void dynamixelreadCallback(const sensor_msgs::JointState::ConstPtr& DynamixelJointState)
{
  int i,j;
  for(i=0;i<INPUT_JOINT_NUMBER;i++){
    for(j=0;j<INPUT_JOINT_NUMBER;j++){
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
//  float x = beads_pos_x;
//  float y = beads_pos_y;
  float x_2 = x * x;
  float y_2 = y * y;

  // 20191215 Micchy追加
  float length_goal;
  length_goal =  sqrt(x_2 + y_2);

	if ( ((arm1_length + arm2_length) <= length_goal)  || (length_goal == 0.0)){
    arm1_sita = 0.0;
    arm2_sita = 0.0;
    ROS_INFO("arm1_sita is %lf",arm1_sita);
    ROS_INFO("arm2_sita is %lf",arm2_sita);
    return;
	}

  // 20191215 Micchy追加
  // 20200317 更新
  if (y > 0){
    arm1_sita = atan2(y,x) - acosf((length_goal/2) / arm1_length);
    arm2_sita = -2 * (asinf((length_goal/2) / arm1_length));
  }
  else{
    arm1_sita = atan2(y,x) + acosf((length_goal/2) / arm1_length);
    arm2_sita = 2 * (asinf((length_goal/2) / arm1_length));
  }

  ROS_INFO("arm1_sita is %lf",arm1_sita);
  ROS_INFO("arm2_sita is %lf",arm2_sita);
  return;
}

int main(int argc, char **argv)
{

  int flag_wait = 0;
  int flag_move = 0;


  ros::init(argc, argv, "move_arm_dynamixel");  // ノードの初期化
  ros::NodeHandle nh; // ノードハンドラ

  float tmp_beads_pos_x , tmp_beads_pos_y ,old_beads_pos_x=0,old_beads_pos_y=0,old_arm1_sita,old_arm2_sita;

  DXJointName[0].data = "base_to_arm1";
  DXJointName[1].data = "arm1_to_arm2";

  step = Waiting;

  std_msgs::String ArmStatePub;

  //パブリッシャの作成 (目標地点の垂れ流し)
  ros::Publisher pub_scara_arm_goal;
  pub_scara_arm_goal = nh.advertise<sensor_msgs::JointState>("/seto_scararobot/goal",1);

  //パブリッシャの作成 (DynamixelWorkBenchへの指示)
  ros::Publisher pub_scara_arm_trajectory;
  pub_scara_arm_trajectory = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);

  //パブリッシャの作成 (状態)
  ros::Publisher pub_arm_state;
  pub_arm_state = nh.advertise<std_msgs::String>("/arm_states",1);

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_beads;
  sub_beads = nh.subscribe("/arm_positions", 60, beadsCallback);

  // サブスクライバ(実機の状態)
  ros::Subscriber sub_dynamixelread;
  sub_dynamixelread = nh.subscribe("/dynamixel_workbench/joint_states", 60, dynamixelreadCallback);

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
  jtp0.joint_names.resize(2);                     // 関節名をセット
  jtp0.joint_names[0] ="base_to_arm1";
  jtp0.joint_names[1] ="arm1_to_arm2";
  jtp0.points.resize(2);                          // ポーズは2つ
  jtp0.points[0].positions.resize(2);             // ポーズ→positionsを2個設定  
  jtp0.points[0].velocities.resize(2);            // ポーズ→velocitiesを2個設定  
  jtp0.points[0].accelerations.resize(2);         // ポーズ→accelerationsを2個設定  
  jtp0.points[0].effort.resize(2);                // ポーズ→effortを2個設定  
  jtp0.points[1].positions.resize(2);             // ポーズ→positionsを2個設定  
  jtp0.points[1].velocities.resize(2);            // ポーズ→velocitiesを2個設定  
  jtp0.points[1].accelerations.resize(2);         // ポーズ→accelerationsを2個設定  
  jtp0.points[1].effort.resize(2);                // ポーズ→effortを2個設定  

  // 原点ポーズをセット
  jtp0.points[0].positions[0] = 0.0;
  jtp0.points[0].positions[1] = 0.0;
  jtp0.points[0].time_from_start = ros::Duration(0.0);  //実行時間0.0sec
  jtp0.points[1].positions[0] = 0.0;
  jtp0.points[1].positions[1] = 0.0;
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
      jtp0.points[1].positions[0] = arm1_sita;
      jtp0.points[1].positions[1] = arm2_sita;

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
    
    loop_rate.sleep();
  }
  return 0;
}


