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

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_test");  // ノードの初期化
  ros::NodeHandle nh; // ノードハンドラ  

  //パブリッシャの作成
  ros::Publisher scara_arm_pub;
  scara_arm_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);

  ros::Rate loop_rate(60);  // 制御周期60Hz

  int count=0;

  while(ros::ok()) {

    sensor_msgs::JointState scara_arm;
    scara_arm.header.stamp = ros::Time::now();

    scara_arm.name.resize(3);
    scara_arm.name[0] = "base_to_arm1";
    scara_arm.name[1] = "arm1_to_arm2";
    scara_arm.name[2] = "end_joint";

    scara_arm.position.resize(3);
    //scara_arm.position[0] = -1.0*(float)count/40.0;
    scara_arm.position[0] = ((3.141592/2) * sin((float(count % 314) / 50.0)));// + (3.141592/4);

    //scara_arm.position[1] = 2.0*(float)count/40.0;
    scara_arm.position[1] = ((3.141592/3) * sin((float(count % 157) / 25.0))) + (3.141592/3);

    scara_arm.position[2] = 0.0;

    count++;

    scara_arm_pub.publish(scara_arm);
    ros::spinOnce();   // コールバック関数を呼ぶ
    loop_rate.sleep();
  }

  return 0;
}