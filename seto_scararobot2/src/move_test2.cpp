// ２号機への移植：対応済み 20210712

// 適当なjoint_stateを吐き出し続けるテストプログラム
// rviz上でロボットアームがクルクル回る
//
// move_test1
//

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