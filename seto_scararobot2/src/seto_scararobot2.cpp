// ２号機への移植：未着手

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <cctype>
#include <math.h>

class SetoScaraRobot
{
private:
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
  int step_; //スカラロボットの状態変数
  std::string received_beads_positions; //受け取ったビーズ情報
  std::string received_arm_states; //受け取ったアームの状態
  std::vector<int> arm_positions; // 受け取ったビーズの位置をアームの座標系に変換したものを保存
  geometry_msgs::Point arm_position; //送信用のアーム情報

  std::string move_endeffector;
  std::string received_endeffector_states; //受け取ったエンドエフェクタの状態

  std_msgs::String send_msg;

  //アームの状態変数
  bool is_arm_waiting_;
  bool is_arm_moving_;
  bool is_arm_goal_;

  bool is_endeffector_waiting_;
  bool is_endeffector_moving_;
  bool is_endeffector_goal_;
  
  //色番号
  //beads_setterのカラーコードと一致させておく
  const int SPACE = 0;
  const int COLOR1 = 1;
  const int COLOR2 = 2;
  const int COLOR3 = 3;
  
  //行番号
  const int COLOR1_ROW = 10; 
  const int COLOR2_ROW = 11;
  const int COLOR3_ROW = 12;

  //ビーズの個数
  const int COLOR1_NUM = 29; 
  const int COLOR2_NUM = 29;
  const int COLOR3_NUM = 29;

  //オフセット位置（-29〜29)
  const int OFFSET_X = 1;
  const int OFFSET_Y = 10;

  const double DISTANCE = 5; //ビーズ間の距離（mm）

  int set_count;

  //ビーズの個数
  int beads_color1_num = COLOR1_NUM;
  int beads_color2_num = COLOR2_NUM;
  int beads_color3_num = COLOR3_NUM;


  ros::Publisher  pub_arm_position;
  ros::Publisher  pub_move_endeffector;

  ros::Subscriber  sub_beads_positions;
  ros::Subscriber  sub_arm_states;
  ros::Subscriber  sub_endeffector_states;

  void init()
  {
    reset();
  }

  void reset()
  {
    step_ = Initialize;
    is_arm_waiting_ = true;
    is_arm_moving_ = false;
    is_arm_goal_ = false;
    is_endeffector_waiting_ = true;
    is_endeffector_moving_ = false;
    is_endeffector_goal_ = false;
    received_beads_positions = "";
    received_arm_states = "";
    received_endeffector_states = "";
    move_endeffector = "";
    set_count = 0;
    beads_color1_num = COLOR1_NUM;
    beads_color2_num = COLOR2_NUM;
    beads_color3_num = COLOR3_NUM;
    send_msg.data = "";
    arm_positions = {};
  }

  void reset_state()
  {
    is_arm_waiting_ = true;
    is_arm_moving_ = false;
    is_arm_goal_ = false;
    is_endeffector_waiting_ = true;
    is_endeffector_moving_ = false;
    is_endeffector_goal_ = false;
  }

  void beadsPositionsCallback(const std_msgs::String::ConstPtr &beads_positions)
  {
    received_beads_positions = beads_positions->data;
    //ROS_INFO("beads_positions:%s", received_beads_positions.c_str());
  }

  void armStatesCallback(const std_msgs::String::ConstPtr &arm_states)
  {
    received_arm_states = arm_states->data;
    //ROS_INFO("received_arm_states:%s", received_arm_states.c_str());
    if(received_arm_states == MSG_ARM_GOAL)
    {
      is_arm_goal_ = true;
      is_arm_waiting_ = true;
      is_arm_moving_ = false;
    }
    else if(received_arm_states == MSG_ARM_WAITING)
    {
      is_arm_waiting_ = true;
      is_arm_moving_ = false;
    }
    else if(received_arm_states == MSG_ARM_MOVING)
    {
      is_arm_waiting_ = false;
      is_arm_moving_ = true;
    }
  }

  void endeffectorStatesCallback(const std_msgs::String::ConstPtr &endeffector_states)
  {
    received_endeffector_states = endeffector_states->data;
    //ROS_INFO("received_endeffector_states:%s", received_endeffector_states.c_str());
      if(received_endeffector_states == MSG_ENDEFFECTOR_GOAL)
    {
      is_endeffector_goal_ = true;
      is_endeffector_waiting_ = true;
      is_endeffector_moving_ = false;
    }
    else if(received_endeffector_states == MSG_ENDEFFECTOR_WAITING)
    {
      is_endeffector_waiting_ = true;
      is_endeffector_moving_ = false;
    }
    else if(received_endeffector_states == MSG_ENDEFFECTOR_MOVING)
    {
      is_endeffector_waiting_ = false;
      is_endeffector_moving_ = true;
    }
  }

  void convertBeadstoArmPositions()
  {
    for (int i = 0; i < (int)received_beads_positions.size(); i++)
    {
      if (std::isalpha(static_cast<unsigned char>(received_beads_positions.c_str()[i])))
      {
        //アルファベットを１０位上の数字に置き換える処理
      }
      else
      {
        arm_positions.push_back(int(received_beads_positions.c_str()[i] - '0'));
      }
      //ROS_INFO("arm_positions:%d", arm_positions[i]);
    }
  }

public:
  int run(int argc, char **argv)
  {
    ros::NodeHandle node_handle; //ノードハンドラ
    ros::Rate loop_rate(10); //制御周期10Hz

    //パブリッシャ＆サブスクライバの設定
    std::string pub_arm_position_topic_name;
    std::string pub_move_endeffector_topic_name;

    std::string sub_beads_positions_topic_name;
    std::string sub_arm_states_topic_name;
    std::string sub_endeffector_states_topic_name;

    node_handle.param<std::string>("pub_arm_position_topic_name", pub_arm_position_topic_name, "/arm_positions");
    node_handle.param<std::string>("pub_move_endeffector_topic_name", pub_move_endeffector_topic_name, "/mes_beadsee");

    node_handle.param<std::string>("sub_beads_positions_topic_name", sub_beads_positions_topic_name, "/beads_positions");
    node_handle.param<std::string>("sub_arm_states_topic_name", sub_arm_states_topic_name, "/arm_states");
    node_handle.param<std::string>("sub_endeffector_states_topic_name", sub_endeffector_states_topic_name, "/mes_beadseeres");

    pub_arm_position = node_handle.advertise<geometry_msgs::Point>(pub_arm_position_topic_name, 60);
    pub_move_endeffector = node_handle.advertise<std_msgs::String>(pub_move_endeffector_topic_name, 60);

    sub_beads_positions = node_handle.subscribe(sub_beads_positions_topic_name, 60, &SetoScaraRobot::beadsPositionsCallback, this);
    sub_arm_states = node_handle.subscribe(sub_arm_states_topic_name, 60, &SetoScaraRobot::armStatesCallback, this);
    sub_endeffector_states = node_handle.subscribe(sub_endeffector_states_topic_name, 60, &SetoScaraRobot::endeffectorStatesCallback, this);

    init();

    ROS_INFO("seto_scararobot start!");

    while (ros::ok())
    {
      switch(step_)
      {
        case Initialize:
        {
          reset();
          step_++;
          ROS_INFO("WaitBeadsInfo");
          break;
        }

        case WaitBeadsInfo:
        {
          if(received_beads_positions != "")
          {
            step_++;
            convertBeadstoArmPositions();
            ROS_INFO("CatchBeads");
          }
          break;
        }

        case SearchSetBeads:
        {
          while(arm_positions[set_count] == SPACE)
          {
            if(set_count <= (arm_positions.size() - 1))
            {
              set_count++;
            }
            else
            {
              break;
            }
          }
          if(set_count > (arm_positions.size() - 1))
          {
            ROS_INFO("CheckFinishTask");
            step_ = CheckFinishTask;
          }
          else
          {
            ROS_INFO("CatchBeads");
            step_++;
            //step_=SetBeads;
          }
          break;
        }

        case CatchBeads:
        {

          if(arm_positions[set_count] == COLOR1)
          {
            arm_position.x = (COLOR1_NUM - beads_color1_num + OFFSET_X)* DISTANCE;
            arm_position.y = (COLOR1_ROW - 1 + OFFSET_Y) * DISTANCE;
            beads_color1_num--;
          }
          else if(arm_positions[set_count] == COLOR2)
          {
            arm_position.x = (COLOR2_NUM - beads_color2_num + OFFSET_X)* DISTANCE;
            arm_position.y = (COLOR2_ROW - 1 + OFFSET_Y) * DISTANCE;
            beads_color2_num--;
          }
          else if(arm_positions[set_count] == COLOR3)
          {
            arm_position.x = (COLOR3_NUM - beads_color3_num + OFFSET_X)* DISTANCE;
            arm_position.y = (COLOR3_ROW - 1 + OFFSET_Y) * DISTANCE;
            beads_color3_num--;
          }
          pub_arm_position.publish(arm_position);
          step_++;
          ROS_INFO("WaitCatchBeads");
          break;
        }

        case WaitCatchBeads:
        {
          if(is_arm_goal_ == true)
          {
            step_++;
            ROS_INFO("EndEffectorforCatch");
          }
          break;
        }

        case EndEffectorforCatch:
        {
          //pub_move_endeffector.publish("Grasp");
          send_msg.data=MSG_ENDEFFECTOR_GRUB;
          pub_move_endeffector.publish(send_msg);
          step_++;
          ROS_INFO("WaitEndEffectorforCatch");
          break;
        }

        case WaitEndEffectorforCatch:
        {
          if(is_endeffector_goal_ == true)
          {
            reset_state();
            step_++;
            ROS_INFO("SetBeads");
          }
          break;
        }

        case SetBeads:
        {
          arm_position.x = ((int)(set_count % (int)sqrt(arm_positions.size())) + OFFSET_X) * DISTANCE;
          arm_position.y = ((int)(set_count / (int)sqrt(arm_positions.size())) + OFFSET_Y) * DISTANCE;
          //arm_position.x = ((int)(set_count % 29) + OFFSET_X)* DISTANCE;
          //arm_position.y = ((int)(set_count / 29) + OFFSET_Y) * DISTANCE;
          pub_arm_position.publish(arm_position);
          step_++;
          // デバッグ用かな、先に進めなかったのでコメントアウト
          // step_=CheckFinishTask;
          ROS_INFO("WaitSetBeads");
          break;
        }

        case WaitSetBeads:
        {
          if(is_arm_goal_ == true)
          {
            step_++;
            ROS_INFO("EndEffectorforSet");
          }
          break;
        }

        case EndEffectorforSet:
        {
          send_msg.data=MSG_ENDEFFECTOR_RELEASE;
          pub_move_endeffector.publish(send_msg);
          step_++;
          ROS_INFO("WaitEndEffectorforSet");
          break;
        }

        case WaitEndEffectorforSet:
        {
          if(is_endeffector_goal_ == true)
          {
            step_++;
            ROS_INFO("CheckFinishTask");
          }
          break;
        }
        
        case CheckFinishTask:
        {
          set_count++;
          ROS_INFO("position:%d", arm_positions.size());
          ROS_INFO("count:%d", set_count);
          if(set_count >= arm_positions.size())
          {
            step_ = Initialize;
          }
          else
          {
            reset_state();
            step_ = SearchSetBeads;
          }
          break;
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
  }
};

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "seto_scararobot"); //ノードの初期化
    SetoScaraRobot seto_scararobot; //コンストラクタ
    return seto_scararobot.run(argc, argv);
};