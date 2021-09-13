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

#include <geometry_msgs/Point.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <cctype>
#include <vector>

class SetoScaraRobot {
   private:
    enum Step {
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

    const std::string MSG_ARM_WAITING = "Waiting";
    const std::string MSG_ARM_MOVING = "Moving";
    const std::string MSG_ARM_GOAL = "Goal";
    const std::string MSG_ENDEFFECTOR_WAITING = "Waiting";
    const std::string MSG_ENDEFFECTOR_MOVING = "Moving";
    const std::string MSG_ENDEFFECTOR_GOAL = "Goal";
    const std::string MSG_ENDEFFECTOR_GRUB = "grub";
    const std::string MSG_ENDEFFECTOR_RELEASE = "release";
    int step_;  //スカラロボットの状態変数
    std::string received_beads_positions;  //受け取ったビーズ情報
    std::string received_arm_states;       //受け取ったアームの状態
    std::vector<int>
        arm_positions;  // 受け取ったビーズの位置をアームの座標系に変換したものを保存
    geometry_msgs::Point arm_position;  //送信用のアーム情報

    std::string move_endeffector;
    std::string received_endeffector_states;  //受け取ったエンドエフェクタの状態

    std_msgs::String send_msg;

    //アームの状態変数
    bool is_arm_waiting_;
    bool is_arm_moving_;
    bool is_arm_goal_;

    bool is_endeffector_waiting_;
    bool is_endeffector_moving_;
    bool is_endeffector_goal_;

    //色番号
    // beads_setterのカラーコードと一致させておく
    const int SPACE = 0;
    const int COLOR1 = 1;
    const int COLOR2 = 2;
    const int COLOR3 = 3;

    std::vector<std::vector<int>> color_first = {
        {0, 8},  {1, 8},  {2, 8},  {0, 9},  {1, 9},  {2, 9},  {0, 10}, {1, 10},
        {2, 10}, {0, 11}, {1, 11}, {2, 11}, {0, 12}, {1, 12}, {2, 12}, {0, 13},
        {1, 13}, {2, 13}, {0, 14}, {1, 14}, {2, 14}, {0, 15}, {1, 15}, {2, 15},
        {0, 16}, {1, 16}, {2, 16}, {0, 17}, {1, 17}, {2, 17}, {0, 18}, {1, 18},
        {2, 18}, {0, 19}, {1, 19}, {2, 19}, {0, 20}, {1, 20}, {2, 20}, {0, 21},
        {1, 21}, {2, 21}, {0, 22}, {1, 22}, {2, 22}, {0, 23}, {1, 23}, {2, 23},
        {0, 24}, {1, 24}, {2, 24}, {0, 25}, {1, 25}, {2, 25}, {0, 26}, {1, 26},
        {2, 26}, {0, 27}, {1, 27}, {2, 27}, {0, 28}, {1, 28}, {2, 28},
    };

    std::vector<std::vector<int>> color_second = {
        {3, 8},  {4, 8},  {5, 8},  {3, 9},  {4, 9},  {5, 9},  {3, 10}, {4, 10},
        {5, 10}, {3, 11}, {4, 11}, {5, 11}, {3, 12}, {4, 12}, {5, 12}, {3, 13},
        {4, 13}, {5, 13}, {3, 14}, {4, 14}, {5, 14}, {3, 15}, {4, 15}, {5, 15},
        {3, 16}, {4, 16}, {5, 16}, {3, 17}, {4, 17}, {5, 17}, {3, 18}, {4, 18},
        {5, 18}, {3, 19}, {4, 19}, {5, 19}, {3, 20}, {4, 20}, {5, 20}, {3, 21},
        {4, 21}, {5, 21}, {3, 22}, {4, 22}, {5, 22}, {3, 23}, {4, 23}, {5, 23},
        {3, 24}, {4, 24}, {5, 24}, {3, 25}, {4, 25}, {5, 25}, {3, 26}, {4, 26},
        {5, 26}, {3, 27}, {4, 27}, {5, 27}, {3, 28}, {4, 28}, {5, 28},
    };

    std::vector<std::vector<int>> color_third = {
        {6, 8},  {7, 8},  {8, 8},  {6, 9},  {7, 9},  {8, 9},  {6, 10}, {7, 10},
        {8, 10}, {6, 11}, {7, 11}, {8, 11}, {6, 12}, {7, 12}, {8, 12}, {6, 13},
        {7, 13}, {8, 13}, {6, 14}, {7, 14}, {8, 14}, {6, 18}, {7, 15}, {8, 15},
        {6, 16}, {7, 16}, {8, 16}, {6, 17}, {7, 17}, {8, 17}, {6, 18}, {7, 18},
        {8, 18}, {6, 19}, {7, 19}, {8, 19}, {6, 20}, {7, 20}, {8, 20}, {6, 21},
        {7, 21}, {8, 21}, {6, 22}, {7, 22}, {8, 22}, {6, 23}, {7, 23}, {8, 23},
        {6, 24}, {7, 24}, {8, 24}, {6, 25}, {7, 25}, {8, 25}, {6, 26}, {7, 26},
        {8, 26}, {6, 27}, {7, 27}, {8, 27}, {6, 28}, {7, 28}, {8, 28},
    };

    //オフセット位置(mm)
    const int OFFSET_X = 20;
    const int OFFSET_Y = 20;

    const double DISTANCE = 5;  //ビーズ間の距離（mm）

    int set_count;

    ros::Publisher pub_arm_position;
    ros::Publisher pub_move_endeffector;

    ros::Subscriber sub_beads_positions;
    ros::Subscriber sub_arm_states;
    ros::Subscriber sub_endeffector_states;

    void init() { reset(); }

    void reset() {
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
        send_msg.data = "";
        arm_positions = {};
    }

    void reset_state() {
        is_arm_waiting_ = true;
        is_arm_moving_ = false;
        is_arm_goal_ = false;
        is_endeffector_waiting_ = true;
        is_endeffector_moving_ = false;
        is_endeffector_goal_ = false;
    }

    void beadsPositionsCallback(
        const std_msgs::String::ConstPtr &beads_positions) {
        received_beads_positions = beads_positions->data;
        // ROS_INFO("beads_positions:%s", received_beads_positions.c_str());
    }

    void armStatesCallback(const std_msgs::String::ConstPtr &arm_states) {
        received_arm_states = arm_states->data;
        // ROS_INFO("received_arm_states:%s", received_arm_states.c_str());
        if (received_arm_states == MSG_ARM_GOAL) {
            is_arm_goal_ = true;
            is_arm_waiting_ = true;
            is_arm_moving_ = false;
        } else if (received_arm_states == MSG_ARM_WAITING) {
            is_arm_waiting_ = true;
            is_arm_moving_ = false;
        } else if (received_arm_states == MSG_ARM_MOVING) {
            is_arm_waiting_ = false;
            is_arm_moving_ = true;
        }
    }

    void endeffectorStatesCallback(
        const std_msgs::String::ConstPtr &endeffector_states) {
        received_endeffector_states = endeffector_states->data;
        // ROS_INFO("received_endeffector_states:%s",
        // received_endeffector_states.c_str());
        if (received_endeffector_states == MSG_ENDEFFECTOR_GOAL) {
            is_endeffector_goal_ = true;
            is_endeffector_waiting_ = true;
            is_endeffector_moving_ = false;
        } else if (received_endeffector_states == MSG_ENDEFFECTOR_WAITING) {
            is_endeffector_waiting_ = true;
            is_endeffector_moving_ = false;
        } else if (received_endeffector_states == MSG_ENDEFFECTOR_MOVING) {
            is_endeffector_waiting_ = false;
            is_endeffector_moving_ = true;
        }
    }

    void convertBeadstoArmPositions() {
        for (int i = 0; i < (int)received_beads_positions.size(); i++) {
            if (std::isalpha(static_cast<unsigned char>(
                    received_beads_positions.c_str()[i]))) {
                //アルファベットを１０以上の数字に置き換える処理（未実装）
            } else {
                arm_positions.push_back(
                    int(received_beads_positions.c_str()[i] - '0'));
            }
            // ROS_INFO("arm_positions:%d", arm_positions[i]);
        }
    }

   public:
    int run(int argc, char **argv) {
        ros::NodeHandle node_handle;  //ノードハンドラ
        ros::Rate loop_rate(10);      //制御周期10Hz

        //パブリッシャ＆サブスクライバの設定
        std::string pub_arm_position_topic_name;
        std::string pub_move_endeffector_topic_name;

        std::string sub_beads_positions_topic_name;
        std::string sub_arm_states_topic_name;
        std::string sub_endeffector_states_topic_name;

        node_handle.param<std::string>("pub_arm_position_topic_name",
                                       pub_arm_position_topic_name,
                                       "/arm_positions");
        node_handle.param<std::string>("pub_move_endeffector_topic_name",
                                       pub_move_endeffector_topic_name,
                                       "/mes_beadsee");

        node_handle.param<std::string>("sub_beads_positions_topic_name",
                                       sub_beads_positions_topic_name,
                                       "/beads_positions");
        node_handle.param<std::string>("sub_arm_states_topic_name",
                                       sub_arm_states_topic_name,
                                       "/arm_states");
        node_handle.param<std::string>("sub_endeffector_states_topic_name",
                                       sub_endeffector_states_topic_name,
                                       "/mes_beadseeres");

        pub_arm_position = node_handle.advertise<geometry_msgs::Point>(
            pub_arm_position_topic_name, 60);
        pub_move_endeffector = node_handle.advertise<std_msgs::String>(
            pub_move_endeffector_topic_name, 60);

        sub_beads_positions = node_handle.subscribe(
            sub_beads_positions_topic_name, 60,
            &SetoScaraRobot::beadsPositionsCallback, this);
        sub_arm_states =
            node_handle.subscribe(sub_arm_states_topic_name, 60,
                                  &SetoScaraRobot::armStatesCallback, this);
        sub_endeffector_states = node_handle.subscribe(
            sub_endeffector_states_topic_name, 60,
            &SetoScaraRobot::endeffectorStatesCallback, this);

        init();

        ROS_INFO("seto_scararobot start!");

        while (ros::ok()) {
            switch (step_) {
                case Initialize: {
                    reset();
                    step_++;
                    ROS_INFO("WaitBeadsInfo");
                    break;
                }

                case WaitBeadsInfo: {
                    if (received_beads_positions != "") {
                        step_++;
                        convertBeadstoArmPositions();
                        ROS_INFO("CatchBeads");
                    }
                    break;
                }

                case SearchSetBeads: {
                    while (arm_positions[set_count] == SPACE) {
                        if (set_count <= (arm_positions.size() - 1)) {
                            set_count++;
                        } else {
                            break;
                        }
                    }
                    if (set_count > (arm_positions.size() - 1)) {
                        ROS_INFO("CheckFinishTask");
                        step_ = CheckFinishTask;
                    } else {
                        ROS_INFO("CatchBeads");
                        step_++;
                    }
                    break;
                }

                case CatchBeads: {
                    if (arm_positions[set_count] == COLOR1) {
                        arm_position.x =
                            color_first[color_first.size() - 1][0] * DISTANCE +
                            OFFSET_X;
                        arm_position.y =
                            color_first[color_first.size() - 1][1] * DISTANCE +
                            OFFSET_Y;
                        color_first.pop_back();
                        ROS_WARN("arm_position.x = %lf arm_position.y = %lf",
                                 arm_position.x, arm_position.y);
                    } else if (arm_positions[set_count] == COLOR2) {
                        arm_position.x =
                            color_second[color_second.size() - 1][0] *
                                DISTANCE +
                            OFFSET_X;
                        arm_position.y =
                            color_second[color_second.size() - 1][1] *
                                DISTANCE +
                            OFFSET_Y;
                        color_second.pop_back();
                        ROS_INFO("arm_position.x = %lf, arm_position.y = %lf",
                                 arm_position.x, arm_position.y);
                    } else if (arm_positions[set_count] == COLOR3) {
                        arm_position.x =
                            color_third[color_third.size() - 1][0] * DISTANCE +
                            OFFSET_X;
                        arm_position.y =
                            color_third[color_third.size() - 1][1] * DISTANCE +
                            OFFSET_Y;
                        color_third.pop_back();
                        ROS_INFO("arm_position.x = %lf, arm_position.y = %lf",
                                 arm_position.x, arm_position.y);
                    }
                    pub_arm_position.publish(arm_position);
                    step_++;

                    ROS_INFO("WaitCatchBeads");
                    break;
                }

                case WaitCatchBeads: {
                    if (is_arm_goal_ == true) {
                        step_++;
                        ROS_INFO("EndEffectorforCatch");
                    }
                    break;
                }

                case EndEffectorforCatch: {
                    // pub_move_endeffector.publish("Grasp");
                    send_msg.data = MSG_ENDEFFECTOR_GRUB;
                    pub_move_endeffector.publish(send_msg);
                    step_++;
                    ROS_INFO("WaitEndEffectorforCatch");
                    break;
                }

                case WaitEndEffectorforCatch: {
                    if (is_endeffector_goal_ == true) {
                        reset_state();
                        step_++;
                        ROS_INFO("SetBeads");
                    }
                    break;
                }

                case SetBeads: {
                    arm_position.x =
                        (int)(set_count % (int)sqrt(arm_positions.size())) *
                            DISTANCE +
                        OFFSET_X;
                    arm_position.y =
                        (int)(set_count / (int)sqrt(arm_positions.size())) *
                            DISTANCE +
                        OFFSET_Y;
                    pub_arm_position.publish(arm_position);
                    step_++;
                    ROS_INFO("WaitSetBeads");
                    break;
                }

                case WaitSetBeads: {
                    if (is_arm_goal_ == true) {
                        step_++;
                        ROS_INFO("EndEffectorforSet");
                    }
                    break;
                }

                case EndEffectorforSet: {
                    send_msg.data = MSG_ENDEFFECTOR_RELEASE;
                    pub_move_endeffector.publish(send_msg);
                    step_++;
                    ROS_INFO("WaitEndEffectorforSet");
                    break;
                }

                case WaitEndEffectorforSet: {
                    if (is_endeffector_goal_ == true) {
                        step_++;
                        ROS_INFO("CheckFinishTask");
                    }
                    break;
                }

                case CheckFinishTask: {
                    set_count++;
                    ROS_INFO("position:%d", arm_positions.size());
                    ROS_INFO("count:%d", set_count);
                    if (set_count >= arm_positions.size()) {
                        step_ = Initialize;
                    } else {
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "seto_scararobot");  //ノードの初期化
    SetoScaraRobot seto_scararobot;            //コンストラクタ
    return seto_scararobot.run(argc, argv);
};