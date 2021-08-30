#!/usr/bin/env /usr/bin/python
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS Node
# Beads End effector Control
#
# The Apache License 2.0
# Copyright (C) 2020-2021 myasu.
# -----------------------------------------------

import logging
import os
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16
#include <rclcpp/qos.hpp>
# 自作クラス
from .BeadsEeSerial import BeadsEeSerial


class BeadsEe(Node):
    """
    BeadsEE制御ノード
    """
    # ノード名
    SELFNODE = "beadsee"
    # トピック名：制御受付
    SELFTOPIC = "mes_" + SELFNODE
    # トピック名：動作結果
    SELFTOPIC_RES = "mes_" + SELFNODE + "res"
    def __init__(self, argCOM=None):

        """
        コンストラクタ
        Parameters
        ----------
        argCOM : string
            ポート名。無指定の場合はシリアル制御をしない
        """

        super().__init__('beadsee')
        
        # String型のchatterトピックを受信するsubscriptionの定義
        # （listener_callbackは受信毎に呼び出されるコールバック関数）
        self.subscriber = self.create_subscription(
            String, self.SELFTOPIC, self._subscribe, 10)

        # String型のchatterトピックを送信するpublisherの定義
        # self.subscriber
        self.publisher = self.create_publisher(String, self.SELFTOPIC_RES, 10)

        # 起動完了メッセージ
        #起動メッセージ
        self.get_logger().info("[{}] Do...(Topic: [Pub: {}, Sub: {}])".format(
            os.path.basename(__file__),
            self.SELFTOPIC_RES,
            self.SELFTOPIC))
        
        self._publish("Waiting")

    def _subscribe(self, msg):
        """Subscribeコールバック"""
        # 受信コマンドをトリム
        command = msg.data.strip()
        # 受信コマンドの表示
        self.get_logger().info("[%s] Recieved command: %s" %
                      (self.__class__.__name__, command))
        # 受信コマンドの解析と処理分岐
        if command == "grub":
            # self._beads.grub()
            self._publish("Moving")
            time.sleep(0.5)
            self._publish("Goal")
            self._publish("Waiting")
        elif command == "release":
            # self._beads.release()
            self._publish("Moving")
            time.sleep(0.5)
            self._publish("Goal")
            self._publish("Waiting")
        else:
            rospy.loginfo("[%s] ! Command error" %
                          (self.__class__.__name__))

    def _eventThread_BeadsEe(self):
        """エンドエフェクタ制御の状態確認スレッド"""
        while True:
            # 結果確認
            (result, sendtime, receivetime) = self._beads.Result
            if result is not None:
                # 結果が入っているときだけPublish
                if result == True:
                    self._publish("Goal")
                    self._publish("Waiting")
                elif result == False:
                    self._publish("Moving")
            # ウェイト
            time.sleep(0.1)

    def _publish(self, argData):
        """
        ROSメッセージのPublish
        Parameters
        ----------
        argData : String
            送信データ文字列
        Return
        ----------
            True: 成功、False: 失敗
        """
        try:
            # self._pub.publish(argData)
            msg = String()
            msg.data = argData
            # chatterトピックにmsgを送信
            self.publisher.publish(msg)
            return True
        except:
            # 例外発生時にメッセージ
            import traceback
            traceback.print_exc()
        return False

def main(args=None):
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    # beadsee ノードの作成
    beadsee = BeadsEe()
    # minimal_publisherノードの実行開始
    rclpy.spin(beadsee)
    # Pythonクライアントライブラリの終了
    rclpy.shutdown()

if __name__ == '__main__':
    main()


