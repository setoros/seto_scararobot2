# seto_scararobot2 について

## 準備

### インストール

事前に以下のパッケージをインストールする。

```
$ sudo apt-get update
$ sudo apt install ros-foxy-xacro
$ sudo apt install ros-foxy-geometry-msgs
$ pip install flask
$ pip install serial
```

### ビルド

1. ROSワークスペースのsrcフォルダの中にクローンして、catkin_make

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/dreamdrive/seto_scararobot2.git
$ cd ..
$ catkin_make
```

## 使い方

### 使い方1 (joint_state_publisherでrviz上のスカラロボットをスライダを動かせます。)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_test2_rviz.py
```

### 使い方2 (rviz上のスカラロボットが勝手に動きます。)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_test1_1_rviz.py
```
joint_state_publisherの代わりに、move_test1_1ノードが立ち上がります。

### 使い方3 (rviz2のロボットアームをwebアプリから動かす。【本命】) 

1. 下記コマンドを実行

```
$ export BS_DIR_PATH = [flask_topicsの絶対パス]
例：export BS_DIR_PATH =　/home/user/ros2/src/seto_scararobot2/beads_setter/flask_topics
$ roslaunch seto_scararobot seto_scararobot2.py
```
2. webブラウザで[localhost:5000](localhost:5000)にアクセスする

### 使い方4 (ロボットアームをwebアプリから動かす。【本命】) ※実機が必要 (未実装)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot move_arm_hardware.launch
```

2. webブラウザで[localhost:5000](localhost:5000)にアクセスする

※ 1.のコマンドの代わりに、seto_scararobot.shを実行してもOK、その場合レイテンシタイマーの設定もシェルの中で行います。

### FTDIドライバのレイテンシタイマーの変更

```
 $ sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
 $ echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## ヒント

### catkin_makeを実行してもroslaunchでlaunchファイルが実行できない or 予測変換で出現しない
以下のコマンドを実行する。
```
$ source install/setup.bash && source install/local_setup.bash
```

## 予定・課題
* (n/a)

## 履歴
* 2021/08/29 :  foxyに対応。ハード対応は今後行う。

## Author

* 瀬戸内ROS勉強会
  * https://ros.xrea.jp/

## License

This is under [Apache License 2.0](./LICENSE).
