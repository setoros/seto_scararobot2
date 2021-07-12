# seto_scararobot2 について

## 準備

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
$ roslaunch seto_scararobot2_description display.launch
```

### 使い方2 (rviz上のスカラロボットが勝手に動きます。)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_test1_rviz.launch
```
joint_state_publisherの代わりに、move_testノードが立ち上がります。

### 使い方3 (IKで座標を指定してrviz上のスカラロボットを動かす。)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot2 move_test2_rviz.launch
```
rqt_ez_publisherが立ち上がりますので、x,yのスライダを動かすことでスカラロボットが動きます。単位は(mm)です。
※ rqt_ez_publisherパッケージをあらかじめインストールしておく必要があります。


### 使い方10 (ロボットアームをwebアプリから動かす。【本命】) ※実機が必要 (未実装)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot move_arm_hardware.launch
```

2. webブラウザで[localhost:5000](localhost:5000)にアクセスする

※ 1.のコマンドの代わりに、seto_scararobot.shを実行してもOK、その場合レイテンシタイマーの設定もシェルの中で行います。

### 使い方11 (gazenoのロボットアームをwebアプリから動かす。【本命】) (未実装)

1. 下記コマンドを実行

```
$ roslaunch seto_scararobot move_arm_gazebo.launch
```
2. webブラウザで[localhost:5000](localhost:5000)にアクセスする

### FTDIドライバのレイテンシタイマーの変更

```
 $ sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
 $ echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## 予定・課題
* (n/a)

## 履歴
* 2021/07/12 :  とりあえず、１号機から移植

## Author

* 瀬戸内ROS勉強会
  * https://ros.xrea.jp/

## License

This is under [MIT license](https://en.wikipedia.org/wiki/MIT_License).
