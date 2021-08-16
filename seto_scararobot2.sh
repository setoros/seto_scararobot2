#!/bin/bash
#--------------------------------------------------------------------
#バックグラウンド実行用のスクリプト
#--------------------------------------------------------------------

#変数の設定
SCRIPTDIR=~/catkin_ws/src/seto_scararobot2/
LOGDIR=$SCRIPTDIR/log
ENVFILE=~/catkin_ws/devel/setup.bash
ENVFILE_ROS=/opt/ros/noetic/setup.bash

#FTDI Timer Setting 1msec
sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

#実行
if [ -f ${ENVFILE} ]; then
    #環境変数読み込み
    echo "Loading ROS Env..."
    source $ENVFILE_ROS
    source $ENVFILE
    if [ -d ${LOGDIR} ]; then
        echo "ROS Launching..."
        #roslaunch実行
        exec roslaunch seto_scararobot2 move_arm_hardware.launch  >> ${LOGDIR}/seto_scararobot2.log 2>&1
    else
        echo "There is no ${LOGDIR}"
    fi
else
    echo "There is no ${ENVFILE}"
fi