#!/usr/bin/env bash

#Record .bag file with topics relevant for MAV operating with GPS/IMU/vision
# (Trinity autopilot)
echo "Usage : source $0 \"README Text\""

CURR_DIR=`pwd`
FOLDER_NAME="~/bags/$(date +"%Y%m%d")"

mkdir -p $FOLDER_NAME
cd $FOLDER_NAME
touch README
rosparam dump "$(date +"%F-%H-%M-%S")".yaml

BAG_NAME="$(date +"%F-%H-%M-%S").bag"
echo $BAG_NAME >> README
echo $1 >> README
rosbag record --output-name=$BAG_NAME \
/camera/driver/device_time \
/camera/imu/data_raw \
/camera/fisheye/image_raw \
/camera/depth/image_raw \
/camera/depth/points \
/camera/color/image_raw \
/camera/ir/image_raw \
/camera/ir2/image_raw \
/camera/depth/image_raw
echo "Enter more info for readme and press ENTER."
read text
echo $text >> README
echo -e "\n" >> README
rosbag info $BAG_NAME
cd CURR_DIR
