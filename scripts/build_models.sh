#!/usr/bin/env bash
set -e
# Reference: https://github.com/evshary/autoware_carla_launch/blob/9483e5339ce9619e2cd249c55e6faf63016e3b4e/script/setup/build_models.sh
# lidar centerpoint (tiny)
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml \
	model_name:=centerpoint_tiny \
	model_path:=$HOME/autoware_data/lidar_centerpoint \
	model_param_path:=$(ros2 pkg prefix autoware_launch --share)/config/perception/object_recognition/detection/lidar_model/centerpoint_tiny.param.yaml \
	build_only:=true

# lidar centerpoint
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml \
        model_name:=centerpoint \
        model_path:=$HOME/autoware_data/lidar_centerpoint \
        model_param_path:=$(ros2 pkg prefix autoware_launch --share)/config/perception/object_recognition/detection/lidar_model/centerpoint.param.yaml \
        build_only:=true

# lidar transfusion
ros2 launch autoware_lidar_transfusion lidar_transfusion.launch.xml \
        model_name:=transfusion \
        model_path:=$HOME/autoware_data/lidar_transfusion \
        model_param_path:=$(ros2 pkg prefix autoware_launch --share)/config/perception/object_recognition/detection/lidar_model/transfusion.param.yaml \
        build_only:=true
