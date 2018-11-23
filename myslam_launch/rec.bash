#!/bin/bash

rosbag record -O world5-run1.bag /camera1/camera_indo /camera1/image_raw /clock /cmd_vel /gazebo/model_states /landmark /logical_camera_image /odom /tf /tf_static
