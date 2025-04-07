#!/bin/bash

docker run --rm -it \
  -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_DOMAIN_ID=42 \
  --net=host \
  carwyn987/robot-arm-slave:arm64v8-1.0.0
