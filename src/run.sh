#!/bin/bash

sudo docker run --rm -d \
  -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_DOMAIN_ID=42 \
  --net=host \
  -v /opt/robot-arm-slave/:/robot-arm-slave \
  --device /dev/gpiomem \
  --device /dev/mem \
  --privileged \
  --volume /sys/class/gpio:/sys/class/gpio \
  carwyn987/robot-arm-slave:1.0.0
