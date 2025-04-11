#!/bin/bash

docker run --rm -it \
  -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_DOMAIN_ID=42 \
  --net=host \
  --entrypoint /bin/bash \
  -v /home/pi_node1/dev/robot-arm-slave/:/robot-arm-slave \
  --device /dev/gpiomem \
  --device /dev/mem \
  --privileged \
  --volume /sys/class/gpio:/sys/class/gpio \
  carwyn987/robot-arm-slave:arm64v8-1.0.0
