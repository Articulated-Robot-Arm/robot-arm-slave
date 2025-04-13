source /opt/ros/humble/setup.bash
cd /opt/ros2_ws/
colcon build --packages-select servo
pip install RPi.GPIO

env -i /bin/bash  -c '
  export HOME=$PWD;
  export RCUTILS_LOGGING_DIRECTORY=$PWD/log/;
  source /opt/ros/humble/setup.bash;
  source install/setup.bash;
  ros2 run servo servo;
'