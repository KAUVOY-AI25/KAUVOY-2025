#!/bin/bash

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 실행할 파이썬 스크립트들
python3 /home/kauvoy/Race/src/dynamic_obstacle.py &
python3 /home/kauvoy/Race/src/static_obstacle.py &
python3 /home/kauvoy/Race/src/uturn.py &

# 모든 스크립트가 실행되도록 대기
wait

echo "모든 파이썬 스크립트가 종료되었습니다."

