# ==============================================
# Main Author: 윤수빈
# Last Modified: 2024-10-05
# Description: Track Race Mission Algorithmn
# Dependecy: fusion.launch를 켜고 구동하세요.
# ==============================================

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import PointCloud
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import time

# 전역 변수 정의
Cone_Sign = None
yolo_bboxes, obstacles = [], {}
yellow_cones, blue_cones = [], []
obstacles_lock = threading.Lock()
left_cone = None  # 전역 변수로 left_cone 설정
steering_angle = None  # 조홠각 초기화

# 현재 중간점 데이터를 저장할 전역 변수 추가
current_midpoints_x, current_midpoints_y = np.array([]), np.array([])

# 플로츠과 plot 요소 초기화
fig, ax = plt.subplots()

timeout_duration = 0.2 # 장애문 정보가 유효하다고 견상할 수 있는 시간 제한 (s)
position_threshold = 0.2 # 두 위치가 동일한 장애문로 견상될 수 있는 거리 임계값 (m)
lookahead_distance = 3.0 # 차량 앞 중간점을 계산하는 거리 (m)

# ROI 범위 설정 (x: 차량 앞뒤, y: 차량 양 옼)
roi_x_min, roi_x_max  = -1.0, 3.5  # ROI의 x 방햨 시작점 (차량 위치), ROI의 x 방햨 끝점 (차량 앞 2.5m)
roi_y_min, roi_y_max  = -4.0, 4.0  # ROI의 y 방햨 시작점 (차량 위치), ROI의 x 방햨 끝점 (차량 좌우 3m)

# ROI 내의 점을 필터링하기
def filter_points_within_roi(x_vals, y_vals, x_min, x_max):
    filtered_x, filtered_y = [], []

    for x, y in zip(x_vals, y_vals):
        if x_min <= x <= x_max:  # x가 ROI 경계 내에 있는지 확인
            filtered_x.append(x)
            filtered_y.append(y)

    return filtered_x, filtered_y

# Y = 1 (파란 콘) 이나 Y = -1 (노란 콘)에서 linear interpolation 
def linear_interpolation_with_fixed_start(x_vals, y_vals, fixed_y):
    if len(x_vals) < 1:
        rospy.logwarn("보간을 수행하기에 충분한 점이 없습니다.")
        return None

    # (X = 0, Y = fixed_y)에서 시작하는 점 추가
    x_vals = [0] + list(x_vals)
    y_vals = [fixed_y] + list(y_vals)

    try:
        # 1차 linear interpolation 수행
        return np.poly1d(np.polyfit(x_vals, y_vals, 1))
    except np.linalg.LinAlgError as e:
        rospy.logwarn(f"linear interpolation에 실패했습니다: {e}")
        return None

# 곡선 사이의 중간점을 계산하는 함수 
def calculate_midpoints(yellow_curve, blue_curve, x_vals, max_distance):
    # x_vals를 max_distance로 제한 
    x_vals = x_vals[x_vals <= max_distance]

    # 제한된 x_vals에 따른 yellow와 blue y값 재계산
    yellow_y_vals, blue_y_vals = yellow_curve(x_vals), blue_curve(x_vals)

    midpoints = (yellow_y_vals + blue_y_vals) / 2
    return midpoints, x_vals

# Lookahead 거리를 사용하여 조홠 각도 계산 
def compute_steering_angle(midpoints, x_vals):
    # midpoints와 x_vals의 길이가 동일한지 확인
    if len(midpoints) != len(x_vals):
        min_len = min(len(midpoints), len(x_vals))
        midpoints = midpoints[:min_len]
        x_vals = x_vals[:min_len]

    # 길이기 계산
    delta_y = np.gradient(midpoints)
    delta_x = np.gradient(x_vals)

    # 길이를 바탕으로 각도 계산
    angles = np.arctan2(delta_x, delta_y)
    return np.degrees(np.mean(angles))

# not-colored 장애문 주변에서 '가장 가까운 콘의 색상'을 기반으로 색상 결정
# interpolation 경우에 요청하신 수식으로 수정
# 노란 선 또는 파란 선을 발견하고 가장 가까운 선의 색상을 이용하여 장애문의 색상을 결정

def determine_obstacle_color(obs_x, obs_y, left_cone):
    global yellow_cones, blue_cones, current_midpoints_x, current_midpoints_y
    
    search_radius = 1.5  # 검색 범경 (미터 단위)
    min_distance = float('inf')
    closest_color = None

    # 노란 콘 검색
    for x, y in yellow_cones:
        dx, dy = obs_x - x, obs_y - y
        distance = np.hypot(dx, dy)
        if distance < min_distance:
            min_distance = distance
            closest_color = 1  # 노란상 콘

    # 파란 콘 검색
    for x, y in blue_cones:
        dx, dy = obs_x - x, obs_y - y
        distance = np.hypot(dx, dy)
        if distance < min_distance:
            min_distance = distance
            closest_color = 2  # 파란상 콘

    if min_distance <= search_radius:
        return closest_color
    else:
        # interpolation 결성 노란 선 또는 파란 선을 발견하고 사이의 전체 선의 길이로 색상 결정
        if current_midpoints_x.size > 0 and current_midpoints_y.size > 0:
            midpoint_curve = np.poly1d(np.polyfit(current_midpoints_x, current_midpoints_y, 1))
            midpoint_y_at_obs_x = midpoint_curve(obs_x)
            
            if obs_y < midpoint_y_at_obs_x:
                return 1 if left_cone == 1 else 2  # left_cone의 색
            else:
                return 1 if left_cone == 0 else 2  # right_cone의 색

        # interpolation이 실패하면 이전 방법 사용
        if current_midpoints_x.size > 0 and current_midpoints_y.size > 0:
            # 현재 중간점 데이터가 있는 경우, x 기준으로 중간점 y 값을 보간
            sorted_indices = np.argsort(current_midpoints_x)
            sorted_midpoints_x, sorted_midpoints_y = current_midpoints_x[sorted_indices], current_midpoints_y[sorted_indices]

            if sorted_midpoints_x[0] <= obs_x <= sorted_midpoints_x[-1]:
                midpoint_y_at_obs_x = np.interp(obs_x, sorted_midpoints_x, sorted_midpoints_y)
                # 장애문 y 좌표와 중간점 y 값 비교
                if obs_y < midpoint_y_at_obs_x:
                    side = 'left' # 장애문이 경로의 왼쪽
                else:
                    side = 'right' # 장애문이 경로의 오른쪽
            else:
                # 장애문 x 좌표가 중간점 x 범위 벌이인 경우 y=0 기준 사용
                side = 'left' if obs_y < 0 else 'right'
        else:
            # 중간점 데이터가 없는 경우 y=0 기준 사용
            side = 'left' if obs_y < 0 else 'right'

        # 좌우에 따른 색상 결정
        if side == 'left':
            return 1 if left_cone == 1 else 2  # left_cone의 색
        else:
            return 1 if left_cone == 0 else 2  # right_cone의 색
