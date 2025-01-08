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
steering_angle = None  # 조향각 초기화

# 브레이크 및 속도 제어를 위한 추가 변수
brake_pub = None
velocity_pub = None

# 현재 중간점 데이터를 저장할 전역 변수 추가
current_midpoints_x, current_midpoints_y = np.array([]), np.array([])

# 플롯과 plot 요소 초기화
fig, ax = plt.subplots()

timeout_duration = 0.2  # 장애물 정보가 유효하다고 간주할 수 있는 시간 제한 (s)
position_threshold = 0.2  # 두 위치가 동일한 장애물로 간주될 수 있는 거리 임계값 (m)
lookahead_distance = 3.0  # 차량 앞 중간점을 계산하는 거리 (m)

# ROI 범위 설정 (x: 차량 앞뒤, y: 차량 양 옆)
roi_x_min, roi_x_max = -1.0, 3.5  # ROI의 x 방향 시작점 (차량 위치), ROI의 x 방향 끝점 (차량 앞 2.5m)
roi_y_min, roi_y_max = -4.0, 4.0  # ROI의 y 방향 시작점 (차량 위치), ROI의 x 방향 끝점 (차량 좌우 3m)

# 브레이크와 속도를 제어하는 함수
def control_brake_and_velocity(steering_angle):
    if abs(steering_angle) > 5:  # 조향각이 5도를 넘으면
        # brake_value를 10으로 설정하고 3초간 유지
        brake_pub.publish(10.0)
        velocity_pub.publish(40.0)  # 속도를 40으로 설정
        rospy.loginfo("Brake applied, velocity set to 40")
        time.sleep(3)  # 3초 대기
        brake_pub.publish(0.0)  # 브레이크 해제
        rospy.loginfo("Brake released")
    else:
        velocity_pub.publish(40.0)  # 조향각이 5도 이하일 때도 속도를 40으로 유지

# ROI 내의 점을 필터링
def filter_points_within_roi(x_vals, y_vals, x_min, x_max):
    filtered_x, filtered_y = [], []

    for x, y in zip(x_vals, y_vals):
        if x_min <= x <= x_max:  # x가 ROI 경계 내에 있는지 확인
            filtered_x.append(x)
            filtered_y.append(y)

    return filtered_x, filtered_y

# Y = 1 (파란 콘) 또는 Y = -1 (노란 콘)에서 linear interpolation
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

    # 제한된 x_vals에 따라 yellow와 blue y값 재계산
    yellow_y_vals, blue_y_vals = yellow_curve(x_vals), blue_curve(x_vals)

    midpoints = (yellow_y_vals + blue_y_vals) / 2
    return midpoints, x_vals

# Lookahead 거리를 사용하여 조향 각도 계산
def compute_steering_angle(midpoints, x_vals):
    # midpoints와 x_vals의 길이가 동일한지 확인
    if len(midpoints) != len(x_vals):
        min_len = min(len(midpoints), len(x_vals))
        midpoints = midpoints[:min_len]
        x_vals = x_vals[:min_len]

    # 기울기 계산
    delta_y = np.gradient(midpoints)
    delta_x = np.gradient(x_vals)

    # 기울기를 바탕으로 각도 계산
    angles = np.arctan2(delta_x, delta_y)
    return np.degrees(np.mean(angles))

# 장애물 주변에서 '가장 가까운 콘의 색상'을 기반으로 색상 결정
def determine_obstacle_color(obs_x, obs_y, left_cone):
    search_radius = 1.5  # 검색 반경 (미터 단위)
    min_distance = float('inf')
    closest_color = None

    # 노란 콘 검색
    for x, y in yellow_cones:
        dx, dy = obs_x - x, obs_y - y
        distance = np.hypot(dx, dy)
        if distance < min_distance:
            min_distance = distance
            closest_color = 1  # 노란색 콘

    # 파란 콘 검색
    for x, y in blue_cones:
        dx, dy = obs_x - x, obs_y - y
        distance = np.hypot(dx, dy)
        if distance < min_distance:
            min_distance = distance
            closest_color = 2  # 파란색 콘

    if min_distance <= search_radius:
        return closest_color
    else:
        # 반경 내에 콘이 없을 경우 기존 방법 사용
        global current_midpoints_x, current_midpoints_y

        if current_midpoints_x.size > 0 and current_midpoints_y.size > 0:
            # 현재 중간점 데이터가 있는 경우, x 기준으로 중간점 y 값을 보간
            sorted_indices = np.argsort(current_midpoints_x)
            sorted_midpoints_x, sorted_midpoints_y = current_midpoints_x[sorted_indices], current_midpoints_y[sorted_indices]

            if sorted_midpoints_x[0] <= obs_x <= sorted_midpoints_x[-1]:
                midpoint_y_at_obs_x = np.interp(obs_x, sorted_midpoints_x, sorted_midpoints_y)
                # 장애물 y 좌표와 중간점 y 값 비교
                if obs_y < midpoint_y_at_obs_x:
                    side = 'left'  # 장애물이 경로의 왼쪽
                else:
                    side = 'right'  # 장애물이 경로의 오른쪽
            else:
                # 장애물 x 좌표가 중간점 x 범위 밖인 경우 y=0 기준 사용
                side = 'left' if obs_y < 0 else 'right'
        else:
            # 중간점 데이터가 없을 경우 y=0 기준 사용
            side = 'left' if obs_y < 0 else 'right'

        # 좌우에 따라 색상 결정
        if side == 'left':
            return 1 if left_cone == 1 else 2  # left_cone의 색
        else:
            return 1 if left_cone == 0 else 2  # right_cone의 색

# plot 초기화
def init():
    ax.set_xlim(-4, 4)
    ax.set_ylim(-2, 10)
    ax.grid(True)
    ax.set_title('Lidar position and steering angle')
    ax.set_xlabel('Y')
    ax.set_ylabel('X')
    ax.invert_xaxis()  # X축 반전
    ax.plot(0, 0, 'ro', label='LiDAR Position and Steering Angle')
    ax.legend(['Yellow Cone', 'Blue Cone', 'Obstacle', 'Midpoint Curve'], loc='upper right')
    return []

# 두 개의 점만을 사용하여 선형 보간을 수행하는 함수
def interpolate_with_two_points(x_vals, y_vals):
    if len(x_vals) != 2:
        rospy.logwarn("이 함수는 정확히 두 개의 점이 필요합니다.")
        return None
    try:
        # 두 점을 통과하는 직선의 방정식을 구합니다.
        return np.poly1d(np.polyfit(x_vals, y_vals, 1))
    except np.linalg.LinAlgError as e:
        rospy.logwarn(f"보간에 실패했습니다: {e}")
        return None

# 노란색 또는 파란색 점이 하나만 있을 때 기울기대로 steering_angle을 설정하는 함수
def handle_single_point_interpolation(yellow_x_filtered, yellow_y_filtered, blue_x_filtered, blue_y_filtered):
    if len(yellow_x_filtered) == 1 and len(blue_x_filtered) == 2:
        rospy.loginfo("노란색 점이 하나만 있고 파란색 점이 두 개 이상인 경우")
        yellow_curve = interpolate_with_two_points(blue_x_filtered, blue_y_filtered)
        steering_angle = compute_steering_angle(yellow_curve(blue_x_filtered), blue_x_filtered)
        steering_angle_pub.publish((steering_angle - 90) * 0.8)
    elif len(blue_x_filtered) == 1 and len(yellow_x_filtered) == 2:
        rospy.loginfo("파란색 점이 하나만 있고 노란색 점이 두 개 이상인 경우")
        blue_curve = interpolate_with_two_points(yellow_x_filtered, yellow_y_filtered)
        steering_angle = compute_steering_angle(blue_curve(yellow_x_filtered), yellow_x_filtered)
        steering_angle_pub.publish((steering_angle - 90) * 0.8)

def update(frame):
    current_time = time.time()  # 현재 시간
    ax.clear()  # 기존 plot 클리어
    init()  # 초기화 함수 호출

    with obstacles_lock:
        obstacles_to_remove = []
        for obs_id, data in obstacles.items():
            # 시간 제한을 넘긴 장애물 제거
            if current_time - data['timestamp'] > timeout_duration:
                obstacles_to_remove.append(obs_id)

        for obs_id in obstacles_to_remove:
            del obstacles[obs_id]

        yellow_cones.clear()  # 노란 콘 초기화
        blue_cones.clear()  # 파란 콘 초기화

        # 고정된 콘 추가
        global left_cone
        if left_cone == 0:  # left cone이 노란색일 때
            yellow_cones.append((-0.8, 1.0))  # 고정된 노란 콘
            blue_cones.append((-0.8, -1.0))  # 고정된 파란 콘
        elif left_cone == 1:  # left cone이 파란색일 때
            yellow_cones.append((-0.8, -1.0))  # 고정된 노란 콘
            blue_cones.append((-0.8, 1.0))  # 고정된 파란 콘

        # 모든 장애물을 플롯하고 보간 대상에 추가
        for obs_id, data in obstacles.items():
            if (roi_x_min <= data['x'] <= roi_x_max) and (roi_y_min <= data['y'] <= roi_y_max):
                if data['color'] == 1:
                    yellow_cones.append((data['x'], data['y']))  # 노란색 장애물 추가
                elif data['color'] == 2:
                    blue_cones.append((data['x'], data['y']))  # 파란색 장애물 추가

        # 콘 plot
        for x, y in yellow_cones:
            ax.plot(y, x, 'yo')  # 노란 콘

        for x, y in blue_cones:
            ax.plot(y, x, 'bo')  # 파란 콘

        # 노란 콘과 파란 콘의 점을 ROI 내에서 필터링
        yellow_x, yellow_y = zip(*yellow_cones) if yellow_cones else ([], [])
        blue_x, blue_y = zip(*blue_cones) if blue_cones else ([], [])

        yellow_x_filtered, yellow_y_filtered = filter_points_within_roi(yellow_x, yellow_y, roi_x_min, roi_x_max)
        blue_x_filtered, blue_y_filtered = filter_points_within_roi(blue_x, blue_y, roi_x_min, roi_x_max)

        # 보간 수행
        yellow_curve, blue_curve = None, None

        # 노란 콘 보간
        if len(yellow_x_filtered) >= 1:
            # 두 개의 점만 있을 경우, 해당 점들로만 보간
            if len(yellow_x_filtered) == 2:
                yellow_curve = interpolate_with_two_points(yellow_x_filtered, yellow_y_filtered)
            else:  # 기존의 보간 방법 사용
                yellow_curve = linear_interpolation_with_fixed_start(yellow_x_filtered, yellow_y_filtered, -1)
        else:
            yellow_curve = None

        # 파란 콘 보간
        if len(blue_x_filtered) >= 1:
            # 두 개의 점만 있을 경우, 해당 점들로만 보간
            if len(blue_x_filtered) == 2:
                blue_curve = interpolate_with_two_points(blue_x_filtered, blue_y_filtered)
            else:  # 기존의 보간 방법 사용
                blue_curve = linear_interpolation_with_fixed_start(blue_x_filtered, blue_y_filtered, 1)
        else:
            blue_curve = None

        # '두 곡선이 모두 존재할 경우에만' 중간점 계산 및 조향각 계산
        if yellow_curve is not None and blue_curve is not None:
            x_vals = np.linspace(min(min(yellow_x_filtered), min(blue_x_filtered)),
                                 max(max(yellow_x_filtered), max(blue_x_filtered)), num=100)

            # ROI 범위로 중간점 곡선 제한
            midpoints, x_vals = calculate_midpoints(yellow_curve, blue_curve, x_vals, lookahead_distance)

            # 현재 중간점 데이터를 전역 변수에 저장
            global current_midpoints_x, current_midpoints_y
            current_midpoints_x, current_midpoints_y = x_vals, midpoints

            # 노란 콘, 파란 콘, 중간점 곡선 플롯
            ax.plot(yellow_curve(x_vals), x_vals, 'y-', label='Yellow Cone')
            ax.plot(blue_curve(x_vals), x_vals, 'b-', label='Blue Cone')
            ax.plot(midpoints, x_vals, 'g--', label='Midpoint Curve')

            # 조향각 계산 및 발행
            global steering_angle
            steering_angle = compute_steering_angle(midpoints, x_vals)
            steering_angle_pub.publish((steering_angle - 90) * 0.8)  # steering angle

            # 브레이크 및 속도 제어 함수 호출
            control_brake_and_velocity(steering_angle)

            # 조향각을 화살표로 표시 (ax.arrow 사용)
            midpoint_x = midpoints[len(midpoints) // 2]
            midpoint_y = x_vals[len(x_vals) // 2]

            # 화살표의 길이와 방향 설정
            arrow_length = 0.8  # 화살표 길이 설정
            arrow_dx = arrow_length * np.cos(np.radians(steering_angle))
            arrow_dy = arrow_length * np.sin(np.radians(steering_angle))

            # 조향각을 나타내는 화살표를 plot, text
            ax.arrow(midpoint_x, midpoint_y, arrow_dx, arrow_dy, color='r', head_width=0.5, length_includes_head=True)
            ax.text(midpoint_x + arrow_dx, midpoint_y + arrow_dy, f'{(steering_angle - 90)*0.8 :.2f}°', color='r', fontsize=12)  # steering angle

        else:
            handle_single_point_interpolation(yellow_x_filtered, yellow_y_filtered, blue_x_filtered, blue_y_filtered)

# 기존 장애물을 찾는 함수
def find_existing_obstacle(x, y, color):
    """주어진 위치와 색상의 기존 장애물을 찾음"""
    for obs_id, data in obstacles.items():
        if abs(data['x'] - x) < position_threshold and abs(data['y'] - y) < position_threshold and data['color'] == color:
            return obs_id
    return None

# YOLO 데이터를 처리하는 콜백 함수
def yolo_callback(msg):
    global yolo_bboxes
    yolo_bboxes.clear()

    Cone_Sign_states = {
        "cone_yellow": 1,  # 1은 노란 콘
        "cone_blue": 2  # 2는 파란 콘
    }

    global Cone_Sign

    for channel in msg.channels:
        if len(channel.values) >= 5:  # 충분한 데이터가 있는지 확인
            bbox = {
                'x1': channel.values[1], 'y1': channel.values[2],
                'x2': channel.values[3], 'y2': channel.values[4],
                'confidence': channel.values[0],
                'label': channel.name
            }
            yolo_bboxes.append(bbox)
            # rospy.loginfo(f"{channel.name}에 대한 경계 상자를 처리했습니다.")

            if channel.name in Cone_Sign_states:
                Cone_Sign = Cone_Sign_states[channel.name]

# 장애물 데이터를 처리하는 콜백 함수
def obstacle_callback(msg):
    global yolo_bboxes
    current_time = time.time()

    in_yolo_box = False
    obs_x, obs_y = msg.data[0], msg.data[1]

    # YOLO 바운딩 박스 내에 있는지 확인
    for bbox in yolo_bboxes:
        if bbox['x1'] <= msg.data[3] <= bbox['x2'] and bbox['y1'] <= msg.data[4] <= bbox['y2']:
            print(f"경계 상자 내의 장애물: label = {bbox['label']}, conf = {bbox['confidence']}")
            print(f"장애물 좌표: x = {obs_x}, y = {obs_y}")
            print(f"거리: {msg.data[5]} m")

            # ROI 범위 안에 있는지 확인
            if roi_x_min <= obs_x <= roi_x_max:
                in_yolo_box = True

                # 콘 타입에 따라 색상 결정
                color = 0  # 기본 색상 (default, 사용할 일 없음)
                if bbox['label'] == "cone_blue":
                    color = 2  # 파란색 콘
                elif bbox['label'] == "cone_yellow":
                    color = 1  # 노란색 콘

                # 위치 임계값 내에서 기존 장애물을 찾기
                obs_id = find_existing_obstacle(obs_x, obs_y, color)

                if obs_id is None:
                    # 장애물이 없다면 새 장애물 ID 생성
                    obs_id = (obs_x, obs_y, color)

                # 장애물 정보 추가 또는 업데이트
                with obstacles_lock:
                    obstacles[obs_id] = {
                        'x': obs_x,
                        'y': obs_y,
                        'color': color,
                        'timestamp': current_time
                    }

    # YOLO 바운딩 박스 외부의 장애물 (ROI 내부의 장애물만)
    if not in_yolo_box and roi_x_min <= obs_x <= roi_x_max:
        print(f"YOLO 박스 외부의 장애물: x = {obs_x}, y = {obs_y}")

        color = determine_obstacle_color(obs_x, obs_y, left_cone)  # Color 지정

        # 색상이 None인 경우 -> 기본 색상 지정
        if color is None:
            color = 1 if left_cone == 1 else 2  # 또는 다른 기본값 설정

        # 위치 임계값 내에서 기존 장애물을 찾기
        obs_id = find_existing_obstacle(obs_x, obs_y, color)

        # 장애물이 None인 경우 -> 새 장애물 ID 생성
        if obs_id is None:
            obs_id = (obs_x, obs_y, color)

        # 장애물 정보 추가 또는 업데이트
        with obstacles_lock:
            obstacles[obs_id] = {
                'x': obs_x,
                'y': obs_y,
                'color': color,
                'timestamp': current_time
            }

# 리스너 함수
def listener():
    rospy.init_node('object_detection_listener', anonymous=True)

    global left_cone  # 전역 변수로 left_cone 설정
    left_cone = int(input("what is the left cone? (yellow=0, blue=1)"))

    # 조향각 publish
    global steering_angle_pub
    steering_angle_pub = rospy.Publisher('/steering_angle', Float64, queue_size=10)

    # 브레이크와 속도 publish
    global brake_pub, velocity_pub
    brake_pub = rospy.Publisher('/brake_value', Float64, queue_size=10)
    velocity_pub = rospy.Publisher('/velocity', Float64, queue_size=10)

    # YOLO 및 장애물 정보 subscribe
    rospy.Subscriber('/yolov9_detect', PointCloud, yolo_callback)
    rospy.Subscriber('/obstacle_info', Float32MultiArray, obstacle_callback)

    # 애니메이션 생성 및 plot 표시
    ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=50, cache_frame_data=False)
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    listener()
