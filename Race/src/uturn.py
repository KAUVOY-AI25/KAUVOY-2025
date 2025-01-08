# ================================================================================
# Main Author: 윤수빈
# Recently Modified Date: 2024-09-29
# Dependency: race_kcity.py, lidar (DBSCAN)
# Description: 예선 유턴 알고리즘(장애물이 인식될 경우 고정 스티어링) 
#		       mode_topic 받아와서 3일 때에만 uturn_flag = 1
# ================================================================================

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Int32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import numpy as np
from matplotlib.patches import Polygon

class UturnDetector:
    def __init__(self):
        self.obstacles, self.obstacles_lock = {}, threading.Lock()  # 장애물 정보에 대한 스레드 락
        self.timeout_duration = 0.4  # 장애물 정보가 유효하다고 간주할 수 있는 시간 제한 (초 단위)
        self.last_obstacle_time = time.time()  # 마지막 장애물 감지 시간 초기화

        # 관심 영역(ROI) 설정
        self.roi_x_max, self.roi_x_min = 5.5, 0  # 라이다 기준 앞뒤(+-)
        self.roi_y_max, self.roi_y_min = 1.5, -0.5  # 라이다 기준 좌우(+-)

        # U-turn flag 초기화
        self.uturn_flag = 0  # 초기값 설정

        # ROS 퍼블리셔
        self.uturn_flag_pub = None
        
        # 현재 모드를 저장할 변수
        self.race_mode = None  # Mode topic에서 들어오는 값을 저장할 변수

        # Matplotlib 초기화
        self.fig, self.ax = plt.subplots()
        self.init_plot()

    def mode_callback(self, msg):
        # 모드 값을 업데이트하는 콜백
        self.race_mode = msg.data
        rospy.loginfo(f"Mode updated to: {self.race_mode}")

    def init_plot(self):
        self.ax.set_xlim(-2, 2)  # Y축 범위 설정
        self.ax.set_ylim(-1.5, 6)  # X축 범위 설정
        self.ax.grid(True)  # 그리드 표시
        self.ax.set_title('LIDAR Based U-Turn')  # 플롯 제목 설정
        self.ax.set_xlabel('Y (from LIDAR)')  # Y축 레이블 설정
        self.ax.set_ylabel('X (from LIDAR)')  # X축 레이블 설정
        self.ax.invert_xaxis()  # LiDAR 기준으로 앞쪽을 x(+)로 설정

        # 자동차를 사각형으로 그리기
        car_width, car_length = 1.2, 1.8  # 자동차의 전체 너비 (양쪽 각각 0.6미터), LiDAR 뒤쪽 자동차 길이
        car_rect = plt.Rectangle((-car_width / 2, -car_length), car_width, car_length,
                                 linewidth=2, edgecolor='blue', facecolor='none', label='Car')
        self.ax.add_patch(car_rect)

        # x=0 위치에 수직선 그리기 (LiDAR 기준 좌우)
        self.ax.axvline(x=0, color='cyan', linestyle='--', label='x=0 line')  # 청록색 점선
        self.ax.plot(0, 0, 'ro', label='LiDAR Position and Steering Angle')  # LiDAR 위치 표시

    def update_plot(self, frame):
        current_time = time.time()
        closest_obstacle, closest_distance = None, float('inf')  # 가장 가까운 장애물 정의

        self.ax.clear()  # 프레임 clear
        self.init_plot()  # plot 초기화

        # 관심 영역(ROI) 표시
        roi_rect = plt.Rectangle((self.roi_y_min, self.roi_x_min), self.roi_y_max - self.roi_y_min, self.roi_x_max - self.roi_x_min,
                                linewidth=2, edgecolor='r', facecolor='none', label='ROI')
        self.ax.add_patch(roi_rect)  

        # uturn_flag 상태를 텍스트로 플롯 상단에 표시
        uturn_flag_text = f"Race Mode: {self.race_mode}, uturn_flag: {self.uturn_flag}"
        self.ax.text(1.8, 4.5, uturn_flag_text, fontsize=10, color='blue', 
                    bbox=dict(facecolor='white', alpha=0.6))  # 플롯 상단에 텍스트 표시

        with self.obstacles_lock:
            active_obstacles = {}
            for obs_id, data in list(self.obstacles.items()):
                if current_time - data['timestamp'] > self.timeout_duration:
                    # 장애물이 유효하지 않으면 삭제
                    del self.obstacles[obs_id]
                    continue

                active_obstacles[obs_id] = data

                # 장애물 플롯 (중심점 및 모서리)
                self.ax.plot(data['center_y'], data['center_x'], 'bo', markersize=4)  # Plot obstacle center

                corners = data['corners']
                for corner in corners:
                    self.ax.plot(corner[1], corner[0], 'go', markersize=4)  # Plot each corner point

                # 장애물까지의 거리 계산 (LiDAR 위치 기준으로 x=0에서 가장 가까운 장애물 선택)
                distance_to_lidar = abs(data['center_x'])
                if distance_to_lidar < closest_distance:
                    closest_distance = distance_to_lidar
                    closest_obstacle = data

                # 장애물의 네 점을 연결하여 사각형 그리기
                if len(corners) == 4:
                    polygon = Polygon([(corner[1], corner[0]) for corner in corners], closed=True, edgecolor='green', facecolor='none', linewidth=1)
                    self.ax.add_patch(polygon)

                # U-Turn 메인 알고리즘 (Mode가 3일 때에만 uturn_flag를 1로 설정)
                if self.race_mode == 4: 
                    if self.roi_x_min <= data['center_x'] <= self.roi_x_max and self.roi_y_min <= data['center_y'] <= self.roi_y_max:
                        self.ax.text(closest_obstacle['center_y'], closest_obstacle['center_x'], 'Closest', color='red')
                        if self.uturn_flag != 1:
                            self.uturn_flag = 1
                            print("장애물이 ROI내에 들어왔고, Mode 3. uturn_flag = 1")
                else:
                    self.uturn_flag = 0
                    print(f"현재 Mode {self.race_mode}. uturn_flag = 0.")

        self.uturn_flag_pub.publish(self.uturn_flag)

        return []

    def obstacle_callback(self, msg):
        current_time = time.time()

        for i in range(0, len(msg.data), 8):  # 4개 점 * 2 좌표(x, y) = 8
            coords = msg.data[i:i+8]
            if len(coords) < 8:
                rospy.logwarn(f"Incomplete obstacle data received: {coords}")
                continue  # 데이터 부족 시 무시

            x_coords = coords[0::2]
            y_coords = coords[1::2]

            corners = list(zip(x_coords, y_coords))
            # 중심점 계산
            center_x, center_y = np.mean(x_coords), np.mean(y_coords)

            obs_id = (center_x, center_y)

            with self.obstacles_lock:
                self.obstacles[obs_id] = {
                    'center_x': center_x,
                    'center_y': center_y,
                    'corners': corners,
                    'timestamp': current_time
                }

    def ros_spin_thread(self):
        rospy.spin()

    def run(self):
        # ROS 노드 초기화
        rospy.init_node('uturn_node', anonymous=True)

        # 퍼블리셔 설정 (정수형 토픽 /uturn_flag)
        self.uturn_flag_pub = rospy.Publisher('/uturn_flag', Int32, queue_size=10)

        # Subscriber 설정
        rospy.Subscriber('/obstacle_info', Float32MultiArray, self.obstacle_callback)
        
        # 모드 서브스크라이버 설정
        rospy.Subscriber('/mode_topic', Int32, self.mode_callback)

        # ROS 스피닝을 별도의 스레드에서 실행
        ros_thread = threading.Thread(target=self.ros_spin_thread)
        ros_thread.daemon = True
        ros_thread.start()

        # 애니메이션 시작
        ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=False, interval=100)
        plt.show()

if __name__ == '__main__':
    detector = UturnDetector()
    detector.run()

