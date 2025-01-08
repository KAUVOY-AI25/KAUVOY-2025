# ================================================================================
# Main Author: 윤수빈, 이창현
# Recently Modified Date: 2024-09-29
# Dependency: kcity_mode_0928.py, lidar (DBSCAN)
# Description: 예선 장애물 회피 알고리즘 (장애물 인식 후 flag생성하여, 차선 영상 기반 steering)
# ================================================================================

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import numpy as np
from matplotlib.patches import Polygon

class LidarObstacleDetector:
    def __init__(self):
        self.obstacles, self.obstacles_lock = {}, threading.Lock()  # 장애물 정보에 대한 스레드 락
        self.timeout_duration = 0.2  # 장애물 정보가 유효하다고 간주할 수 있는 시간 제한 (초 단위)

        # 관심 영역(ROI) 설정
        self.roi_x_min, self.roi_x_max = -1.5, 5.0   # 라이다 기준 앞뒤(+-)
        self.roi_y_min, self.roi_y_max = -2.0, 2.0   # 라이다 기준 좌우(+-)

        # 장애물 플래그 및 카운트 변수
        self.obstacle_flag = 0  # 현재 가장 가까운 장애물 위치 ( -1 | 0 | 1 = 왼쪽 | 없음(잡히기 전) | 오른쪽 -> 1004 장해물 통과 완료) 
        self.left_obstacle_Pass_count, self.right_obstacle_Pass_count = 0, 0  # 장애물이 x <= 0을 지난 순간 카운트

        # 장애물 통과 여부 플래그
        self.left_obstacle_Passed, self.right_obstacle_Passed = False, False

        # 회피 모드 정의
        self.avoidance_mode = None  # 초기에는 미설정 (왼쪽 먼저? 오른쪽 먼저?)

        # 현재 모드
        self.race_mode = 0  # 모드 저장

        # ROS 퍼블리셔
        self.obstacle_flag_pub = None

        # Matplotlib 초기화
        self.fig, self.ax = plt.subplots()
        self.init_plot()

        # ROS subscriber to /mode_topic
        rospy.Subscriber('/mode_topic', Int32, self.mode_callback)

    def mode_callback(self, msg):
        # 모드 값을 업데이트하는 콜백
        self.race_mode = msg.data
        rospy.loginfo(f"Mode updated to: {self.race_mode}")

    def init_plot(self):
        self.ax.set_xlim(-2, 2)  # Y축 범위 설정
        self.ax.set_ylim(-1.5, 5)  # X축 범위 설정
        self.ax.grid(True)  # 그리드 표시
        self.ax.set_title('LIDAR Based Obstacle Detection')  # 플롯 제목 설정
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
        
                # ----------------------------- [장애물 회피 메인 알고리즘] -----------------------------
                # 왼쪽, 오른쪽 장애물 모두 통과한 상태 -> 장애물 회피 완료
                if self.left_obstacle_Passed and self.right_obstacle_Passed:
                    print("장애물 미션 끝! 마무리 주행 시작!", "flag는", self.obstacle_flag)
                    self.obstacle_flag = 1004
                # ------------------------------------------------------------------------------------
                # <왼쪽 먼저 통과 모드에서, 왼쪽 장애물을 통과한 상태> -> 오른쪽 장애물 회피 필요                
                elif self.avoidance_mode == 'left_obstacle_first' and self.left_obstacle_Passed==True:
                    #  오른쪽 장애물이 아직 안 잡히는 경우 -> 왼쪽 차선으로 먼저 붙어서 주행 신호 
                    self.obstacle_flag = 1
                    print("왼쪽 장애물을 통과! 오른쪽 장애물 회피를 준비!", "flag는", self.obstacle_flag)
                    #  오른쪽 장애물이 잡히는 경우 -> 왼쪽 차선으로 붙어서 주행                
                    if data['center_x'] <= 5 and data['center_y'] < 0:  
                        self.obstacle_flag = 1
                        print("RIGHT OBSTACLE")
                    
                        # 오른쪽 장애물이 라이다 지점(x=0) 통과하는 순간을 감지 (카운터 사용)
                        if not self.right_obstacle_Passed and data['center_x'] <= 0 and data['center_y'] <= 0:
                            self.right_obstacle_Pass_count += 1
                            self.right_obstacle_Passed = True
                            print(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                            rospy.loginfo(f"Right obstacle Passed x=1, count: {self.right_obstacle_Pass_count}")
                            rospy.loginfo(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                            
                # <오른쪽 먼저 통과 모드에서, 오른쪽 장애물을 통과한 상태> -> 왼쪽 장애물 회피 필요                
                elif self.avoidance_mode =='right_obstacle_first' and self.right_obstacle_Passed==True:
                    #  왼쪽 장애물이 아직 안 잡히는 경우 -> 오른쪽 차선으로 먼저 붙어서 주행 신호 
                    self.obstacle_flag = -1 
                    print("오른쪽 장애물을 통과! 왼쪽 장애물 회피를 준비!", "flag는", self.obstacle_flag)
                    #  왼쪽 장애물이 잡히는 경우 -> 오른쪽 차선으로 붙어서 주행         
                    if data['center_x'] <= 5 and data['center_y'] > 0:  # 왼쪽 장애물
                        self.obstacle_flag = -1
                        print("LEFT OBSTACLE")
                
                        # 왼쪽 장애물이 라이다 지점(x=0) 통과하는 순간을 감지 (카운터 사용)
                        if not self.left_obstacle_Passed and data['center_x'] <= 0 and data['center_y'] >= 0:
                            self.left_obstacle_Pass_count += 1
                            self.left_obstacle_Passed = True
                            print(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                            rospy.loginfo(f"Right obstacle Passed x=1, count: {self.left_obstacle_Pass_count}")
                            rospy.loginfo(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                # ------------------------------------------------------------------------------------        
                # <아직 첫 번째 장애물을 통과하지 않은 상태>
                elif closest_obstacle is not None:
                    # 가장 가까운 장애물이 전방 5m 내에 들어왔는지 확인
                    if closest_obstacle['center_x'] <= 5:                       
                        if closest_obstacle['center_y'] < 0:  # 오른쪽 장애물
                            self.obstacle_flag = 1
                            print("RIGHT OBSTACLE")
                    
                            # 오른쪽 장애물이 x <= 0을 통과하는 순간을 감지하여 카운트 증가
                            if not self.right_obstacle_Passed and closest_obstacle['center_x'] <= 0.6:
                                self.right_obstacle_Pass_count += 1
                                self.right_obstacle_Passed = True
                                print(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                                rospy.loginfo(f"Right obstacle Passed x=1, count: {self.right_obstacle_Pass_count}")
                                rospy.loginfo(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                    
                        elif closest_obstacle['center_y'] > 0:  # 왼쪽 장애물
                            self.obstacle_flag = -1
                            print("LEFT OBSTACLE")
                    
                            # 왼쪽 장애물이 x <= 0을 통과하는 순간을 감지하여 카운트 증가
                            if not self.left_obstacle_Passed and closest_obstacle['center_x'] <= 0.6:
                                self.left_obstacle_Pass_count += 1
                                self.left_obstacle_Passed = True
                                print(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                                rospy.loginfo(f"Left obstacle Passed x=1, count: {self.left_obstacle_Pass_count}")
                                rospy.loginfo(f"LEFT Obstacle Passed: {self.left_obstacle_Passed} / RIGHT Obstacle Passed: {self.right_obstacle_Passed}")
                # ------------------------------------------------------------------------------------                
                    self.ax.text(closest_obstacle['center_y'], closest_obstacle['center_x'], 'Closest', color='red')
        # 범례에 현재 상태 표시
        legend_text = (f"Race Mode: {self.race_mode}\n"
                    f"Avodiance Mode: {self.avoidance_mode}\n"
                    f"Obstacle Flag: {self.obstacle_flag}\n"
                    f"Left Passed: {self.left_obstacle_Passed}/" f"Right Passed: {self.right_obstacle_Passed}")

        # 텍스트 박스를 그리기
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        self.ax.text(0.05, 0.95, legend_text, transform=self.ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)
        
        # ROS에 obstacle_flag Publish -> kcity.py 
        self.obstacle_flag_pub.publish(self.obstacle_flag)
        
        return []
    
    def obstacle_callback(self, msg):
        if self.race_mode == 6:
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
        
                # 장애물이 관심 영역(ROI) 내에 있는지 확인
                if self.roi_x_min <= center_x <= self.roi_x_max and self.roi_y_min <= center_y <= self.roi_y_max:
                    obs_id = (center_x, center_y)
        
                    with self.obstacles_lock:
                        self.obstacles[obs_id] = {
                            'center_x': center_x,
                            'center_y': center_y,
                            'corners': corners,
                            'timestamp': current_time
                        }
        
                    # 회피 모드가 아직 설정되지 않은 경우 설정
                    if self.avoidance_mode is None:
                        if center_y > 0:  # 왼쪽 장애물이 가까운 경우
                            self.avoidance_mode = 'left_obstacle_first'
                            rospy.loginfo("Set mode to: 왼쪽 장애물 먼저 회피")
                        elif center_y < 0:  # 오른쪽 장애물이 가까운 경우
                            self.avoidance_mode = 'right_obstacle_first'
                            rospy.loginfo("Set mode to: 오른쪽 장애물 먼저 회피")
        
        else:
            print("모드가 6이 아니기 때문에, static_obstacle_flag를 띄울 수 없음")   
    
    def ros_spin_thread(self):
        rospy.spin()
    
    def run(self):
        # ROS 노드 초기화
        rospy.init_node('lidar_obstacle_detection', anonymous=True)
        
        # 퍼블리셔 설정 (정수형 토픽 /static_obstacle_flag)
        self.obstacle_flag_pub = rospy.Publisher('/static_obstacle_flag', Int32, queue_size=10)
    
        # Subscriber 설정
        rospy.Subscriber('/obstacle_info', Float32MultiArray, self.obstacle_callback)
        
        # ROS 스피닝을 별도의 스레드에서 실행
        ros_thread = threading.Thread(target=self.ros_spin_thread)
        ros_thread.daemon = True
        ros_thread.start()
        
        # 애니메이션 시작
        ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=False, interval=100)
        plt.show()

if __name__ == '__main__':
    detector = LidarObstacleDetector()
    detector.run()
