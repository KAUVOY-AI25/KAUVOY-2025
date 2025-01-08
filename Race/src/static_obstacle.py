# ================================================================================
# Main Author: 윤수빈, 이창현
# Recently Modified Date: 2024-10-13
# Dependency: kcity_mode_0928.py, lidar (DBSCAN)
# Description: 예선 장애물 회피 알고리즘 (장애물 인식 후 flag생성하여, 차선 영상 기반 steering)
# -- 10/13 K-CITY 실험 완료 --
# ================================================================================
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
        self.obstacles, self.obstacles_lock = {}, threading.Lock()
        self.timeout_duration = 0.2
        
        # 관심 영역(ROI) 설정
        self.roi_x_min, self.roi_x_max = -1.5, 5.0
        self.roi_y_min, self.roi_y_max = -2.0, 2.0
        
        # 장애물 플래그 및 카운트 변수
        self.obstacle_flag = 0
        self.left_obstacle_Pass_count, self.right_obstacle_Pass_count = 0, 0
        self.left_passed_time = None

        # 장애물 통과 여부 플래그
        self.left_obstacle_Passed, self.right_obstacle_Passed = False, False
        self.static_finish = False

        # 회피 모드 정의
        self.avoidance_mode = None
        
        # 현재 모드
        self.race_mode = 0
        
        # 2초 타이머 변수
        self.obstacle_pass_time = None
        self.publish_110_or_011 = False
        self.publish_99_once = False  
        
        #1. 장애물 위치 판단 (left_obstacle_first / right_obstacle_first)
	#2. 장애물 하나 통과 후 스티링 조정 (99)
	#3. 남아있는 장애물 (-1/1)
	#4. 장애물 모두 통과 후 스티어링 조정 (110 / 11)
	#5. 정적 장애물 통과 완료 (1000)

        # ROS 퍼블리셔
        self.obstacle_flag_pub = None

        # Matplotlib 초기화
        self.fig, self.ax = plt.subplots()
        self.init_plot()

        # ROS subscriber to /mode_topic
        rospy.Subscriber('/mode_topic', Int32, self.mode_callback)

    def mode_callback(self, msg):
        self.race_mode = msg.data
        rospy.loginfo(f"Mode updated to: {self.race_mode}")

    def init_plot(self):
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-1.5, 5)
        self.ax.grid(True)
        self.ax.set_title('LIDAR Based Obstacle Detection')
        self.ax.set_xlabel('Y (from LIDAR)')
        self.ax.set_ylabel('X (from LIDAR)')
        self.ax.invert_xaxis()
        
        car_width, car_length = 1.2, 1.8
        car_rect = plt.Rectangle((-car_width / 2, -car_length), car_width, car_length,
                                 linewidth=2, edgecolor='blue', facecolor='none', label='Car')
        self.ax.add_patch(car_rect)
        self.ax.axvline(x=0, color='cyan', linestyle='--', label='x=0 line')
        self.ax.plot(0, 0, 'ro', label='LiDAR Position and Steering Angle')

    def obstacle_callback(self, msg):
        if self.race_mode == 7:
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
            print("모드가 7이 아니기 때문에, static_obstacle_flag를 띄울 수 없음")   
            
    def update_plot(self, frame):
        current_time = time.time()
        closest_obstacle, closest_distance = None, float('inf')
        self.ax.clear()
        self.init_plot()

        roi_rect = plt.Rectangle((self.roi_y_min, self.roi_x_min), self.roi_y_max - self.roi_y_min, self.roi_x_max - self.roi_x_min,
                                linewidth=2, edgecolor='r', facecolor='none', label='ROI')
        self.ax.add_patch(roi_rect)

        with self.obstacles_lock:
            active_obstacles = {}
            for obs_id, data in list(self.obstacles.items()):
                if current_time - data['timestamp'] > self.timeout_duration:
                    del self.obstacles[obs_id]
                    continue

                active_obstacles[obs_id] = data
                self.ax.plot(data['center_y'], data['center_x'], 'bo', markersize=4)
                corners = data['corners']
                for corner in corners:
                    self.ax.plot(corner[1], corner[0], 'go', markersize=4)

                distance_to_lidar = abs(data['center_x'])
                if distance_to_lidar < closest_distance:
                    closest_distance = distance_to_lidar
                    closest_obstacle = data

                if len(corners) == 4:
                    polygon = Polygon([(corner[1], corner[0]) for corner in corners], closed=True, edgecolor='green', facecolor='none', linewidth=1)
                    self.ax.add_patch(polygon)
        

        
        # 타이머 관련 로직 (장애물 여부와 상관없이 4초 후에 1000으로 전환)
        if self.left_obstacle_Passed and self.right_obstacle_Passed:
            if self.obstacle_pass_time is None:
                self.obstacle_pass_time = current_time
                self.publish_110_or_011 = True

            elapsed_time = current_time - self.obstacle_pass_time

            if elapsed_time <= 2.0 and not self.static_finish:  # 1000으로 전환되었으면 더 이상 110/11 플래그 나오지 않도록
                if self.avoidance_mode == 'right_obstacle_first':
                    self.obstacle_flag = 11
                    rospy.loginfo("스티어링을 왼쪽으로 조정 중입니다.")
                elif self.avoidance_mode == 'left_obstacle_first':
                    self.obstacle_flag = 110
                    rospy.loginfo("스티어링을 오른쪽으로 조정 중입니다.")
            elif not self.static_finish:
                # 장애물 유무와 상관없이 4초 후에는 1000으로 전환
                self.obstacle_flag = 1000
                rospy.loginfo("4초 후 플래그를 1000으로 전환합니다.")
                
                # 타이머와 관련 변수 리셋
                self.publish_110_or_011 = False
                self.obstacle_pass_time = None
                self.static_finish = True  # 장애물 통과가 완료됨을 표시
        
        # else:
        #     # 기존 로직 수행
        #     if not self.publish_110_or_011:
        #         if self.avoidance_mode == 'left_obstacle_first' and self.left_obstacle_Passed:
        #             self.obstacle_flag = 1
        #             if closest_obstacle and closest_obstacle['center_x'] <= 5 and closest_obstacle['center_y'] < 0:
        #                 self.obstacle_flag = 1
        #                 if not self.right_obstacle_Passed and closest_obstacle['center_x'] <= 0 and closest_obstacle['center_y'] <= 0:
        #                     self.right_obstacle_Pass_count += 1
        #                     self.right_obstacle_Passed = True
        #         elif self.avoidance_mode == 'right_obstacle_first' and self.right_obstacle_Passed:
        #             self.obstacle_flag = -1
        #             if closest_obstacle and closest_obstacle['center_x'] <= 5 and closest_obstacle['center_y'] > 0:
        #                 self.obstacle_flag = -1
        #                 if not self.left_obstacle_Passed and closest_obstacle['center_x'] <= 0 and closest_obstacle['center_y'] >= 0:
        #                     self.left_obstacle_Pass_count += 1
        #                     self.left_obstacle_Passed = True
        #         elif closest_obstacle is not None:
        #             if closest_obstacle['center_x'] <= 5:
        #                 if closest_obstacle['center_y'] < 0:
        #                     self.obstacle_flag = 1
        #                     if not self.right_obstacle_Passed and closest_obstacle['center_x'] <= 0.6:
        #                         self.right_obstacle_Pass_count += 1
        #                         self.right_obstacle_Passed = True
        #                 elif closest_obstacle['center_y'] > 0:
        #                     self.obstacle_flag = -1
        #                     if not self.left_obstacle_Passed and closest_obstacle['center_x'] <= 0.6:
        #                         self.left_obstacle_Pass_count += 1
        #                         self.left_obstacle_Passed = True
        else:
            # 기존 로직 수행
            if not self.publish_110_or_011:
                if self.avoidance_mode == 'left_obstacle_first' and self.left_obstacle_Passed:
                    # left_obstacle_Passed가 True가 된 시간을 기록하고 1.5초 동안 obstacle_flag = 99
                    if self.left_passed_time is None:  # 처음 True가 될 때 시간을 기록
                        self.left_passed_time = time.time()

                    # 1.5초가 경과했는지 체크
                    if time.time() - self.left_passed_time <= 1.5:
                        self.obstacle_flag = 99
                    else:
                        self.obstacle_flag = 1  # 1.5초가 지나면 원래 로직으로 돌아감

                    if closest_obstacle and closest_obstacle['center_x'] <= 5 and closest_obstacle['center_y'] < 0:
                        self.obstacle_flag = 1
                        if not self.right_obstacle_Passed and closest_obstacle['center_x'] <= 0 and closest_obstacle['center_y'] <= 0:
                            self.right_obstacle_Pass_count += 1
                            self.right_obstacle_Passed = True
                elif self.avoidance_mode == 'right_obstacle_first' and self.right_obstacle_Passed:
                    self.obstacle_flag = -1
                    if closest_obstacle and closest_obstacle['center_x'] <= 5 and closest_obstacle['center_y'] > 0:
                        self.obstacle_flag = -1
                        if not self.left_obstacle_Passed and closest_obstacle['center_x'] <= 0 and closest_obstacle['center_y'] >= 0:
                            self.left_obstacle_Pass_count += 1
                            self.left_obstacle_Passed = True
                elif closest_obstacle is not None:
                    if closest_obstacle['center_x'] <= 5:
                        if closest_obstacle['center_y'] < 0:
                            self.obstacle_flag = 1
                            if not self.right_obstacle_Passed and closest_obstacle['center_x'] <= 0.6:
                                self.right_obstacle_Pass_count += 1
                                self.right_obstacle_Passed = True
                        elif closest_obstacle['center_y'] > 0:
                            self.obstacle_flag = -1
                            if not self.left_obstacle_Passed and closest_obstacle['center_x'] <= 0.6:
                                self.left_obstacle_Pass_count += 1
                                self.left_obstacle_Passed = True

        legend_text = (f"Race Mode: {self.race_mode}\n"
                    f"Avodiance Mode: {self.avoidance_mode}\n"
                    f"Obstacle Flag: {self.obstacle_flag}\n"
                    f"Left Passed: {self.left_obstacle_Passed}/" f"Right Passed: {self.right_obstacle_Passed}")

        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        self.ax.text(0.05, 0.95, legend_text, transform=self.ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)

        self.obstacle_flag_pub.publish(self.obstacle_flag)
        
        return []


    
    
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
