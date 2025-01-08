# ================================================================================
# Main Author: 윤수빈
# Recently Modified Date: 2024-10-09
# Dependency: static_obstacle
# Description: 정적 장애물 후, 동적 장애물이 감지될 경우 1111 flag, 장애물이 없어지면 1004 flag로 변경 후 정속 주행
# ------- 10-13 K-CITY 실험완료 ------
# ================================================================================


import rospy
from std_msgs.msg import Float32MultiArray, Int32
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
        
        # 장애물 플래그 및 타이머 변수
        self.dynamic_obstacle_flag = 0
        self.static_obstacle_flag = 0
        self.last_obstacle_time = None
        self.obstacle_clear_time = 4  # 4초
        
        # ROS 퍼블리셔
        self.dynamic_obstacle_flag_pub = None
        self.flag_locked = False  # 플래그가 1004가 된 후 더 이상 변하지 않도록 하기 위한 잠금 플래그

        # Matplotlib 초기화
        self.fig, self.ax = plt.subplots()
        self.init_plot()

    def static_obstacle_flag_callback(self, msg):
        self.static_obstacle_flag = msg.data
        rospy.loginfo(f"정적장애물 통과: {self.static_obstacle_flag}")

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
        if self.static_obstacle_flag == 1000 and not self.flag_locked:
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
                    
                    # 장애물 감지 시 1111 플래그 퍼블리시
                    self.dynamic_obstacle_flag = 1111
                    self.dynamic_obstacle_flag_pub.publish(self.dynamic_obstacle_flag)
                    self.last_obstacle_time = current_time  # 마지막 장애물 감지 시간 업데이트
                    rospy.loginfo("동적 장애물이 감지되었습니다. flag = 1111")
        
        else:
            rospy.loginfo("정적 장애물 통과를 완료했습니다. flag=1000 고정")
            
    def update_plot(self, frame):
        current_time = time.time()

        if self.last_obstacle_time and not self.flag_locked:
            # 1111 플래그가 퍼블리시된 이후 4초 동안 장애물이 없으면 플래그를 1004로 설정
            if current_time - self.last_obstacle_time > self.obstacle_clear_time:
                self.dynamic_obstacle_flag = 1004
                self.dynamic_obstacle_flag_pub.publish(self.dynamic_obstacle_flag)
                self.flag_locked = True  # 플래그를 잠금 처리하여 더 이상 변경되지 않도록 함
                rospy.loginfo("4초동안 동적 장애물이 감지되지 않아, flag=1004 고정")

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

        legend_text = (f"Static Obstacle Flag: {self.static_obstacle_flag}\n"
                        f"Dynamic Obstacle Flag: {self.dynamic_obstacle_flag}")

        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        self.ax.text(0.05, 0.95, legend_text, transform=self.ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)

        return []

    def ros_spin_thread(self):
        rospy.spin()
    
    def run(self):
        # ROS 노드 초기화
        rospy.init_node('lidar_obstacle_detection', anonymous=True)
        
        # 퍼블리셔 설정 (정수형 토픽 /dynamic_obstacle_flag)
        self.dynamic_obstacle_flag_pub = rospy.Publisher('/dynamic_obstacle_flag', Int32, queue_size=10)
    
        # Subscriber 설정
        rospy.Subscriber('/static_obstacle_flag', Int32, self.static_obstacle_flag_callback)
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
