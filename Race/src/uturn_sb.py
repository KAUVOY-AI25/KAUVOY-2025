# ================================================================================
# Main Author: 윤수빈
# Recently Modified Date: 2024-10-27
# Dependency: race_kcity.py, lidar (DBSCAN)
# Description: 예선 유턴 알고리즘(장애물이 인식될 경우 고정 스티어링) 
#		       mode_topic 받아와서 3일 때에만 uturn_flag = 1
# ================================================================================

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
        self.obstacles, self.obstacles_lock = {}, threading.Lock()
        self.timeout_duration = 0.4
        self.last_obstacle_time = time.time()

        self.roi_x_max, self.roi_x_min = 5.5, 0
        self.roi_y_max, self.roi_y_min = 1.5, -0.5

        self.uturn_flag = 0
        self.uturn_flag_pub = None
        self.race_mode = None

        self.fig, self.ax = plt.subplots()
        self.init_plot()

        # 추가된 변수들
        self.transition_to_5 = False
        self.transition_time = None

    def mode_callback(self, msg):
        previous_mode = self.race_mode
        self.race_mode = msg.data

        # mode_topic이 4에서 5로 바뀔 때 타이머 시작
        if previous_mode == 4 and self.race_mode == 5:
            self.uturn_flag = 2
            self.transition_to_5 = True
            self.transition_time = time.time()
            rospy.loginfo("Mode transitioned from 4 to 5, setting uturn_flag to 2 for 3 seconds.")

    def init_plot(self):
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-1.5, 6)
        self.ax.grid(True)
        self.ax.set_title('LIDAR Based U-Turn')
        self.ax.set_xlabel('Y (from LIDAR)')
        self.ax.set_ylabel('X (from LIDAR)')
        self.ax.invert_xaxis()

        car_width, car_length = 1.2, 1.8
        car_rect = plt.Rectangle((-car_width / 2, -car_length), car_width, car_length,
                                 linewidth=2, edgecolor='blue', facecolor='none', label='Car')
        self.ax.add_patch(car_rect)

        self.ax.axvline(x=0, color='cyan', linestyle='--', label='x=0 line')
        self.ax.plot(0, 0, 'ro', label='LiDAR Position and Steering Angle')

    def update_plot(self, frame):
        current_time = time.time()

        # 3초 후 uturn_flag를 1004로 고정
        if self.transition_to_5 and current_time - self.transition_time >= 3:
            self.uturn_flag = 1004
            self.transition_to_5 = False
            rospy.loginfo("3 seconds passed after transitioning to mode 5, setting uturn_flag to 1004 permanently.")

        self.ax.clear()
        self.init_plot()

        roi_rect = plt.Rectangle((self.roi_y_min, self.roi_x_min), self.roi_y_max - self.roi_y_min, self.roi_x_max - self.roi_x_min,
                                 linewidth=2, edgecolor='r', facecolor='none', label='ROI')
        self.ax.add_patch(roi_rect)

        uturn_flag_text = f"Race Mode: {self.race_mode}, uturn_flag: {self.uturn_flag}"
        self.ax.text(1.8, 4.5, uturn_flag_text, fontsize=10, color='blue',
                     bbox=dict(facecolor='white', alpha=0.6))

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
                if len(corners) == 4:
                    polygon = Polygon([(corner[1], corner[0]) for corner in corners], closed=True, edgecolor='green', facecolor='none', linewidth=1)
                    self.ax.add_patch(polygon)

                if self.race_mode == 4:
                    if self.roi_x_min <= data['center_x'] <= self.roi_x_max and self.roi_y_min <= data['center_y'] <= self.roi_y_max:
                        self.ax.text(data['center_y'], data['center_x'], 'Closest', color='red')
                        if self.uturn_flag != 1:
                            self.uturn_flag = 1
                            print("장애물이 ROI내에 들어왔고, Mode 3. uturn_flag = 1")
                elif self.race_mode != 5:
                    self.uturn_flag = 0
                    print(f"현재 Mode {self.race_mode}. uturn_flag = 0.")

        self.uturn_flag_pub.publish(self.uturn_flag)
        return []

    def obstacle_callback(self, msg):
        current_time = time.time()

        for i in range(0, len(msg.data), 8):
            coords = msg.data[i:i+8]
            if len(coords) < 8:
                rospy.logwarn(f"Incomplete obstacle data received: {coords}")
                continue

            x_coords = coords[0::2]
            y_coords = coords[1::2]

            corners = list(zip(x_coords, y_coords))
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
        rospy.init_node('uturn_node', anonymous=True)
        self.uturn_flag_pub = rospy.Publisher('/uturn_flag', Int32, queue_size=10)

        rospy.Subscriber('/obstacle_info', Float32MultiArray, self.obstacle_callback)
        rospy.Subscriber('/mode_topic', Int32, self.mode_callback)

        ros_thread = threading.Thread(target=self.ros_spin_thread)
        ros_thread.daemon = True
        ros_thread.start()

        ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=False, interval=100)
        plt.show()

if __name__ == '__main__':
    detector = UturnDetector()
    detector.run()
