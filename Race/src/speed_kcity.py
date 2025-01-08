#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Point

class SpeedController:
    def __init__(self):
        rospy.init_node('speed_controller', anonymous=True)
        
        # Publishers
        self.velocity_pub = rospy.Publisher('/velocity', Float64, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake_value', Float64, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/dynamic_obstacle_flag', Int32, self.obstacle_flag_callback)
        rospy.Subscriber('/utm_k_coordinates', Point, self.position_callback)
        
        # Initialize variables
        self.speed_mode = 'V0'  # Default mode
        self.velocity = 10
        self.brake_value = 1
        self.current_point = None  # Name of the current point
        self.brake_start_time = None
        self.brake_duration = 0
        self.brake_applied = False
        
        # Define points with arbitrary coordinates
        self.points = {
            'p1': {'x': 935535.4343842492, 'y': 1915853.7083747445, 'range_x': 2.0, 'range_y': 2.0}, # 우회전 전에 감속
            'p2': {'x': 935568.812423597, 'y': 1915894.0254116408, 'range_x': 2.0, 'range_y': 2.0}, # 다시 정속 주행
            # 'p3': {'x': 935712.4391184604, 'y': 1915903.1095250512, 'range_x': 2.0, 'range_y': 2.0}, # uturn 전에 감속
            'p3': {'x': 935715.9626123841, 'y': 1915927.7016128052, 'range_x': 2.0, 'range_y': 2.0}, # uturn 전에 감속
            # 'p4': {'x': 935724.6294522777, 'y': 1915982.2432133642, 'range_x': 3.5, 'range_y': 2.0}, # 다시 정속 주행
            'p4': {'x': 935724.1866835199, 'y': 1915958.0047577277, 'range_x': 3.5, 'range_y': 2.0}, # 다시 정속 주행
            # 'p5': {'x': 935698.5916912002, 'y': 1915782.5166950258, 'range_x': 3.0, 'range_y': 2.0}, # 장애물 전에 감속
            'p5': {'x': 935709.4041781367, 'y': 1915796.3687378326, 'range_x': 3.0, 'range_y': 2.0}, # 장애물 전에 감속
        }
        
        self.rate = rospy.Rate(10)  # 10 Hz

    def obstacle_flag_callback(self, msg):
        if msg.data == 1111:
            self.speed_mode = 'V4'
            self.apply_brake(5)  # Apply brake for 3 seconds
            self.velocity = 0
            self.current_point = 'Obstacle_Clear'
            dynamic_pass = True
            rospy.loginfo("동적 장애물 감지 되었습니다 - 정지")
        elif msg.data == 1004:
            self.speed_mode = 'V0'
            self.velocity = 10
            self.current_point = 'Obstacle_Clear'
            rospy.loginfo("동적 장애물 이동 완료 - 정상 주행")


    def position_callback(self, msg):
        x = msg.x
        y = msg.y
        point_found = False

        for point_name, point_info in self.points.items():
            if (point_info['x'] - point_info['range_x'] <= x <= point_info['x'] + point_info['range_x'] and
                point_info['y'] - point_info['range_y'] <= y <= point_info['y'] + point_info['range_y']):
                point_found = True
                if self.current_point != point_name:
                    self.current_point = point_name
                    self.brake_applied = False  # Reset brake for new point

                    if point_name == 'p1':
                        self.speed_mode = 'V1'
                        self.velocity = 10
                        self.apply_brake(2)  # Apply brake for 3 seconds
                        rospy.loginfo("p1 진입 - V1: 우회전 구간 감속")
                    elif point_name == 'p2':
                        self.speed_mode = 'V0'
                        self.velocity = 10
                        rospy.loginfo("p2 진입 - V0: 정상 속도 주행")
                    elif point_name == 'p3':
                        self.speed_mode = 'V2'
                        self.velocity = 10
                        self.apply_brake(1)  # Apply brake for 5 seconds
                        rospy.loginfo("p3 진입 - V2: U-turn 구간 감속")
                    elif point_name == 'p4':
                        self.speed_mode = 'V0'
                        self.velocity = 10
                        rospy.loginfo("p4 진입 - V0: 정상 속도 주행")
                    elif point_name == 'p5':
                        self.speed_mode = 'V3'
                        self.velocity = 6
                        self.apply_brake(2)  # Apply brake for 4 seconds
                        rospy.loginfo("p5 진입 - V3: 터널 주행 및 장애물 회피를 위한 감속")
                break

        if not point_found and self.current_point not in ['Obstacle_Clear', None]:
            # Remain in current speed mode until next point or flag
            pass

    def apply_brake(self, duration):
        if not self.brake_applied:
            self.brake_value = 3
            self.brake_start_time = rospy.get_time()
            self.brake_duration = duration
            self.brake_applied = True
            rospy.loginfo(f"Brake applied with value 10 for {duration} seconds.")

    def run(self):
        while not rospy.is_shutdown():
            # Handle brake timing
            if self.brake_applied:
                elapsed_time = rospy.get_time() - self.brake_start_time
                if elapsed_time >= self.brake_duration:
                    self.brake_value = 1  # Reset brake value after duration
                    self.brake_applied = False
                    rospy.loginfo("Brake released.")

            # Publish velocity and brake_value
            self.velocity_pub.publish(Float64(self.velocity))
            self.brake_pub.publish(Float64(self.brake_value))

            # Log current status
            mode_description = {
                'V0': 'V0(Default)',
                'V1': 'V1(Right_Turn)',
                'V2': 'V2(U-Turn)',
                'V3': 'V3(정적 & 동적 장애물 준비)',
                'V4': 'V4(동적 장애물 감지)'              
            }
            rospy.loginfo(f"Mode: {mode_description[self.speed_mode]}, Velocity: {self.velocity}, Brake: {self.brake_value}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SpeedController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
