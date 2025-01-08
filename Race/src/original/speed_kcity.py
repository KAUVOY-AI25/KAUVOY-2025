#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, UInt8
from geometry_msgs.msg import Point

class SpeedController:
    def __init__(self):
        rospy.init_node('speed_controller', anonymous=True)
        
        # Publishers
        self.velocity_pub = rospy.Publisher('/velocity', UInt8, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake_value', Int32, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/obstacle_flag', Int32, self.obstacle_flag_callback)
        rospy.Subscriber('/utm_k_coordinates', Point, self.position_callback)
        
        # Initialize variables
        self.speed_mode = 'V0'  # Default mode
        self.velocity = 15
        self.brake_value = 1
        self.current_point = None  # Name of the current point
        self.brake_start_time = None
        self.brake_duration = 0
        self.brake_applied = False
        
        # Define points with arbitrary coordinates
        self.points = {
            'p1': {'x': 945537.2142277326, 'y': 1915854.4602438337, 'range_x': 2.0, 'range_y': 2.0},
            'p2': {'x': 935568.812423597, 'y': 1915894.0254116408, 'range_x': 2.0, 'range_y': 2.0},
            'p3': {'x': 935715.8246683534, 'y': 1915960.6649203082, 'range_x': 2.0, 'range_y': 2.0},
            'p4': {'x': 935726.6034503337, 'y': 1915982.8715943422, 'range_x': 3.5, 'range_y': 2.0},
            'p5': {'x': 935705.0715675439, 'y': 1915791.2096753416, 'range_x': 3.0, 'range_y': 2.0},
        }
        
        self.rate = rospy.Rate(10)  # 10 Hz

    def obstacle_flag_callback(self, msg):
        if msg.data == 1004:
            self.speed_mode = 'V0'
            self.velocity = 15
            self.current_point = 'Obstacle_Clear'
            rospy.loginfo("Obstacle flag received. Returning to V0(Default).")

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
                        self.velocity = 13
                        self.apply_brake(3)  # Apply brake for 3 seconds
                        rospy.loginfo("Entered p1 range. Mode: V1(Right_Turn)")
                    elif point_name == 'p2':
                        self.speed_mode = 'V0'
                        self.velocity = 15
                        rospy.loginfo("Entered p2 range. Mode: V0(Default)")
                    elif point_name == 'p3':
                        self.speed_mode = 'V2'
                        self.velocity = 7
                        self.apply_brake(3)  # Apply brake for 5 seconds
                        rospy.loginfo("Entered p3 range. Mode: V2(U-Turn)")
                    elif point_name == 'p4':
                        self.speed_mode = 'V0'
                        self.velocity = 15
                        rospy.loginfo("Entered p4 range. Mode: V0(Default)")
                    elif point_name == 'p5':
                        self.speed_mode = 'V3'
                        self.velocity = 6
                        self.apply_brake(4)  # Apply brake for 4 seconds
                        rospy.loginfo("Entered p5 range. Mode: V3(Obstacle)")
                break

        if not point_found and self.current_point not in ['Obstacle_Clear', None]:
            # Remain in current speed mode until next point or flag
            pass

    def apply_brake(self, duration):
        if not self.brake_applied:
            self.brake_value = 15
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
            self.velocity_pub.publish(UInt8(self.velocity))
            self.brake_pub.publish(Int32(self.brake_value))

            # Log current status
            mode_description = {
                'V0': 'V0(Default)',
                'V1': 'V1(Right_Turn)',
                'V2': 'V2(U-Turn)',
                'V3': 'V3(Obstacle)'
            }
            rospy.loginfo(f"Mode: {mode_description[self.speed_mode]}, Velocity: {self.velocity}, Brake: {self.brake_value}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SpeedController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
