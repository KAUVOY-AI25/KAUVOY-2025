#!/usr/bin/env python3
# ================================================================================
# Main Author: 이창현, 박민아, 손유정, 이종원, 최민서
# Recently Modified Date: 2024-09-22
# Dependency:  ublox_gps with RTK Service
# Description: utmk 좌표변환 알고리즘 (imu  동기화 삭제 버전)
# ================================================================================

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Vector3Stamped
from pyproj import Proj, transform
import math


class SensorDataProcessor:
    def __init__(self):
        self.last_time = None
        self.last_x = None
        self.last_y = None
        self.velocity_pub = rospy.Publisher("/velocity", Vector3Stamped, queue_size=10)
        self.gps_velocity_pub = rospy.Publisher("/gpsvelocity", Vector3Stamped, queue_size=10)
        self.gps_pub = rospy.Publisher("/utm_k_coordinates", Point, queue_size=10)
        # Removed IMU-related publishers
        # self.imu_pub = rospy.Publisher("/imu_orientation", PoseWithCovarianceStamped, queue_size=10)
        # self.imu_combined_pub = rospy.Publisher("/imu_combined_data", Imu, queue_size=10)

    def calculate_velocity(self, current_x, current_y, current_time):
        if self.last_time is not None and self.last_x is not None and self.last_y is not None:
            time_delta = (current_time - self.last_time).to_sec()
            if time_delta > 0:
                velocity_x = (current_x - self.last_x) / time_delta
                velocity_y = (current_y - self.last_y) / time_delta
                total_velocity = math.sqrt(velocity_x**2 + velocity_y**2)

                velocity_msg = Vector3Stamped()
                velocity_msg.header.stamp = current_time
                velocity_msg.vector.x = total_velocity  # X에 total velocity 값을 넣음
                velocity_msg.vector.y = 0  # Y 및 Z는 0으로 설정
                velocity_msg.vector.z = 0
                self.gps_velocity_pub.publish(velocity_msg)

                rospy.loginfo("Total velocity: %f m/s", total_velocity)

        self.last_x = current_x
        self.last_y = current_y
        self.last_time = current_time

    def callback(self, gps_msg):
        wgs84 = Proj(init='epsg:4326')
        utm_k = Proj(init='epsg:5179')
        x, y = transform(wgs84, utm_k, gps_msg.longitude, gps_msg.latitude)
        current_time = gps_msg.header.stamp

        # Velocity calculation
        self.calculate_velocity(x, y, current_time)

        # Publishing transformed GPS coordinates
        point_msg = Point(x=x, y=y, z=0)
        self.gps_pub.publish(point_msg)

        rospy.loginfo("UTM-K: X: %f, Y: %f" % (x, y))
        # Removed IMU-related logging
        # rospy.loginfo("IMU Orientation x: %f, y: %f, z: %f, w: %f" %
        #               (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w))
        #
        # rospy.loginfo("IMU Linear Acceleration x: %f, y: %f, z: %f" %
        #               (imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z))
        # rospy.loginfo("IMU Angular Velocity x: %f, y: %f, z: %f" %
        #               (imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z))
        # rospy.loginfo("IMU Combined Data Orientation x: %f, y: %f, z: %f, w: %f, xl: %f, yl: %f, zl: %f, xa: %f, ya: %f, za: %f" %
        #               (imu_combined_msg.orientation.x, imu_combined_msg.orientation.y, imu_combined_msg.orientation.z, imu_combined_msg.orientation.w, imu_combined_msg.linear_acceleration.x, imu_combined_msg.linear_acceleration.y, imu_combined_msg.linear_acceleration.z, imu_combined_msg.angular_velocity.x, imu_combined_msg.angular_velocity.y, imu_combined_msg.angular_velocity.z))


def listener():
    rospy.init_node('sensor_sync_and_convert_node', anonymous=True)
    processor = SensorDataProcessor()

    # Subscribe only to GPS topic
    gps_sub = rospy.Subscriber('ublox_gps/fix', NavSatFix, processor.callback)

    # Removed IMU subscription and message_filters
    # gps_sub = message_filters.Subscriber('ublox_gps/fix', NavSatFix)
    # imu_sub = message_filters.Subscriber('vectornav/IMU', Imu)
    # ats = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], queue_size=10, slop=0.1)
    # ats.registerCallback(processor.callback)

    rospy.spin()


if __name__ == '__main__':
    listener()

