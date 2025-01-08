# ================================================================================
# Main Author: 이창현
# Recently Modified Date: 2024-10-06
# Dependency: roslaunch utmk utmk.launch
# Description: GPS 위치에 따른 KCITY 주행 모드 설정
# ================================================================================

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import math

class LocationChecker:
    def __init__(self):
        # 미리 설정해 둔 UTM-K 좌표 리스트 (여러 좌표 설정 가능)
        self.predefined_points = [
            #{'x': 935555.3217148576, 'y': 1915888.3818145876, 'id': 1, 'publish': 1, 'mode': "Mode 1"},  # 우회전 시작 < --- 수정 필요'
            #{'x': 935553.0245516023, 'y': 1915886.1029283544, 'id': 1, 'publish': 1, 'mode': "Mode 1"},  # 우회전 시작 화살표 끝에서 우회전 시작
            {'x': 935555.4210703123, 'y': 1915888.6141207975, 'id': 1, 'publish': 1, 'mode': "Mode 1"},  # 우회전 시작 정지선에서 우회전 시작
            {'x': 935565.822435323, 'y': 1915895.091252599, 'id': 2, 'publish': 0, 'mode': "Mode 0"},  # 우회전 끝   < --- 넓힙시다
            #{'x': 935568.812423597, 'y': 1915894.0254116408, 'id': 2, 'publish': 0, 'mode': "Mode 0"},  # 우회전 끝   < --- 넓힙시다       
            {'x': 935585.9351096852, 'y': 1915885.1625464286, 'id': 3, 'publish': 2, 'mode': "Mode 2"},  # 직진 시작
            {'x': 935604.965713788, 'y': 1915875.2421872846, 'id': 4, 'publish': 0, 'mode': "Mode 0"},   # 직진 끝
            
            {'x': 935712.4391184604, 'y': 1915903.1095250512, 'id': 10, 'publish': 3, 'mode': "Mode 3"},   # uturn하러 가기 전에 roi 줄이는 부분
            #{'x': 935715.8246683534, 'y': 1915960.6649203082, 'id': 5, 'publish': 4, 'mode': "Mode 4"},   # 유턴 
            #{'x': 935715.7802333442, 'y': 1915942.1149793898, 'id': 5, 'publish': 4, 'mode': "Mode 4"},   # 유턴
            {'x': 935715.9626123841, 'y': 1915927.7016128052, 'id': 5, 'publish': 4, 'mode': "Mode 4"},   # 유턴  
            {'x': 935724.1866835199, 'y': 1915958.0047577277, 'id': 6, 'publish': 5, 'mode': "Mode 5"}, # 유턴 끝
            #{'x': 935724.9131891772, 'y': 1915979.9111616614, 'id': 6, 'publish': 5, 'mode': "Mode 5"}, # 유턴 끝
            
            {'x': 935726.7598293906, 'y': 1915943.0405478114, 'id': 7, 'publish': 0, 'mode': "Mode 0"},   # 일반 주행 (하이패스 통과)
            {'x': 935727.6577212248, 'y': 1915849.4279913339, 'id': 8, 'publish': 6, 'mode': "Mode 6"},   # roi 줄이는 부분
            
            # {'x': 935709.7591437796, 'y': 1915796.3660894986, 'id': 9, 'publish': 7, 'mode': "Mode 7"}    # 장애물
            {'x': 935709.4041781367, 'y': 1915796.3687378326, 'id': 9, 'publish': 7, 'mode': "Mode 7"}    # 장애물
        ]
  
        self.location_flag_pub = rospy.Publisher("/mode_topic", Int32, queue_size=10)
        rospy.Subscriber("/utm_k_coordinates", Point, self.callback)
        
        self.current_mode = 0
        self.last_published = 0
        self.mode7_activated = False  # Mode7가 한 번 활성화되었는지 추적하는 플래그

    def calculate_distance_and_direction(self, current_x, current_y, target_x, target_y):
        # 거리 계산 (유클리드 거리)
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        
        # 방향 계산 (360도 기준, 정북에서 시계 방향)
        angle = math.degrees(math.atan2(target_y - current_y, target_x - current_x))
        if angle < 0:
            angle += 360
            
        return distance, angle

    def is_within_target_range(self, current_x, current_y, target):
        # 점 1~5, 점 9는 2m x 2m 범위, 점 6~8은 가로 4m, 세로 2m 범위
        if target['id'] in [1, 2, 3, 4, 5, 9, 10]:
            return abs(current_x - target['x']) <= 2.0 and abs(current_y - target['y']) <= 2.0
        elif target['id'] in [6]:
            return abs(current_x - target['x']) <= 3.0 and abs(current_y - target['y']) <= 10.0
        elif target['id'] in [7, 8]:
            return abs(current_x - target['x']) <= 2.0 and abs(current_y - target['y']) <= 3.0
        return False

    def callback(self, point_msg):
        current_x = point_msg.x
        current_y = point_msg.y
        
        # Mode7가 활성화된 상태에서는 더 이상 모드를 변경하지 않음
        if self.mode7_activated:
            rospy.loginfo(f"Mode 7 is already activated, staying in Mode 7")
            self.location_flag_pub.publish(7)
            return
        
        closest_target = None
        min_distance = float('inf')
        closest_direction = None

        for target in self.predefined_points:
            target_x = target['x']
            target_y = target['y']
            
            distance, direction = self.calculate_distance_and_direction(current_x, current_y, target_x, target_y)

            if distance < min_distance:
                min_distance = distance
                closest_target = target
                closest_direction = direction
        
        if closest_target:
            if self.is_within_target_range(current_x, current_y, closest_target):
                if closest_target['publish'] == 7:
                    # Mode7가 인식되면 해당 모드로 변경 후, 이후로는 모드가 변경되지 않음
                    rospy.loginfo(f"{closest_target['mode']} activated. Mode will not change.")
                    self.mode7_activated = True
                    self.current_mode = 7
                elif self.current_mode != closest_target['publish']:
                    self.current_mode = closest_target['publish']
                    rospy.loginfo(f"{closest_target['mode']}, Distance: {min_distance:.2f} meters, Direction: {closest_direction:.2f} degrees")
                    self.last_published = self.current_mode
            else:
                rospy.loginfo(f"Closest target out of range, Distance: {min_distance:.2f} meters, Direction: {closest_direction:.2f} degrees")
            
            self.location_flag_pub.publish(self.current_mode)
        else:
            rospy.loginfo(f"Continuing with last mode: Mode {self.last_published}, Distance: {min_distance:.2f} meters, Direction: {closest_direction:.2f} degrees")
            self.location_flag_pub.publish(self.last_published)

def listener():
    rospy.init_node('location_checker_node', anonymous=True)
    location_checker = LocationChecker()
    rospy.spin()

if __name__ == '__main__':
    listener()
