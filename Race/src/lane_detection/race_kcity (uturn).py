# ================================================================================
# Main Author: 윤수빈, 이창현
# Recently Modified Date: 2024-10-19
# Dependency: static_obs.py, uturn.py, mode_kcity.py, multicam
# Description: 전체 주행 mode(gps기반)에 따른 kcity 주행 알고리즘 실행 (차선 픽셀 기반, 유턴, 장애물 회피)
# 유턴 관련 알고리즘 추가 mode 4->5 직후 스티어링 2초간 15도
# ================================================================================

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from torchvision import transforms
from PIL import Image as PILImage
import numpy as np
from collections import deque
import math
from module import NIA_SEGNet_module
import sys
import time

# 특정 경로를 시스템 경로에 추가 (여기서는 PyTorch 라이브러리가 설치된 경로)
sys.path.append("/home/kauvoy/anaconda3/envs/vision/lib/python3.8/site-packages/torch")

class ModeTransitionHandler:
    def __init__(self):
        self.previous_mode = None
        self.mode_change_time = None
        self.uturn_steering = False  # uturn_steering 플래그 초기값

    def check_mode_transition(self, current_mode):
        if self.previous_mode == 4 and current_mode == 5:
            # MODE 4 -> MODE 5 전환을 감지한 순간
            self.mode_change_time = time.time()  # 전환 시간을 기록
            self.uturn_steering = True  # 2초 동안 uturn_steering 플래그를 True로 설정
            #print("유턴 스티어링 조정중")
        self.previous_mode = current_mode

    def handle_transition(self):
        if self.uturn_steering:
            # 현재 시간이 전환 시간에서 2초 지났는지 확인
            if time.time() - self.mode_change_time > 2:
                self.uturn_steering = False  # 2초가 지나면 uturn_steering 플래그를 False로 설정
                #print("유턴 스티어링 조정완료")
                     
class FrameProcessor:
    def __init__(self, model, output_topic, steering_angle_topic):
        self.model = model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')  # GPU가 가능하면 GPU를 사용하고, 아니면 CPU 사용
        self.model.to(self.device)
        self.bridge = CvBridge()  # OpenCV와 ROS 사이의 이미지 변환을 위한 브릿지
        self.output_topic = output_topic
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=10)  # 이미지 토픽을 ROS에 퍼블리시하기 위한 설정
        self.steering_angle_pub = rospy.Publisher(steering_angle_topic, Float64, queue_size=10)  # 조향각 퍼블리시를 위한 설정
        self.roi_mask = None  # ROI (관심영역) 마스크 초기화

        # 곡률과 조향각 계산을 위한 큐 (최근 5개의 값을 저장)
        self.steering_angles = deque(maxlen=5)
        self.left_curvatures, self.right_curvatures = deque(maxlen=5), deque(maxlen=5)
        
        self.mode = 0  # default 모드 = 0
        self.static_obstacle_flag = 0  # static obstacle flag 초기값
        self.uturn_flag = 0 # uturn flag 초기값

    def set_mode(self, mode):
        self.mode = mode
    def set_uturn_flag(self, flag):
        self.uturn_flag = flag
    def set_static_obstacle_flag(self, flag):
        self.static_obstacle_flag = flag
   
    def process_frame(self, frame, frame_idx):
        # Step 1: 원본 프레임에서 차선 검출 수행
        img = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        transform = transforms.Compose([
            transforms.Resize((512, 1024)),  # 이미지를 512x1024로 리사이즈
            transforms.ToTensor(),  # 이미지를 텐서로 변환
        ])
        img_tensor = transform(img).unsqueeze(0)  # 배치 차원을 추가하여 텐서를 모델에 맞게 변환
        img_tensor = img_tensor.to(self.device)  # 텐서를 디바이스(GPU 또는 CPU)로 이동

        self.model.eval()  # 모델을 평가 모드로 전환
        with torch.no_grad():  # 그라디언트 계산을 비활성화하여 추론 실행
            output = self.model(img_tensor)['out']  # 모델의 출력값을 얻음

        output_predictions = output.argmax(1).squeeze().cpu().numpy().astype(np.uint8)  # 예측 결과를 최대값 인덱스로 변환하고 numpy 배열로 변환

        # ROI 마스크 적용
        if self.roi_mask is None:
            self.roi_mask = self.get_roi_mask(output_predictions.shape)  # ROI 마스크를 처음에만 생성
        output_predictions = output_predictions * self.roi_mask  # ROI 마스크를 예측 결과에 적용

        # Step 2: 출력 크기를 원본 프레임 크기에 맞게 리사이즈
        resized_output = cv2.resize(output_predictions, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Step 3: 원본 프레임에 Bird's Eye View (BEV) 변환 적용
        bev_original_frame = self.apply_birds_eye_view(frame)

        # Step 4: 리사이즈된 출력에 BEV 변환을 적용하여 BEV 변환된 차선 검출 결과 얻기
        bev_output = self.apply_birds_eye_view(resized_output)

        # Step 5: BEV 변환된 차선 검출 결과를 기반으로 마스크 생성
        mask = np.zeros_like(bev_original_frame[:, :, 0])  # 마스크 초기화
        mask[bev_output > 0] = 255  # 차선이 검출된 부분을 흰색으로 마스크 설정

        # Step 6: 차선 검출 결과를 BEV 원본 프레임에 오버레이
        bev_overlay = bev_original_frame.copy()
        bev_overlay[mask == 255] = [255, 0, 255]  # 검출된 차선을 녹색으로 표시

        # Step 7: BEV 오버레이에 ROI 그리기
        bev_overlay = self.draw_roi_on_bev(bev_overlay)

        # Step 8: 차선을 시각화하고, 특정 포인트까지의 거리 및 곡률 계산
        bev_overlay, distance_to_point, left_curvature, right_curvature = self.visualize_lanes(bev_overlay, bev_output)

        # Step 9: 검출된 차선을 기반으로 조향각 계산
        _, _, _, steering_angle = self.calculate_curvature_and_steering(bev_output, bev_overlay)

        # Step 10: 현재 모드를 비디오에 오버레이
        mode_text = f"Mode: {self.mode} ("
        if self.mode == 0:
            mode_text += "General)"
        elif self.mode == 1:
            mode_text += "Right Lane Only)"
        elif self.mode == 2:
            mode_text += "Ignore Lanes)"
        elif self.mode == 3:
            mode_text += "Narrow ROI)"
        elif self.mode == 4:
            mode_text += "U-turn)"
        elif self.mode == 5:
            mode_text += "High-Pass)"  
        elif self.mode == 6:
            mode_text += "Narrow ROI)"
        elif self.mode == 7:
            mode_text += "Tunnel & Obstacle)"  

        cv2.putText(bev_overlay, mode_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Step 11: 조향각과 최종 BEV 오버레이 프레임을 퍼블리시
        try:
            if steering_angle is not None:
                cv2.putText(bev_overlay, f"Steering Angle: {steering_angle:.2f} degrees", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)  # 조향각을 프레임에 표시
                self.steering_angle_pub.publish(Float64(steering_angle))  # ROS 토픽으로 조향각 퍼블리시
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(bev_overlay, "bgr8"))  # ROS 토픽으로 BEV 오버레이 이미지 퍼블리시
        except CvBridgeError as e:
            print(e)

    # Control
    def calculate_curvature_and_steering(self, bev_output, bev_overlay):
        # 차선 정보 추출
        lane_info = self.extract_lane_info(bev_output)
        left_curvature, right_curvature = None, None
        steering_angle = None

        # ROI Mask 계산
        height, width = bev_output.shape
        roi_corners = self.calculate_roi_corners(width, height)
        roi_mask = np.zeros_like(bev_output, dtype=np.uint8)
        cv2.fillPoly(roi_mask, [roi_corners], 1)

        # ROI 내에서 Left, Right 차선
        if lane_info['left']:
            left_coordinates = lane_info['left'][0]['coordinates']
            left_coordinates = left_coordinates[roi_mask[left_coordinates[:, 1], left_coordinates[:, 0]] > 0]
            if len(left_coordinates) > 0:
                left_coordinates = self.select_sample_points(left_coordinates)  # 좌측 차선 좌표 샘플링
                left_polynomial = self.fit_polynomial(left_coordinates)  # 좌측 차선 다항식 피팅
                left_curvature = self.calculate_curvature(left_polynomial, bev_output.shape[0])  # 좌측 차선 곡률 계산

        if lane_info['right']:
            right_coordinates = lane_info['right'][0]['coordinates']
            right_coordinates = right_coordinates[roi_mask[right_coordinates[:, 1], right_coordinates[:, 0]] > 0]
            if len(right_coordinates) > 0:
                right_coordinates = self.select_sample_points(right_coordinates)  # 우측 차선 좌표 샘플링
                right_polynomial = self.fit_polynomial(right_coordinates)  # 우측 차선 다항식 피팅
                right_curvature = self.calculate_curvature(right_polynomial, bev_output.shape[0])  # 우측 차선 곡률 계산

##############################################################################################################################################################################
        # [Mode 0]: General (일반주행)
        if self.mode == 0:
            if left_curvature is not None and right_curvature is not None:
                lane_center_x = (np.mean(left_coordinates[:, 0]) + np.mean(right_coordinates[:, 0])) / 2  # 차선 중앙 계산
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif left_curvature is not None:
                # 좌측 차선만 감지된 경우, 좌측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(left_coordinates[:, 0]) + 130  
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif right_curvature is not None:
                # 우측 차선만 감지된 경우, 우측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(right_coordinates[:, 0]) - 130
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            else:
                # 두 개의 차선이 모두 감지되지 않은 경우, 조향각을 0으로 설정
                steering_angle = 0.0

        # [Mode 1]: Default steering (우회전)
        elif self.mode == 1:
            steering_angle = 8

        # [Mode 2]: Ignore the Lane (직진)
        elif self.mode == 2:
            steering_angle = 0.0
            
        # [Mode 3]: U-TURN하러 가기 전에 차선 ROI 좁히기 
        if self.mode == 3:
            if left_curvature is not None and right_curvature is not None:
                lane_center_x = (np.mean(left_coordinates[:, 0]) + np.mean(right_coordinates[:, 0])) / 2  # 차선 중앙 계산
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif left_curvature is not None:
                # 좌측 차선만 감지된 경우, 좌측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(left_coordinates[:, 0]) + 130  
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif right_curvature is not None:
                # 우측 차선만 감지된 경우, 우측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(right_coordinates[:, 0]) - 130
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            else:
                # 두 개의 차선이 모두 감지되지 않은 경우, 조향각을 0으로 설정
                steering_angle = 0.0         

        # [Mode 4]: U-Turn
        elif self.mode == 4:
            steering_angle = 0.0
            
            # Mode 3 에서 유턴 플래그가 뜨면 무조건 유턴 !!!
            if self.uturn_flag == 1:
                steering_angle = 17; 
                print("!!!!!!!!!!MODE = 3  ----------->  U-TURN!!!!!!!!!!!!!!!!!!!!!!!!")
            
            elif left_curvature is not None and right_curvature is not None:
                # 두 차선이 모두 감지된 경우, 양 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = (np.mean(left_coordinates[:, 0]) + np.mean(right_coordinates[:, 0])) / 2  # 차선 중앙 계산
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif left_curvature is not None:
                # 좌측 차선만 감지된 경우, 좌측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(left_coordinates[:, 0]) + 130  
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif right_curvature is not None:
                # 우측 차선만 감지된 경우, 우측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(right_coordinates[:, 0]) - 130 
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            else:
                # 두 개의 차선이 모두 감지되지 않은 경우, 조향각을 0으로 설정
                steering_angle = 0.0
 
        # [Mode 5]: HIGH-PASS
        elif self.mode == 5:
            steering_angle = 13.0
            if left_curvature is not None and right_curvature is not None:
                lane_center_x = (np.mean(left_coordinates[:, 0]) + np.mean(right_coordinates[:, 0])) / 2  # 차선 중앙 계산
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif left_curvature is not None:
                # 좌측 차선만 감지된 경우, 좌측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(left_coordinates[:, 0]) + 130 
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif right_curvature is not None:
                # 우측 차선만 감지된 경우, 우측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(right_coordinates[:, 0]) - 130 
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif ModeTransitionHandler.uturn_steering == True:
                steering_angle = 13
                print("유턴 후 steering 조정 중")
            else:
                # 두 개의 차선이 모두 감지되지 않은 경우, 조향각을 0으로 설정
                steering_angle = 0.0
     
        # [Mode 6]: Narrow Roi (넓은 차선 방지용)-> General Race
        elif self.mode == 6:
            steering_angle = 0.0
            if left_curvature is not None and right_curvature is not None:
                lane_center_x = (np.mean(left_coordinates[:, 0]) + np.mean(right_coordinates[:, 0])) / 2  # 차선 중앙 계산
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif left_curvature is not None:
                # 좌측 차선만 감지된 경우, 좌측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(left_coordinates[:, 0]) + 130 
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            elif right_curvature is not None:
                # 우측 차선만 감지된 경우, 우측 차선의 곡률을 기반으로 조향각 계산
                lane_center_x = np.mean(right_coordinates[:, 0]) - 130  #
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])  # 조향각 계산
            else:
                # 두 개의 차선이 모두 감지되지 않은 경우, 조향각을 0으로 설정
                steering_angle = 0.0    
     
        # [Mode 7]: Tunnel & Obstacle
        elif self.mode == 7:
            steering_angle = 0.0
            if left_curvature is not None and right_curvature is not None: 
                lane_center_x = (np.mean(left_coordinates[:, 0]) + np.mean(right_coordinates[:, 0])) / 2  # 차선 중앙 계산
                vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
                # static_obstacle_flag에 따른 제어
                # -------------------------------------------------------------------------
                if self.static_obstacle_flag == 1:  # 오른쪽에 장애물이 있을 때
                    # 왼쪽 차선 중심으로 조향각 계산
                    lane_center_x = np.mean(left_coordinates[:, 0]) + 65  
                    rospy.loginfo("오른쪽에 장애물이 있으므로, 왼쪽 차선에 붙어 주행")
                    steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
                elif self.static_obstacle_flag == -1:  # 왼쪽에 장애물이 있을 때
                    # 오른쪽 차선 중심으로 조향각 계산
                    lane_center_x = np.mean(right_coordinates[:, 0]) - 65 
                    rospy.loginfo("왼쪽에 장애물이 있으므로, 오른쪽 차선에 붙어 주행")
                    steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
                elif self.static_obstacle_flag == 11: #right
                    steering_angle = -4
                elif self.static_obstacle_flag == 110: #left
                    steering_angle = 4
                                        
                # -------------------------------------------------------------------------
                elif self.static_obstacle_flag == 0 or 1000:
                    lane_center_x = (np.mean(left_coordinates[:, 0]) + np.mean(right_coordinates[:, 0])) / 2  # 차선 중앙 계산
                    vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                    steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산   

            elif left_curvature is not None:
                # 좌측 차선만 감지된 경우
                lane_center_x = np.mean(left_coordinates[:, 0]) + 130  # 좌측 차선 중심으로 이동 (값 조정 가능)
                vehicle_center_x = bev_overlay.shape[1] // 2
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
                # static_obstacle_flag에 따른 제어
                # --------------------------------------------------------------------------
                if self.static_obstacle_flag == 1 or -1:  # 오른쪽에 장애물이 있을 때
                    # 왼쪽 차선 중심으로 조향각 계산
                    lane_center_x = np.mean(left_coordinates[:, 0]) + 65  
                    rospy.loginfo("오른쪽에 장애물이 있으므로, 왼쪽 차선에 붙어 주행")
                    steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
                elif self.static_obstacle_flag == 11: #right
                    steering_angle = -4
                elif self.static_obstacle_flag == 110: #left
                    steering_angle = 4
                # --------------------------------------------------------------------------
                elif self.static_obstacle_flag == 0 or 1000 or 1111 or 1004 :
                    lane_center_x = np.mean(left_coordinates[:, 0]) + 130   # 차선 중앙 계산
                    vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산           
                    steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산     
        
            elif right_curvature is not None:
                # 우측 차선만 감지된 경우
                lane_center_x = np.mean(right_coordinates[:, 0]) - 130  # 우측 차선 중심으로 이동 (값 조정 가능)
                vehicle_center_x = bev_overlay.shape[1] // 2
                steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
                
                # static_obstacle_flag에 따른 제어
                # ------------------------------------------------------------------
                
                if self.static_obstacle_flag == 99: #왼쪽 장애물을 회피했는데, 왼쪽 차선을 잡지 못할 때 스티어링 조정
                    steering_angle = -8
                elif self.static_obstacle_flag == -1 or 1:  # 왼쪽에 장애물이 있을 때
                    # 왼쪽 차선 중심으로 조향각 계산
                    lane_center_x = np.mean(right_coordinates[:, 0]) - 65  
                    rospy.loginfo("왼쪽에 장애물이 있으므로, 오른쪽 차선에 붙어 주행")
                    steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
                elif self.static_obstacle_flag == 11: #right
                    steering_angle = -4
                elif self.static_obstacle_flag == 110: #left
                    steering_angle = 4
                # ------------------------------------------------------------------
                elif self.static_obstacle_flag == 0 or 1000 or 1111 or 1004 :
                    lane_center_x = np.mean(right_coordinates[:, 0]) - 130   # 차선 중앙 계산
                    vehicle_center_x = bev_overlay.shape[1] // 2  # 차량 중앙 계산
                    steering_angle = self.calculate_steering_angle(vehicle_center_x, lane_center_x, bev_output.shape[0])   # 조향각 계산
               
            else:
                # 차선이 감지되지 않은 경우
                steering_angle = 0.0                        
                        
        return None, left_curvature, right_curvature, steering_angle
    
##############################################################################################################################################################################

    def select_sample_points(self, coordinates, num_points=30):
        if len(coordinates) <= num_points:
            return coordinates
        interval = len(coordinates) // num_points
        return coordinates[::interval]  # 지정된 개수의 포인트를 샘플링하여 반환
   
    def fit_polynomial(self, lane_coordinates):
        x = lane_coordinates[:, 0]
        y = lane_coordinates[:, 1]
        return np.polyfit(y, x, 2)  # 차선 좌표에 2차 다항식 피팅

    def calculate_curvature(self, polynomial, y_eval):
        A = polynomial[0]
        B = polynomial[1]
        curvature = ((1 + (2 * A * y_eval + B) ** 2) ** 1.5) / np.abs(2 * A)  # 곡률 계산
        return curvature

    def calculate_steering_angle(self, vehicle_center_x, lane_center_x, height):
        dx = lane_center_x - vehicle_center_x  # 차량 중앙과 차선 중앙 사이의 차이 계산
        angle_radians = math.atan2(dx, height)  # 아크탄젠트를 사용하여 각도 계산
        return math.degrees(angle_radians) * 0.6  # 계산된 각도를 도 단위로 변환하고 스케일링

    def apply_birds_eye_view(self, frame):
        height, width = frame.shape[:2]

        # 원근 변환을 위한 원본 이미지의 좌표 설정
        src_points = np.float32([
            [width * 0.2, height * 0.2],   # 좌측 상단
            [width * 0.8, height * 0.2],   # 우측 상단
            [width * 1.0, height],         # 우측 하단
            [width * 0.0, height]          # 좌측 하단
        ])

        # 변환 결과 이미지의 좌표 설정
        dst_points = np.float32([
            [width * 0.1, 0],              # 좌측 상단
            [width * 0.9, 0],              # 우측 상단
            [width * 0.7, height],         # 우측 하단
            [width * 0.3, height]          # 좌측 하단
        ])

        # 변환 행렬 계산
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bev_frame = cv2.warpPerspective(frame, matrix, (width, height))  # 원근 변환 적용

        return bev_frame

    def get_roi_mask(self, shape):
        mask = np.zeros(shape, dtype=np.uint8)  # ROI 마스크 초기화
        height, width = shape
        roi_corners = self.calculate_roi_corners(width, height)  # ROI 꼭짓점 계산
        cv2.fillPoly(mask, [roi_corners], 1)  # ROI 영역을 마스크에 채움
        return mask

    def draw_roi_on_bev(self, bev_frame):
        height, width = bev_frame.shape[:2]
        roi_corners = self.calculate_roi_corners(width, height)  # ROI 꼭짓점 계산
        cv2.polylines(bev_frame, [roi_corners], isClosed=True, color=(0, 255, 0), thickness=2)  # ROI를 녹색으로 그림
        return bev_frame

    def calculate_roi_corners(self, width, height):
        if self.mode == 3 or self.mode == 4:
            # 모드 3에서는 오른쪽 차선 ROI를 감소
            return np.array([[
                (width * 0.15, height * 1),   # 좌측 하단
                (width * 0.85, height * 1),   # 우측 하단 
                (width * 0.8, height * 0.45),   # 우측 상단 
                (width * 0.2, height * 0.45)    # 좌측 상단
            ]], dtype=np.int32)
        elif self.mode == 6:
            # 모드 6에서는 왼쪽 차선 ROI를 감소
            return np.array([[
                (width * 0.25, height * 1),   # 좌측 하단
                (width * 0.9, height * 1),   # 우측 하단
                (width * 0.9, height * 0.45),   # 우측 상단
                (width * 0.25, height * 0.45)    # 좌측 상단
            ]], dtype=np.int32)
        elif self.mode == 7:
            # 모드 7에서는 왼쪽 차선 ROI를 감소
            return np.array([[
                (width * 0.1, height * 0.95),    # 좌측 하단
                (width * 0.9, height * 0.95),  # 우측 하단
                (width * 0.9, height * 0.15),        # 우측 상단
                (width * 0.1, height * 0.15)     # 좌측 상단
            ]], dtype=np.int32)
        else:
            # Default
            return np.array([[
                (width * 0.15, height * 0.95),    # 좌측 하단
                (width * 0.9, height * 0.95),  # 우측 하단
                (width * 0.9, height * 0.15),        # 우측 상단
                (width * 0.15, height * 0.15)     # 좌측 상단
            ]], dtype=np.int32)


    def extract_lane_info(self, bev_output):
        unique_labels = np.unique(bev_output)  # 출력 이미지의 고유 라벨 추출
        lane_info = {'left': [], 'right': [], 'others': []}  # 차선 정보 초기화
        height, width = bev_output.shape
        mid_x = width // 2  # 이미지의 중앙 X 좌표 계산
        for label in unique_labels:
            if label != 0:  # 배경 라벨 제외
                lane_pixels = np.argwhere(bev_output == label)  # 특정 라벨의 픽셀 위치 찾기
                lane_coordinates = lane_pixels[:, [1, 0]]  # (y, x) 좌표를 (x, y)로 변환
                lane_coordinates = np.unique(lane_coordinates, axis=0)  # 중복 제거

                if label == 1:
                    left_coordinates = lane_coordinates[lane_coordinates[:, 0] < mid_x]  # 좌측 차선 좌표
                    right_coordinates = lane_coordinates[lane_coordinates[:, 0] >= mid_x]  # 우측 차선 좌표

                    if len(left_coordinates) > 0:
                        lane_info['left'].append({'label': label, 'coordinates': left_coordinates})  # 좌측 차선 정보 추가
                    if len(right_coordinates) > 0:
                        lane_info['right'].append({'label': label, 'coordinates': right_coordinates})  # 우측 차선 정보 추가
                else:
                    lane_info['others'].append({'label': label, 'coordinates': lane_coordinates})  # 기타 라벨 정보 추가

        return lane_info

    def visualize_lanes(self, bev_overlay, bev_output):
        lane_info = self.extract_lane_info(bev_output)  # 차선 정보 추출
        left_coordinates, right_coordinates = None, None
        distance_to_point = None
        left_curvature, right_curvature = None, None

        height, width = bev_overlay.shape[:2]
        roi_corners = self.calculate_roi_corners(width, height)
        cv2.polylines(bev_overlay, [roi_corners], isClosed=True, color=(0, 255, 0), thickness=2)  # ROI 그리기

        car_position = (width // 2, height)  # 차량 위치 계산

        roi_mask = np.zeros_like(bev_output, dtype=np.uint8)
        cv2.fillPoly(roi_mask, [roi_corners], 1)  # ROI 마스크 생성

        for side in lane_info:
            for info in lane_info[side]:
                label = str(info['label'])
                coordinates = info['coordinates']
                color = self.get_label_color(label)  # 라벨에 따른 255색상 선택

                # ROI를 기반으로 좌표 필터링
                coordinates = coordinates[roi_mask[coordinates[:, 1], coordinates[:, 0]] > 0]

                if side == 'left' and len(coordinates) > 0:
                    left_coordinates = coordinates
                    start_point = coordinates[0]
                    end_point = coordinates[-1]
                    cv2.circle(bev_overlay, tuple(start_point), 10, (0, 0, 0), -1)  # 시작점 표시 (검정색)
                    cv2.circle(bev_overlay, tuple(end_point), 10, (0, 0, 0), -1)  # 끝점 표시 (검정색)
                    for point in coordinates:
                        cv2.circle(bev_overlay, tuple(point), 1, (0, 255, 0), -1)  # 좌측 차선을 녹색으로 표시

                elif side == 'right' and len(coordinates) > 0:
                    right_coordinates = coordinates
                    start_point = coordinates[0]
                    end_point = coordinates[-1]
                    cv2.circle(bev_overlay, tuple(start_point), 10, (0, 255, 255), -1)  # 시작점 표시 (노란색)
                    cv2.circle(bev_overlay, tuple(end_point), 10, (0, 255, 255), -1)  # 끝점 표시 (노란색)
                    for point in coordinates:
                        cv2.circle(bev_overlay, tuple(point), 1, (0, 0, 255), -1)  # 우측 차선을 빨간색으로 표시

                else:
                    for point in coordinates:
                        cv2.circle(bev_overlay, tuple(point), 1, color, -1)  # 기타 라벨의 좌표를 작은 점으로 표시

                    if label == '2' and len(coordinates) > 0:
                        specific_point = tuple(coordinates[0])
                        cv2.circle(bev_overlay, specific_point, 10, (255, 255, 255), -1)  # 특정 좌표 표시
                        rospy.loginfo(f"Specific Point for label {label}: {specific_point}")
                        distance_to_point = np.linalg.norm(np.array(car_position) - np.array(specific_point))  # 차량과 특정 포인트 간 거리 계산

        if left_coordinates is not None and len(left_coordinates) > 0:
            left_coordinates = self.select_sample_points(left_coordinates)  # 좌측 차선 좌표 샘플링
            left_polynomial = self.fit_polynomial(left_coordinates)  # 좌측 차선 다항식 피팅
            left_curvature = self.calculate_curvature(left_polynomial, height)  # 좌측 차선 곡률 계산

        if right_coordinates is not None and len(right_coordinates) > 0:
            right_coordinates = self.select_sample_points(right_coordinates)  # 우측 차선 좌표 샘플링
            right_polynomial = self.fit_polynomial(right_coordinates)  # 우측 차선 다항식 피팅
            right_curvature = self.calculate_curvature(right_polynomial, height)  # 우측 차선 곡률 계산

        return bev_overlay, distance_to_point, left_curvature, right_curvature

    def get_label_color(self, label):
        if label == '1':
            return (0, 255, 0)  # 녹색
        elif label == '2':
            return (255, 0, 0)  # 빨간색
        elif label == '3':
            return (0, 0, 255)  # 파란색
        else:
            return (255, 0, 255)  # 기본 핑크색

# 기존 main 함수 안에서 코드 추가
def main():
    rospy.init_node('lane_detection_publisher')  # ROS 노드 초기화
    model = NIA_SEGNet_module.load_from_checkpoint(checkpoint_path="/home/kauvoy/Race/src/lane_detection/epoch=4-step=187500.ckpt")  # 모델 체크포인트 로드
    model.eval()  # 모델을 평가 모드로 전환

    output_topic = '/lane_detection'
    steering_angle_topic = '/steering_angle'
    frame_processor = FrameProcessor(model, output_topic, steering_angle_topic)  # 프레임 처리 객체 초기화

    mode_transition_handler = ModeTransitionHandler()  # Mode 전환 처리기 추가

    # 모드를 수신하기 위한 구독
    mode_topic = '/mode_topic'
    rospy.Subscriber(mode_topic, Int32, lambda msg: frame_processor.set_mode(msg.data))
    rospy.Subscriber(mode_topic, Int32, lambda msg: mode_transition_handler.check_mode_transition(msg.data))  # MODE 전환 감지

    # static obstacle flag 토픽을 구독
    static_obstacle_flag_topic = '/static_obstacle_flag'
    rospy.Subscriber(static_obstacle_flag_topic, Int32, lambda msg: frame_processor.set_static_obstacle_flag(msg.data))

    # uturn flag 토픽을 구독
    uturn_flag_topic = '/uturn_flag'
    rospy.Subscriber(uturn_flag_topic, Int32, lambda msg: frame_processor.set_uturn_flag(msg.data))

    rate = rospy.Rate(10)  # 10Hz 루프
    while not rospy.is_shutdown():
        mode_transition_handler.handle_transition()  # 전환 상태 처리
        rate.sleep()

def frame_callback(image_msg, frame_processor):
    try:
        frame = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")  # ROS 이미지 메시지를 OpenCV 이미지로 변환
        frame_processor.process_frame(frame, 0)  # 프레임 처리 (프레임 인덱스는 필요에 따라 설정)
    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#rostopic pub /mode_topic std_msgs/Int32 "data: 6"
