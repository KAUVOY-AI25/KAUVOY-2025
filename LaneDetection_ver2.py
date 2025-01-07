#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import torch
from torchvision import transforms
from PIL import Image as PILImage
import numpy as np
from module import NIA_SEGNet_module
import sys
from scipy.spatial import KDTree
import time

sys.path.append("/home/kauvoy/anaconda3/envs/vision/lib/python3.8/site-packages/torch")

class FrameProcessor:
    def __init__(self, model, output_topic):
        self.model = model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        self.bridge = CvBridge()
        self.output_topic = output_topic
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=10)
        self.roi_mask = None

        # Define ROI vertices (adjust as needed)
        self.roi_top_left_ratio = (0.15, 0.5)
        self.roi_top_right_ratio = (0.85, 0.5)
        self.roi_bottom_left_ratio = (0 ,1)
        self.roi_bottom_right_ratio = (1, 1)

        # 차선 데이터 초기화
        self.left_lane_points = None
        self.right_lane_points = None
        self.threshold = 300  # 프레임 간 이동 거리 임계값 (조정 가능)


    # def update_lane_points(self, output_predictions):
    #     """
    #     이전 프레임의 LEFT/RIGHT LANE 각각에 대해,
    #     현재 프레임의 점들을 가까운 쪽으로 분류.
    #     """
    #     # 현재 프레임의 모든 차선 점 수집
    #     lane_pixels = np.argwhere(output_predictions > 0)[:, [1, 0]]  # (y, x) -> (x, y)

    #     if len(lane_pixels) == 0:
    #         rospy.logwarn("No lane pixels found")
    #         return

    #     # KDTree 생성
    #     left_tree = KDTree(self.left_lane_points) if self.left_lane_points is not None else None
    #     right_tree = KDTree(self.right_lane_points) if self.right_lane_points is not None else None

    #     # LEFT와 RIGHT LANE으로 분류
    #     new_left_lane, new_right_lane = [], []
    #     for point in lane_pixels:
    #         left_dist, right_dist = float('inf'), float('inf')

    #         # LEFT LANE과의 거리 계산
    #         if left_tree is not None:
    #             left_dist, _ = left_tree.query(point)

    #         # RIGHT LANE과의 거리 계산
    #         if right_tree is not None:
    #             right_dist, _ = right_tree.query(point)

    #         # 가까운 쪽으로 점을 할당 (임계값 기준)
    #         if left_dist < right_dist and left_dist < self.threshold:
    #             new_left_lane.append(point)
    #         elif right_dist < self.threshold:
    #             new_right_lane.append(point)

    #     # 새로운 LANE 업데이트
    #     self.left_lane_points = np.array(new_left_lane) if len(new_left_lane) > 0 else self.left_lane_points
    #     self.right_lane_points = np.array(new_right_lane) if len(new_right_lane) > 0 else self.right_lane_points

    #     # 로그 출력
    #     rospy.loginfo(f"Updated left_lane_points: {len(self.left_lane_points) if self.left_lane_points is not None else 0}")
    #     rospy.loginfo(f"Updated right_lane_points: {len(self.right_lane_points) if self.right_lane_points is not None else 0}")

    def reinitialize_lane_points(self, output_predictions):
        """
        점의 개수 불균형 발생 시, 차선을 재설정합니다.
        """
        lane_info = self.extract_lane_info(output_predictions)
        if len(lane_info) > 0:
            self.left_lane_points = lane_info[0]['coordinates']
            self.right_lane_points = lane_info[1]['coordinates']
        else:
            rospy.logwarn("Failed to reinitialize lane points: No valid lanes found.")


    def update_lane_points(self, output_predictions):
        """
        이전 프레임의 LEFT/RIGHT LANE 각각에 대해,
        현재 프레임의 점들을 가까운 쪽으로 분류. 이전 프레임과 동일한 점은 추가 계산 없이 처리.
        """
        # 차선에 해당하는 모든 점을 수집
        lane_pixels = np.argwhere(output_predictions > 0)[:, [1, 0]]  # (y, x) -> (x, y)

        if len(lane_pixels) == 0:
            rospy.logwarn("No lane pixels found")
            return

        # 이전 프레임의 좌표를 `set`으로 변환 (검색 속도 향상)
        prev_left_set = set(map(tuple, self.left_lane_points)) if self.left_lane_points is not None else set()
        prev_right_set = set(map(tuple, self.right_lane_points)) if self.right_lane_points is not None else set()

        # 동일한 점과 새로운 점 분리
        same_left = [point for point in lane_pixels if tuple(point) in prev_left_set]
        same_right = [point for point in lane_pixels if tuple(point) in prev_right_set]
        new_points = [point for point in lane_pixels if tuple(point) not in prev_left_set and tuple(point) not in prev_right_set]

        # KDTree 생성 (새로운 점에만 사용)
        left_tree = KDTree(self.left_lane_points) if self.left_lane_points is not None and len(self.left_lane_points) > 0 else None
        right_tree = KDTree(self.right_lane_points) if self.right_lane_points is not None and len(self.right_lane_points) > 0 else None

        # 새로운 점 분류
        new_left_lane, new_right_lane = [], []
        for point in new_points:
            left_dist, right_dist = float('inf'), float('inf')

            # LEFT LANE과의 거리 계산
            if left_tree is not None:
                left_dist, _ = left_tree.query(point)

            # RIGHT LANE과의 거리 계산
            if right_tree is not None:
                right_dist, _ = right_tree.query(point)

            # 가까운 쪽으로 점을 분류 (임계값 사용)
            if left_dist < right_dist and left_dist < self.threshold:
                new_left_lane.append(point)
            elif right_dist < self.threshold:
                new_right_lane.append(point)

        # 최종 차선 점 업데이트
        self.left_lane_points = np.array(same_left + new_left_lane) if len(same_left + new_left_lane) > 0 else self.left_lane_points
        self.right_lane_points = np.array(same_right + new_right_lane) if len(same_right + new_right_lane) > 0 else self.right_lane_points

        # 점 개수 불균형 확인 및 재설정
        if (self.left_lane_points is not None and self.right_lane_points is not None):
            left_count = len(self.left_lane_points)
            right_count = len(self.right_lane_points)

            if left_count > 4 * right_count or right_count > 4 * left_count:
                rospy.logwarn(f"Lane point imbalance detected: Left={left_count}, Right={right_count}. Reinitializing KDTree...")
                self.reinitialize_lane_points(output_predictions)

        # 로그 출력
        rospy.loginfo(f"Updated left_lane_points: {len(self.left_lane_points)}")
        rospy.loginfo(f"Updated right_lane_points: {len(self.right_lane_points)}")

    def filter_odd_points(self, output_predictions):
        """
        딥러닝 결과에서 LABEL == 2인 점을 제외하고 홀수 Y 좌표만 남깁니다.
        """
        filtered_predictions = np.zeros_like(output_predictions)

        # 모든 Y 좌표에서 홀수 행만 복사, LABEL == 2 제외
        for y in range(output_predictions.shape[0]):
            if y % 2 == 1:  # Y 좌표가 홀수인 경우
                row = output_predictions[y, :]
                row[row == 2] = 0  # LABEL == 2 제거
                filtered_predictions[y, :] = row

        for x in range(output_predictions.shape[1]):
            if x % 2 == 1:
                col = output_predictions[: ]

        rospy.loginfo(f"Filtered predictions to odd rows, excluded LABEL == 2: shape {filtered_predictions.shape}")
        return filtered_predictions



    def process_frame(self, frame, frame_idx):
        start_time = time.time()
        img = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        transform = transforms.Compose([
            transforms.Resize((512, 1024)),
            transforms.ToTensor(),
        ])
        img_tensor = transform(img).unsqueeze(0)
        img_tensor = img_tensor.to(self.device)

        self.model.eval()
        with torch.no_grad():
            output = self.model(img_tensor)['out']

        output_predictions = output.argmax(1).squeeze().cpu().numpy()

        if self.roi_mask is None:
            self.roi_mask = self.get_roi_mask(output_predictions.shape)
        output_predictions = output_predictions * self.roi_mask  # ROI 적용

        # 홀수 좌표만 필터링
        output_predictions = self.filter_odd_points(output_predictions)

        # 최초 프레임: KDTree와 DFS로 초기화
        if self.left_lane_points is None or self.right_lane_points is None:
            lane_info = self.extract_lane_info(output_predictions)
            if len(lane_info) > 0:
                self.left_lane_points = lane_info[0]['coordinates']
                self.right_lane_points = lane_info[1]['coordinates']
        else:
            # 이후 프레임: 이전 프레임의 차선을 기준으로 업데이트
            self.update_lane_points(output_predictions)

            # lane_info 생성
            lane_info = [
                {'label': 'left_lane', 'coordinates': self.left_lane_points},
                {'label': 'right_lane', 'coordinates': self.right_lane_points},
            ]

        # 시각화 및 결과 출력
        overlay = self.visualize_lanes(frame, output_predictions, lane_info)  # lane_info 추가
        rospy.loginfo(f"Frame processing time: {time.time() - start_time:.2f} seconds")

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
        except CvBridgeError as e:
            print(e)

            
    def get_roi_mask(self, shape):
        mask = np.zeros(shape, dtype=np.uint8)
        height, width = shape
        roi_corners = self.calculate_roi_corners(width, height)
        cv2.fillPoly(mask, roi_corners, 1)
        return mask

    def calculate_roi_corners(self, width, height):
        return np.array([[
            (width * self.roi_bottom_left_ratio[0], height * self.roi_bottom_left_ratio[1]),    # Bottom-left
            (width * self.roi_bottom_right_ratio[0], height * self.roi_bottom_right_ratio[1]),  # Bottom-right
            (width * self.roi_top_right_ratio[0], height * self.roi_top_right_ratio[1]),        # Top-right
            (width * self.roi_top_left_ratio[0], height * self.roi_top_left_ratio[1])           # Top-left
        ]], dtype=np.int32)
    
    def extract_lane_info(self, output_predictions):
        unique_labels = np.unique(output_predictions)
        lane_pixels = []

        for label in unique_labels:
            if label != 0:  # 배경 제외
                pixels = np.argwhere(output_predictions == label)
                if pixels.size > 0:
                    lane_pixels.extend(pixels[:, [1, 0]])  # (y, x) -> (x, y)

        if len(lane_pixels) == 0:
            rospy.logwarn("No lane pixels found")
            return []

        lane_pixels = np.array(lane_pixels)
        center_x = output_predictions.shape[1] // 2

        left_candidates = lane_pixels[lane_pixels[:, 0] < center_x]
        right_candidates = lane_pixels[lane_pixels[:, 0] >= center_x]

        if len(left_candidates) == 0 or len(right_candidates) == 0:
            rospy.logwarn("Insufficient candidates for left or right lanes")
            return []

        left_start = left_candidates[np.argmax(left_candidates[:, 1])]
        right_start = right_candidates[np.argmax(right_candidates[:, 1])]

        def dfs(start, pixels, visited, threshold=20):
            tree = KDTree(pixels)
            stack = [start]
            connected_points = []

            while stack:
                point = stack.pop()
                if tuple(point) in visited:
                    continue
                visited.add(tuple(point))
                connected_points.append(point)

                indices = tree.query_ball_point(point, threshold)
                for idx in indices:
                    neighbor = pixels[idx]
                    if tuple(neighbor) not in visited:
                        stack.append(neighbor)

            return np.array(connected_points)

        visited = set()
        left_lane_points = dfs(left_start, lane_pixels, visited)
        right_lane_points = dfs(right_start, lane_pixels, visited)

        return [
            {'label': 'left_lane', 'coordinates': left_lane_points},
            {'label': 'right_lane', 'coordinates': right_lane_points}
        ]


    def publish_lane_info(self, lane_info):
        # Publish lane points and labels
        for info in lane_info:
            label = str(info['label'])
            coordinates = info['coordinates']
            rospy.loginfo(f"Label: {label}, Coordinates: {coordinates}")

    def visualize_lanes(self, frame, output_predictions, lane_info):
        resized_frame = cv2.resize(frame, (1024, 512))  # Resize the frame for visualization
        overlay = resized_frame.copy()
        height, width = resized_frame.shape[:2]
        roi_corners = self.calculate_roi_corners(width, height)
        cv2.polylines(overlay, roi_corners, isClosed=True, color=(0, 255, 0), thickness=2)
        
        for info in lane_info:
            label = info['label']
            coordinates = info['coordinates']

            # 차선별 색상 지정
            if label == 'left_lane':
                color = (255, 0, 0)  # Blue for left lane
            elif label == 'right_lane':
                color = (0, 0, 255)  # Red for right lane
            else:
                color = (0, 255, 0)  # Default color for other labels
                
            # 좌표 시각화
            for point in coordinates:
                x, y = int(point[0]), int(point[1])  # 좌표를 정수형으로 변환
                cv2.circle(overlay, (x, y), 1, color, -1)  # 각 좌표를 시각화

            # 중심점 계산 및 레이블 추가
            if len(coordinates) > 0:
                centroid = tuple(np.mean(coordinates, axis=0, dtype=int))  # 중심점 계산
                cv2.putText(overlay, label, centroid, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # LABEL == 2인 점들 시각화 (옵션)
        label_2_points = np.argwhere(output_predictions == 2)
        for point in label_2_points:
            x, y = point[1], point[0]
            cv2.circle(overlay, (x, y), 1, (0, 255, 255), -1)  # Yellow for LABEL == 2
                
        return overlay


    def get_label_color(self, label):
        if label == '1':
            return (200, 150, 200)  # BGR color for label 1
        elif label == '2':
            return (255, 0, 0)  # BGR color for label 2
        elif label == '3':
            return (0, 0, 255)  # BGR color for label 3
        else:
            return (0, 255, 0)  # Default color for other labels

def main():
    rospy.init_node('lane_detection_publisher')
    model = NIA_SEGNet_module.load_from_checkpoint(checkpoint_path="/home/kauvoy/catkin_ws/src/lane_detction/epoch=4-step=187500.ckpt")
    model.eval()

    output_topic = '/lane_detection_overlay'
    frame_processor = FrameProcessor(model, output_topic)

    # Subscribe to camera image topic
    #image_topic = '/camera2/usb_cam2/image_raw'
    image_topic = '/img_topic'
    #image_topic = '/panorama/image'

    rospy.Subscriber(image_topic, Image, frame_callback, frame_processor)

    rospy.spin()

def frame_callback(image_msg, frame_processor):
    try:
        frame = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")
        frame_processor.process_frame(frame, 0)  # Frame index can be set to any value, not used in this case
    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass