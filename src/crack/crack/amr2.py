import time
import os
import sys
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from crack_msgs.msg import Obstacle, Danger
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Robot2Dectctor(Node):
    def __init__(self, model):
        super().__init__('robot2_detector')
        self.group = ReentrantCallbackGroup()
        self.model = model
        self.latest_image = None
        self.depth_image = None
        self.depth_header = None
        self.should_shutdown = False
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.K = None
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10, callback_group=self.group) # 1
        self.obstacle_publisher = self.create_publisher(Obstacle, '/obstacle', 10, callback_group=self.group)  #2
        self.crack_publisher = self.create_publisher(Danger, '/danger', 10, callback_group=self.group)  #3
        self.classNames = model.names if hasattr(model, 'names') else ['Object']
        self.can_publish_car = False     # car publish 가능 여부
        self.car_timer_started = False   # 타이머 중복 방지

        self.marker_id = 0

        # way point를 통해 계산한 탈출구 좌표
        self.exit4 = PointStamped()
        self.exit4.header.frame_id = 'map'
        self.exit4.point.x = 4.0
        self.exit4.point.y = 2.0

        self.exit5 = PointStamped()
        self.exit5.header.frame_id = 'map'
        self.exit5.point.x = 1.0
        self.exit5.point.y = 3.0

        self.detected_human = self.create_subscription(Obstacle, '/detected_human', self.human_callback, 10, callback_group=self.group)
        self.detected_big_crack = self.create_subscription(Danger, '/danger', self.danger_callback,10, callback_group=self.group)        
        # transform 시작 여부를 체크할 플래그
        self.transform_started = False

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.subscription = self.create_subscription(
            CompressedImage,
            self.rgb_topic,
            self.rgb_compressed_callback,
            10,callback_group=self.group) # 4

        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10,callback_group=self.group) # 5

        self.info_subscription = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.camera_info_callback,
            10,callback_group=self.group) # 6

        # ★ 3초 후에 변환 시작 타이머 설정
        self.get_logger().info("TF Tree 안정화 시작. 3초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(3.0, self.start_transform)


    def human_callback(self, msg):

        if msg.class_name == 'car':
            try:
                # 사람 위치 정보
                human_x = float(msg.x)
                human_y = float(msg.y)

                # 출구 4와의 거리 계산
                dx4 = human_x - self.exit4.point.x
                dy4 = human_y - self.exit4.point.y
                dist_exit4 = (dx4 ** 2 + dy4 ** 2) ** 0.5

                # 출구 5와의 거리 계산
                dx5 = human_x - self.exit5.point.x
                dy5 = human_y - self.exit5.point.y
                dist_exit5 = (dx5 ** 2 + dy5 ** 2) ** 0.5

                # 더 가까운 출구 판단
                if dist_exit4 < dist_exit5:
                    nearest_exit = "exit4 (4.0, 2.0)"
                    nearest_dist = dist_exit4
                else:
                    nearest_exit = "exit5 (1.0, 3.0)"
                    nearest_dist = dist_exit5

                self.get_logger().info(
                    f"사람 위치: ({human_x:.2f}, {human_y:.2f}) → 가장 가까운 출구: {nearest_exit}, 거리: {nearest_dist:.2f}m"
                )
                
            except ValueError as e:
                self.get_logger().warn(f"Obstacle 위치 정보 파싱 오류: {e}")

    def danger_callback(self, msg):
        if msg.danger_sense == 1.0 :
            self.transform_started = True
        else:
            self.get_logger().info('아직 큰 균열이 감지되지 않았습니다')


    def start_transform(self):
        # 첫 변환 시도
        if self.transform_started == True:
            self.thread = threading.Thread(target=self.detection_loop, daemon=True) # python thread
            self.thread.start()
            self.start_timer.cancel()
        else:
            self.get_logger().info('아직 transform이 시작되지 않았습니다.')


    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth CV bridge conversion failed: {e}")

    def rgb_compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"RGB compressed conversion failed: {e}")

    def stop_car_publishing(self):
        self.get_logger().info("10초 경과: car publish 중단")
        self.can_publish_car = False
        self.car_timer_started = False   # 다시 타이머 설정 가능하게 초기화

    def calculate_3d_coordinates(self, x, y, z):
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        return X, Y, z

    def detection_loop(self):

        while not self.should_shutdown:
            if self.latest_image is None or self.depth_image is None:
                time.sleep(0.01)
                continue
            
            img = self.latest_image.copy()
            results = self.model.predict(img, stream=True)

            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    ux, uy = (x1 + x2) // 2, (y1 + y2) // 2
                    cls = int(box.cls[0]) if box.cls is not None else 0
                    conf = float(box.conf[0]) if box.conf is not None else 0.0
                    class_name = self.classNames[cls]

                    # 깊이 이미지 범위 체크
                    if 0 <= uy < self.depth_image.shape[0] and 0 <= ux < self.depth_image.shape[1]:
                        z = float(self.depth_image[uy, ux]) / 1000.0  # mm → m

                        if z == 0.0 or np.isnan(z):
                            continue  # 잘못된 깊이 데이터는 무시

                        # 3D 좌표 계산
                        X, Y, Z = self.calculate_3d_coordinates(ux, uy, z)

                        # 좌표 변환
                        pt = PointStamped()
                        pt.header.frame_id = self.depth_header.frame_id
                        pt.header.stamp = self.depth_header.stamp
                        pt.point.x = X
                        pt.point.y = Y
                        pt.point.z = Z

                        try:
                            pt_map = self.tf_buffer.transform(
                                pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5)
                            )

                            # 1. 모든 객체 로그 출력 (좌표 포함)
                            self.get_logger().info(
                                f"[{class_name}] Pos: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f}, dis: {z:.2f})"
                            )


                            if class_name.lower() == 'crack' :
                                X1, Y1, Z1 = self.calculate_3d_coordinates(x1, y1, z)
                                X2, Y2, Z2 = self.calculate_3d_coordinates(x2, y2, z)
                                size = np.sqrt((X1 - X2) ** 2 + (Y1 - Y2) ** 2 )
                                size2 = (X1 - X2)
                                self.get_logger().info(f"Bounding box 3D width (기존 코드 (x y z)): {size:.2f}")
                                self.get_logger().info(f"Bounding box 3D length (X 좌표 차이만으로 계산): {size2:.2f}")

                                danger=Danger()
                                danger.x = round(pt_map.point.x,)


                                ## size는 직접 쟤보면서 확인해봐야 할 것 같음!
                                if size >= 0.15:
                                    self.get_logger().info(
                                    f" Danger! Width {(size-0.15):.2f} over !" )
                                    danger = Danger()
                                    danger.danger_sense = 1.0 # 위험
                                    self.crack_publisher.publish(danger)

                                else:
                                    self.get_logger().info(
                                    f" Safe !" )
                                    danger = Danger()
                                    danger.danger_sense = 0.0 # 안전
                                    self.crack_publisher.publish(danger)
                            # Marker 및 obstacle 메시지 발행
                            obstacle = Obstacle()
                            obstacle.class_name = class_name
                            obstacle.obstacle_distance = z
                            obstacle.x_pose = f"{pt_map.point.x:.2f}"
                            obstacle.y_pose = f"{pt_map.point.y:.2f}"
                            obstacle.z_pose = f"{pt_map.point.z:.2f}"
                            self.publish_marker(obstacle, pt_map.point.x, pt_map.point.y, self.marker_id)
                            self.marker_id+=1

                            # 균열일 경우 계속 정보 전달
                            # if class_name.lower() == 'crack':
                            #     self.obstacle_publisher.publish(obstacle)

                            # 사람일 경우 10초만 정보 전달
                            if class_name.lower() == 'car':
                                
                                # 타이머가 아직 시작되지 않았다면 시작
                                if not self.car_timer_started:
                                    self.can_publish_car = True
                                    self.car_timer_started = True
                                    self.create_timer(10.0, self.stop_car_publishing)
                                
                                # 사람 위치 publisher 추가
                                if self.can_publish_car:
                                    self.obstacle_publisher.publish(obstacle)

                        except Exception as e:
                            self.get_logger().warn(f"TF transform to map failed: {e}")

                    # 시각화용 표시 (좌표 상관없이 항상 표시)
                    label = f"{class_name} {conf:.2f}"
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 이미지 띄우기
            cv2.imshow("YOLO Detection with 3D", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutdown requested via 'q'")
                self.should_shutdown = True
                break

    def publish_marker(self,obstacle,x,y, marker_id):
            marker = Marker()
            # Header 설정
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            
            # 마커의 네임스페이스와 ID 설정 (여러 마커를 구분할 때 사용)
            marker.ns = "basic_shapes"
            marker.id = marker_id
            
            # 마커 타입 설정 (SPHERE)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 마커의 위치와 크기 설정
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # 마커 색상 설정
            #근데 이 부분 obstacle.classname으로 비교해야 하지 않나?

            if obstacle.class_name == 'crack':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # 투명도 (1.0은 불투명)
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # 투명도 (1.0은 불투명)
            # 생명주기 설정 (0.0은 영구적 표시)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            # 마커 메시지 발행
            self.publisher_.publish(marker)
def main():
    model_path = '/home/hbk/crack_ws/src/my_best.pt'
    rclpy.init()
    model = YOLO(model_path)
    node = Robot2Dectctor(model)
    executor = MultiThreadedExecutor(num_threads=8) # callback 5
    try:
        while rclpy.ok() and not node.should_shutdown:
            # rclpy.spin_once(node, timeout_sec=0.05)
            executor.add_node(node)
            executor.spin_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested via Ctrl+C.")
    finally:
        node.should_shutdown = True
        node.thread.join(timeout=1.0)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()