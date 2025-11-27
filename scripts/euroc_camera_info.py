#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import yaml

class EurocCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('euroc_camera_info_publisher')
        
        # 구독: Raw 이미지
        self.sub_cam0 = self.create_subscription(Image, '/cam0/image_raw', self.cam0_callback, 10)
        self.sub_cam1 = self.create_subscription(Image, '/cam1/image_raw', self.cam1_callback, 10)
        
        # 발행: Camera Info
        self.pub_info0 = self.create_publisher(CameraInfo, '/cam0/camera_info', 10)
        self.pub_info1 = self.create_publisher(CameraInfo, '/cam1/camera_info', 10)

        # --- Cam0 (Left) Parameters [From cam0_pinhole.yaml] ---
        self.info0 = CameraInfo()
        self.info0.width = 752
        self.info0.height = 480
        self.info0.distortion_model = "plumb_bob"
        self.info0.d = [-0.295456, 0.086623, 0.000002, 0.000014, 0.0] # k1, k2, p1, p2, k3
        # K Matrix (Intrinsic)
        self.info0.k = [461.1586, 0.0, 362.6593,
                        0.0, 459.7529, 248.5211,
                        0.0, 0.0, 1.0]
        # P Matrix (Projection) - Left camera is reference
        self.info0.p = [461.1586, 0.0, 362.6593, 0.0,
                        0.0, 459.7529, 248.5211, 0.0,
                        0.0, 0.0, 1.0, 0.0]

        # --- Cam1 (Right) Parameters [From cam1_pinhole.yaml] ---
        self.info1 = CameraInfo()
        self.info1.width = 752
        self.info1.height = 480
        self.info1.distortion_model = "plumb_bob"
        self.info1.d = [-0.292941, 0.084798, -0.000299, 0.000300, 0.0]
        # K Matrix
        self.info1.k = [460.0978, 0.0, 373.1492,
                        0.0, 458.9098, 254.4073,
                        0.0, 0.0, 1.0]
        
        # P Matrix (Projection) - Needs Baseline!
        # EuRoC Baseline is approx 0.11m (11cm). Tx = -fx * baseline
        # Tx = -460.0978 * 0.11007 ≈ -50.64
        self.info1.p = [460.0978, 0.0, 373.1492, -50.64, 
                        0.0, 458.9098, 254.4073, 0.0,
                        0.0, 0.0, 1.0, 0.0]

    def cam0_callback(self, msg):
        self.info0.header = msg.header
        self.info0.header.frame_id = "cam0" # TF Frame ID
        self.pub_info0.publish(self.info0)

    def cam1_callback(self, msg):
        self.info1.header = msg.header
        self.info1.header.frame_id = "cam1" # TF Frame ID
        self.pub_info1.publish(self.info1)

def main(args=None):
    rclpy.init(args=args)
    node = EurocCameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()