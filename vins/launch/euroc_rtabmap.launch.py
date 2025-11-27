from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Static TF Publisher (body -> cam0)
        # VINS의 기준인 'body'와 카메라 'cam0'를 연결합니다.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'cam0']
        ),
        # Cam0 -> Cam1 (Baseline 0.11m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.11007', '0', '0', '0', '0', '0', 'cam0', 'cam1']
        ),

        # 2. Camera Info Publisher (Custom Node)
        # [주의] executable 경로는 본인 환경에 맞게 확인해주세요!
        Node(
            executable='/home/ssafy/ros2_ws/src/VINS-Fusion-ROS2-humble/scripts/euroc_camera_info.py',
            name='camera_info_pub',
            output='screen'
        ),

        # 3. RTAB-Map (Stereo Mode)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'body',            # [수정됨] base_link -> body
                'subscribe_stereo': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'approx_sync': True,
                'queue_size': 20,
                'Rtabmap/ImagesAlreadyRectified': 'false',
                
                # QoS 설정 (필요시 추가, EuRoC는 보통 괜찮음)
                'qos_image': 2,
                'qos_camera_info': 2,
            }],
            remappings=[
                ('left/image_rect', '/cam0/image_raw'),
                ('right/image_rect', '/cam1/image_raw'),
                ('left/camera_info', '/cam0/camera_info'),
                ('right/camera_info', '/cam1/camera_info'),
                ('odom', '/odometry')
            ]
        ),

        # 4. Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            parameters=[{
                'subscribe_stereo': True,
                'subscribe_odom_info': False,
                'queue_size': 20,
                'frame_id': 'body',            # [수정됨] base_link -> body
                'approx_sync': True,           # [추가됨] 중요!
            }],
            remappings=[
                ('left/image_rect', '/cam0/image_raw'),
                ('right/image_rect', '/cam1/image_raw'),
                ('left/camera_info', '/cam0/camera_info'),
                ('right/camera_info', '/cam1/camera_info'),
                ('odom', '/vins_estimator/odometry')
            ]
        ),
    ])