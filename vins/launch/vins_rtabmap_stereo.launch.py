from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. VINS-Fusion 설정
    vins_path = get_package_share_directory('vins')
    # config 파일 경로가 맞는지 다시 한 번 확인하세요!
    vins_config = os.path.join(vins_path, 'config', 'my_realsense_config.yaml') 

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. TF: body(VINS) -> camera_link(RealSense) 연결
        # ---------------------------------------------------------
        # (임시로 0,0,0으로 설정. 추후 정확한 값 입력 권장)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'camera_link']
        ),

        # ---------------------------------------------------------
        # 2. RTAB-Map (Mapping Node)
        # ---------------------------------------------------------
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'body',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': True,      # Stereo 사용
                'subscribe_odom_info': False,
                'approx_sync': True,
                'queue_size': 20,              # 큐 사이즈 넉넉하게
                
                # --- [핵심 수정] QoS 설정 (2: Best Effort) ---
                'qos_image': 2,        # 이미지 데이터 Best Effort 허용
                'qos_camera_info': 2,  # 카메라 정보 Best Effort 허용
                
                # VINS Odometry 사용
                'odom_frame_id': 'world',
                'publish_tf': False,           # Odom TF는 VINS가 담당
                'wait_for_transform': 0.2,
                
                # Stereo Matching 파라미터 (지도 품질 조절)
                'Stereo/MinDisparity': '0',
                'Stereo/MaxDisparity': '128',
                'Stereo/OpticalFlow': 'false', # Optical Flow 대신 Stereo Matching 사용
                'Vis/MinInliers': '10',
                'Rtabmap/DetectionRate': '2',  # 지도 업데이트 속도 (Hz) - CPU 부하 줄임
            }],
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
                ('left/camera_info', '/camera/camera/infra1/camera_info'),
                ('right/camera_info', '/camera/camera/infra2/camera_info'),
                ('odom', '/vins_estimator/odometry')
            ]
        ),
        
        # ---------------------------------------------------------
        # 3. Visualization (RTAB-Map GUI)
        # ---------------------------------------------------------
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': True,
                'subscribe_odom_info': False,
                'frame_id': 'body',
                'approx_sync': True,
                
                # --- [핵심 수정] GUI도 QoS 맞춰줘야 함 ---
                'qos_image': 2,
                'qos_camera_info': 2,
            }],
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
                ('left/camera_info', '/camera/camera/infra1/camera_info'),
                ('right/camera_info', '/camera/camera/infra2/camera_info'),
                ('odom', '/vins_estimator/odometry')
            ]
        ),
    ])