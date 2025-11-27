# VINS-Fusion for ROS 2 Humble (CPU Mode)

**Tested with Intel RealSense D435i (Mono + IMU)**

이 프로젝트는 **VINS-Fusion**을 ROS 2 Humble 환경에서 동작하도록 포팅한 버전
👉 (_기반 저장소: [JanekDev/VINS-Fusion-ROS2-humble](https://github.com/JanekDev/VINS-Fusion-ROS2-humble)_)
을 바탕으로, **CPU-only(Non-CUDA)** 환경과 **RealSense D435i**에서 안정적으로 작동하도록 수정한 버전입니다.

---

## 1. Prerequisites (환경 및 의존성)

### **System**

- **OS:** Ubuntu 22.04 LTS (Jammy)
- **ROS:** ROS 2 Humble Hawksbill
- **Hardware:** Intel RealSense D435i (RGB-D + IMU)

### **Dependencies 설치**

```bash
sudo apt update
sudo apt install -y ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-ros \
    ros-humble-realsense2-camera
```

#### Ceres Solver

```bash
sudo apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev \
    libsuitesparse-dev libceres-dev
```

---

## 2. Source Code Modifications (코드 수정 사항)

ROS 2 Humble / Non-CUDA 환경에서 빌드하기 위한 필수 수정입니다.

---

### **2.1 Disable GPU/CUDA**

`vins/src/featureTracker/feature_tracker.h`

```cpp
// #define GPU_MODE 1   // 주석 처리하여 CPU 모드 사용
```

---

### **2.2 Ceres Solver Options 수정**

`vins/src/estimator/estimator.cpp` (약 Line 1171)

```cpp
// options.dense_linear_algebra_library_type = ceres::CUDA;  // 에러 발생
options.dense_linear_algebra_library_type = ceres::EIGEN;    // EIGEN 사용
```

---

### **2.3 Fix rclcpp::Duration (ROS 2 Humble 호환성)**

`loop_fusion/src/utility/CameraPoseVisualization.cpp` (약 Line 95)

```cpp
// marker.lifetime = rclcpp::Duration(0);   // 에러
marker.lifetime = rclcpp::Duration(0, 0);   // (sec, nanosec)
```

---

### **2.4 Fix Sensor QoS (RealSense 호환성 필수)**

`vins/src/rosNodeTest.cpp`

RealSense는 **Best Effort** QoS를 사용하므로, Subscriber QoS를 아래처럼 변경해야 합니다.

```cpp
imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), imu_callback);

img0_sub = this->create_subscription<sensor_msgs::msg::Image>(
    img0_topic, rclcpp::SensorDataQoS(), img0_callback);
```

---

## 3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 4. Configuration (설정 파일)

`src/VINS-Fusion-ROS2-humble/config/my_realsense_config.yaml` 생성

> ⚠️ **중요:**
>
> - Mono + IMU → `num_of_cam: 1`
> - IMU 토픽: `unite_imu_method:=2` 사용 시 `/camera/camera/imu`
> - 캘리브레이션 파일은 **절대 경로 금지**, 반드시 **상대 경로 사용**

---

### **my_realsense_config.yaml**

```yaml
%YAML:1.0

# --- Sensor Settings ---
imu: 1
num_of_cam: 1

# --- Topic Names ---
imu_topic: "/camera/camera/imu"
image0_topic: "/camera/camera/infra1/image_rect_raw"
output_path: "/tmp/"

# --- Calibration Files ---
cam0_calib: "realsense_d435i/left.yaml"
image_width: 640
image_height: 480

# --- Extrinsic Parameter ---
estimate_extrinsic: 1
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1, 0, 0, -0.01,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1 ]

# --- Algorithm Settings ---
multiple_thread: 1
max_cnt: 150
min_dist: 30
freq: 10
F_threshold: 1.0
show_track: 1
flow_back: 1

max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 10.0

# IMU Noise (D435i)
acc_n: 0.1
gyr_n: 0.01
acc_w: 0.001
gyr_w: 0.0001
g_norm: 9.805

estimate_td: 0
td: 0.00
load_previous_pose_graph: 0
pose_graph_save_path: "/tmp/pose_graph/"
save_image: 0
```

---

## 5. Running

3개의 터미널에서 실행합니다.

---

### **Terminal 1 — RealSense Camera**

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 \
    enable_sync:=true \
    enable_infra1:=true \
    enable_infra2:=true \
    depth_module.profile:=640x480x30
```

---

### **Terminal 2 — VINS-Fusion**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run vins vins_node src/VINS-Fusion-ROS2-humble/config/my_realsense_config.yaml
```

> 📌 실행 후 "waiting for image..." → 카메라를 천천히 움직여서 초기화하세요.

---

### **Terminal 3 — RViz**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d src/VINS-Fusion-ROS2-humble/config/vins_rviz_config.rviz
```

> ⚠️ RViz Fixed Frame → `world` 또는 `odom`

---

## 6. Troubleshooting

### **Segmentation Fault**

가능한 원인:

1. **config.yaml 경로 오류**

   - 절대 경로 사용 시
     `/config//home/...` 같은 잘못된 경로 생성
   - 반드시 상대 경로 사용 (`realsense_d435i/left.yaml`)

2. **QoS mismatch (가장 흔함)**

   - Subscriber를 `SensorDataQoS()`로 수정해야 정상 작동

---

### **Incompatible QoS Warning**

RealSense는 Best Effort → VINS는 Reliable
→ 반드시 QoS 수정 필요 (위 2.4 항목)

---

### **Waiting for image and imu... (무한 대기)**

확인 Checklist:

- `ros2 topic list`
  → 토픽 이름이 config와 일치하는가?

- unite_imu_method:=2 옵션 사용했는가?
  → 사용하지 않으면 IMU 발행 안 됨

---

## Summary

이 패치는 다음 사용자에게 적합합니다:

 ROS 2 Humble 환경에서 CUDA 없이 CPU-only로 RealSense D435i Mono + IMU 조합을 안정적으로 VINS-Fusion에 연결하여 사용하고 싶은 사용자
