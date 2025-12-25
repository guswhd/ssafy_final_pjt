# ssafy_final_pjt
Balancing Robot with visual SLAM
컴파일 에러때문에 micro-ros example int32_publisher_custom_transport 기반으로 실행

하드웨어 구성
JGB37-520 Motor * 2
DRV8871 Motor Driver
12V LiPo 3S battery
Pololu D buck converter
ESP32S3 N16R8-WROOM
BNO-085 IMU

프로그램 기본 구성
FreeRTOS 를 이용해 uROS 와 밸런싱 제어 루트를 실행
uROS 의 경우 상태 출력 publisher와 속도, 방향 (Twist)를 받는 subscriber 로 구성
balancing control 의 경우 3개의 PID loop 으로 구성:
    Balancing
    Odometry
    Turn
