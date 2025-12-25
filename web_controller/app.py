#!/usr/bin/env python3
from flask import Flask, render_template, request, jsonify
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import math

# --- [설정] ---
MAX_LIN_VEL = 0.25
MAX_ANG_VEL = 1.0
LIN_STEP = 0.01
ANG_STEP = 0.1

app = Flask(__name__)

# --- 쿼터니언 -> 오일러 각도 변환 함수 (수학) ---
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

# --- ROS 2 Node ---
class WebTeleop(Node):
    def __init__(self):
        super().__init__('web_teleop_node')
        
        # 1. 명령 보내는 입 (Publisher)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. 위치 듣는 귀 (Subscriber) - RTAB-Map 연결
        self.subscription = self.create_subscription(
            Odometry,
            '/rtabmap/odom',  # RTAB-Map이 주는 토픽
            self.odom_callback,
            10)
        
        self.get_logger().info("Web Teleop + Odometry Monitor Started!")
        
        # 상태 변수
        self.target_lin = 0.0
        self.target_ang = 0.0
        
        # 로봇 위치 정보 (웹으로 보낼 것)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0 # 바라보는 각도 (도)

    def odom_callback(self, msg):
        # 위치 (Position)
        self.pos_x = round(msg.pose.pose.position.x, 2)
        self.pos_y = round(msg.pose.pose.position.y, 2)
        
        # 방향 (Orientation) - 쿼터니언을 각도로 변환
        q = msg.pose.pose.orientation
        yaw_rad = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.heading = round(math.degrees(yaw_rad), 1) # 라디안 -> 도 변환

    def update_speed(self, lin_step, ang_step, is_stop=False):
        if is_stop:
            self.target_lin = 0.0
            self.target_ang = 0.0
        else:
            self.target_lin = round(self.target_lin + lin_step, 2)
            self.target_lin = max(min(self.target_lin, MAX_LIN_VEL), -MAX_LIN_VEL)
            
            self.target_ang = round(self.target_ang + ang_step, 2)
            self.target_ang = max(min(self.target_ang, MAX_ANG_VEL), -MAX_ANG_VEL)

        msg = Twist()
        msg.linear.x = float(self.target_lin)
        msg.angular.z = float(self.target_ang)
        self.publisher.publish(msg)

# 전역 변수
ros_node = None

def start_ros_node():
    global ros_node
    rclpy.init()
    ros_node = WebTeleop()
    rclpy.spin(ros_node)

# --- Flask 라우팅 ---
@app.route('/')
def index():
    return render_template('index.html')

# [추가] 실시간 상태값 요청 처리
@app.route('/status', methods=['GET'])
def status():
    if ros_node:
        return jsonify({
            "lin": ros_node.target_lin,
            "ang": ros_node.target_ang,
            "x": ros_node.pos_x,
            "y": ros_node.pos_y,
            "head": ros_node.heading
        })
    return jsonify({"x": 0, "y": 0, "head": 0})

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    action = data.get('action')
    
    lin_step = 0.0
    ang_step = 0.0
    is_stop = False
    
    if action == 'up': lin_step = LIN_STEP
    elif action == 'down': lin_step = -LIN_STEP
    elif action == 'left': ang_step = ANG_STEP
    elif action == 'right': ang_step = -ANG_STEP
    elif action == 'upleft': lin_step, ang_step = LIN_STEP, ANG_STEP
    elif action == 'upright': lin_step, ang_step = LIN_STEP, -ANG_STEP
    elif action == 'downleft': lin_step, ang_step = -LIN_STEP, ANG_STEP
    elif action == 'downright': lin_step, ang_step = -LIN_STEP, -ANG_STEP
    elif action == 'stop': is_stop = True
        
    if ros_node:
        ros_node.update_speed(lin_step, ang_step, is_stop)
        
    return jsonify({"status": "success"})

if __name__ == '__main__':
    threading.Thread(target=start_ros_node, daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
