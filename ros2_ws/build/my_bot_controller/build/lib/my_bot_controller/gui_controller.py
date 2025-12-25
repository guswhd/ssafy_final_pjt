#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt, QTimer

class RobotController(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.initUI()
        
        # ROS 통신을 위한 타이머 (Qt 이벤트 루프와 충돌 방지)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)  # 0.05초마다 ROS 메시지 처리

    def initUI(self):
        self.setWindowTitle('SSAFY Balancing Bot Remote')
        self.setGeometry(100, 100, 320, 450)
        self.setStyleSheet("background-color: #f0f0f0;")
        
        layout = QVBoxLayout()

        # 상태 라벨
        self.label = QLabel('READY', self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 20px; font-weight: bold; color: #333; margin-bottom: 20px;")
        layout.addWidget(self.label)

        # 버튼 스타일 정의
        btn_style = """
            QPushButton {
                background-color: #4CAF50; color: white; border-radius: 10px;
                font-size: 16px; font-weight: bold; padding: 15px;
            }
            QPushButton:pressed { background-color: #388E3C; }
        """
        
        # 버튼 레이아웃 구성
        btn_fwd = QPushButton('▲\nFORWARD')
        btn_fwd.setStyleSheet(btn_style)
        btn_fwd.clicked.connect(lambda: self.pub_cmd(0.15, 0.0, "FORWARD"))
        
        btn_back = QPushButton('▼\nBACKWARD')
        btn_back.setStyleSheet(btn_style)
        btn_back.clicked.connect(lambda: self.pub_cmd(-0.15, 0.0, "BACKWARD"))
        
        btn_left = QPushButton('◀ LEFT')
        btn_left.setStyleSheet(btn_style)
        btn_left.clicked.connect(lambda: self.pub_cmd(0.0, 0.8, "TURN LEFT"))
        
        btn_right = QPushButton('RIGHT ▶')
        btn_right.setStyleSheet(btn_style)
        btn_right.clicked.connect(lambda: self.pub_cmd(0.0, -0.8, "TURN RIGHT"))

        btn_stop = QPushButton('STOP (Space)')
        btn_stop.setStyleSheet("background-color: #F44336; color: white; border-radius: 10px; font-size: 18px; font-weight: bold; padding: 15px;")
        btn_stop.clicked.connect(lambda: self.pub_cmd(0.0, 0.0, "STOPPED"))

        # 레이아웃 배치
        layout.addWidget(btn_fwd)
        
        hbox = QHBoxLayout()
        hbox.addWidget(btn_left)
        hbox.addWidget(btn_right)
        layout.addLayout(hbox)
        
        layout.addWidget(btn_back)
        layout.addWidget(btn_stop)

        self.setLayout(layout)

    def pub_cmd(self, lin, ang, text):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.publisher.publish(msg)
        self.label.setText(text)
        if text == "STOPPED":
            self.label.setStyleSheet("font-size: 20px; font-weight: bold; color: red;")
        else:
            self.label.setStyleSheet("font-size: 20px; font-weight: bold; color: green;")

    def spin_ros(self):
        # Qt 루프 안에서 ROS 이벤트를 살짝 처리해줌
        rclpy.spin_once(self.node, timeout_sec=0)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W: self.pub_cmd(0.15, 0.0, "FORWARD")
        elif event.key() == Qt.Key_S: self.pub_cmd(-0.15, 0.0, "BACKWARD")
        elif event.key() == Qt.Key_A: self.pub_cmd(0.0, 0.8, "LEFT")
        elif event.key() == Qt.Key_D: self.pub_cmd(0.0, -0.8, "RIGHT")
        elif event.key() == Qt.Key_Space: self.pub_cmd(0.0, 0.0, "STOPPED")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gui_controller')
    
    app = QApplication(sys.argv)
    ex = RobotController(node)
    ex.show()
    
    # Qt 앱이 종료될 때까지 실행
    exit_code = app.exec_()
    
    # 종료 처리
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
