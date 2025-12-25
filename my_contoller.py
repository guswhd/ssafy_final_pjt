import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSlider, QLabel
from PyQt5.QtCore import Qt, QTimer

class MyRobotController(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
        # ROS 2 노드 초기화
        rclpy.init(args=None)
        self.node = rclpy.create_node('custom_controller_gui')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # 주기적으로 ROS 통신을 처리하기 위한 타이머
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100) # 0.1초마다

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0

    def initUI(self):
        self.setWindowTitle('SSAFY Balancing Bot Controller')
        self.setGeometry(100, 100, 300, 400)
        layout = QVBoxLayout()

        # 속도 표시 라벨
        self.label = QLabel('Ready to Connect', self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 15px; font-weight: bold; color: blue;")
        layout.addWidget(self.label)

        # 컨트롤 버튼들
        btn_layout = QVBoxLayout()
        
        btn_fwd = QPushButton('Forward (▲)')
        btn_fwd.clicked.connect(lambda: self.set_vel(0.1, 0.0, "Moving Forward"))
        
        btn_back = QPushButton('Backward (▼)')
        btn_back.clicked.connect(lambda: self.set_vel(-0.1, 0.0, "Moving Backward"))
        
        btn_left = QPushButton('Turn Left (◀)')
        btn_left.clicked.connect(lambda: self.set_vel(0.0, 0.5, "Turning Left"))
        
        btn_right = QPushButton('Turn Right (▶)')
        btn_right.clicked.connect(lambda: self.set_vel(0.0, -0.5, "Turning Right"))

        btn_stop = QPushButton('EMERGENCY STOP (SPACE)')
        btn_stop.setStyleSheet("background-color: red; color: white; font-weight: bold; height: 50px;")
        btn_stop.clicked.connect(lambda: self.set_vel(0.0, 0.0, "STOPPED"))

        btn_layout.addWidget(btn_fwd)
        hbox = QHBoxLayout()
        hbox.addWidget(btn_left)
        hbox.addWidget(btn_right)
        btn_layout.addLayout(hbox)
        btn_layout.addWidget(btn_back)
        layout.addLayout(btn_layout)
        layout.addWidget(btn_stop)

        self.setLayout(layout)

    def set_vel(self, lin, ang, text):
        self.target_linear = lin
        self.target_angular = ang
        self.label.setText(text)
        self.publish_cmd()

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.target_linear)
        msg.angular.z = float(self.target_angular)
        self.publisher.publish(msg)

    def spin_ros(self):
        # ROS 2 콜백 처리 (이게 없으면 노드가 멈춤)
        rclpy.spin_once(self.node, timeout_sec=0)

    # 키보드 제어 추가 (WASD)
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            self.set_vel(0.15, 0.0, "Key: Forward")
        elif event.key() == Qt.Key_S:
            self.set_vel(-0.15, 0.0, "Key: Backward")
        elif event.key() == Qt.Key_A:
            self.set_vel(0.0, 0.8, "Key: Left")
        elif event.key() == Qt.Key_D:
            self.set_vel(0.0, -0.8, "Key: Right")
        elif event.key() == Qt.Key_Space:
            self.set_vel(0.0, 0.0, "Key: STOP")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyRobotController()
    ex.show()
    sys.exit(app.exec_())
