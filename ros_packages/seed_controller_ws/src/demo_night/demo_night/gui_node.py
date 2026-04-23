import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QHBoxLayout
from PyQt6.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt6.QtGui import QPainter, QColor, QPen

# --- ROS2 Worker Thread ---
class ROS2Thread(QThread, Node):
    gui_state_signal = pyqtSignal(str)
    
    def __init__(self):
        QThread.__init__(self)
        Node.__init__(self, 'pyqt_gui_node')
        
        self.state_pub = self.create_publisher(String, '/gui/system_state', 10)
        self.create_subscription(Float32MultiArray, '/commander/wheel_cmd', self.cmd_callback, 10)
        
        self.current_state = "IDLE"
        self.current_wl = 0.0
        self.current_wr = 0.0

    def cmd_callback(self, msg):
        self.current_wl = msg.data[0]
        self.current_wr = msg.data[1]

    def set_state(self, new_state):
        self.current_state = new_state
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.gui_state_signal.emit(self.current_state)

    def run(self):
        rclpy.spin(self)

# --- Virtual Twin Canvas ---
class TwinCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(400, 400)
        self.robot_x = 200.0
        self.robot_y = 200.0
        self.robot_theta = 0.0
        
        # Loaded from your dead-reckoning calibration
        self.cmd_to_mps = 0.005 
        self.wheel_base = 0.4 # meters

    def update_kinematics(self, wl, wr, dt=0.1):
        v_left = wl * self.cmd_to_mps
        v_right = wr * self.cmd_to_mps
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        
        self.robot_theta += omega * dt
        self.robot_x += v * math.cos(self.robot_theta) * 10 # Scale for display
        self.robot_y += v * math.sin(self.robot_theta) * 10 # Scale for display
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setBrush(QColor(30, 30, 30))
        painter.drawRect(self.rect())
        
        # Draw Robot
        painter.translate(self.robot_x, self.robot_y)
        painter.rotate(math.degrees(self.robot_theta))
        painter.setBrush(QColor(0, 255, 0))
        painter.drawEllipse(-10, -10, 20, 20)
        painter.setPen(QPen(Qt.GlobalColor.white, 2))
        painter.drawLine(0, 0, 15, 0) # Direction vector

# --- Main Window ---
class MainWindow(QMainWindow):
    def __init__(self, ros_thread):
        super().__init__()
        self.setWindowTitle("Weeding Robot Commander GUI")
        self.ros_thread = ros_thread
        
        self.canvas = TwinCanvas()
        
        # Buttons
        self.btn_idle = QPushButton("IDLE / STOP")
        self.btn_rehome = QPushButton("REHOME SEQUENCE")
        self.btn_optical = QPushButton("OPTICAL PATHING")
        self.btn_traj = QPushButton("TRAJECTORY PATHING")
        
        self.btn_idle.clicked.connect(lambda: self.ros_thread.set_state("IDLE"))
        self.btn_rehome.clicked.connect(lambda: self.ros_thread.set_state("REHOME"))
        self.btn_optical.clicked.connect(lambda: self.ros_thread.set_state("OPTICAL"))
        self.btn_traj.clicked.connect(lambda: self.ros_thread.set_state("TRAJECTORY"))

        # Layout
        vbox = QVBoxLayout()
        vbox.addWidget(QLabel("Commander Mode Select"))
        vbox.addWidget(self.btn_idle)
        vbox.addWidget(self.btn_rehome)
        vbox.addWidget(self.btn_optical)
        vbox.addWidget(self.btn_traj)
        
        hbox = QHBoxLayout()
        hbox.addWidget(self.canvas)
        hbox.addLayout(vbox)
        
        container = QWidget()
        container.setLayout(hbox)
        self.setCentralWidget(container)

        # Kinematic Update Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_twin)
        self.timer.start(100) # 10Hz

    def update_twin(self):
        self.canvas.update_kinematics(self.ros_thread.current_wl, self.ros_thread.current_wr)

if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    
    ros_thread = ROS2Thread()
    ros_thread.start()
    
    window = MainWindow(ros_thread)
    window.show()
    
    sys.exit(app.exec())