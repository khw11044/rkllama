# StateNode.py

import math
import time
import threading
import logging
import traceback
from typing import Optional



import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, Int64MultiArray
from std_msgs.msg import String, UInt8, Int8, Bool

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)


import threading


class StateManager(Node):
    def __init__(self):
        super().__init__("state_manager_node")
        # 이동 관련 
        self.move_flag = False
        self.move_start_time = 0.0
        self.move_duration = 0.0
        self.cur_linear_x = 0.0
        self.cur_angular_z = 0.0
        # 귀 움직임 관련 
        self.left_ear_position = 0.5
        self.right_ear_position = 0.5
        # 다리 움직임 관련 
        self.left_leg_position = 0.0
        self.right_leg_position = 0.0
        # 바닥 레이저 
        
        
        
        # ============================ 데이터 발행 (pub) ===============================================================
        
        # 4. 모터 ------------------------------------------------------------------------------------------
        ## 4.1 wheel 명령 발행
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/edie8/diff_drive_controller/cmd_vel_unstamped", 10
        )
        # 4.2 귀 움직임 명령 발행
        self.left_ear_pub = self.create_publisher(
            Float64MultiArray, "/edie8_l_ear_position_controller/commands", 10
        )
        self.right_ear_pub = self.create_publisher(
            Float64MultiArray, "/edie8_r_ear_position_controller/commands", 10
        )
        ## 4.3 다리 움직임 명령 발행 
        self.left_leg_pub = self.create_publisher(
            Float64MultiArray, "/edie8_l_leg_position_controller/commands", 10
        )
        self.right_leg_pub = self.create_publisher(
            Float64MultiArray, "/edie8_r_leg_position_controller/commands", 10
        )
        
        # 기타: 타이머 ------------------------------------------------------------------------------------------
        self.cmd_vel_timer = self.create_timer(0.01, self.cmd_vel_timer_cb)
        
        self.get_logger().info("StateManager initialized.")


    def publish_twist_to_cmd_vel(self, linear_x, angular_z, duration):
        self.cur_linear_x = linear_x
        self.cur_angular_z = angular_z
        self.move_duration = duration
        self.move_start_time = time.time()
        self.move_flag = True

    def cmd_vel_timer_cb(self):
        if self.move_flag:
            elapsed = time.time() - self.move_start_time
            if elapsed < self.move_duration:
                twist = Twist()
                twist.linear.x = self.cur_linear_x
                twist.angular.z = self.cur_angular_z
                self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                self.move_flag = False

    def publish_ear_position(self):
        left_msg = Float64MultiArray()
        right_msg = Float64MultiArray()
        left_msg.data = [self.left_ear_position]
        right_msg.data = [self.right_ear_position]
        self.left_ear_pub.publish(left_msg)
        self.right_ear_pub.publish(right_msg)

    def publish_leg_position(self):
        left_msg = Float64MultiArray()
        right_msg = Float64MultiArray()
        left_msg.data = [self.left_leg_position]
        right_msg.data = [self.right_leg_position]
        self.left_leg_pub.publish(left_msg)
        self.right_leg_pub.publish(right_msg)


# -------------------------------------------------
# ✅ 여기에 싱글턴 get_state 함수 정의!
# -------------------------------------------------
# 전역 상태
_robot_node: StateManager = None
_executor: SingleThreadedExecutor = None
_executor_thread: threading.Thread = None
_initialized = False

# 로깅 설정
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

def get_state() -> StateManager:
    global _robot_node, _executor, _executor_thread, _initialized

    if _robot_node is None:
        if not _initialized:
            if not rclpy.ok():
                logger.info("ROS2 초기화 시작")
                rclpy.init()
            _initialized = True

        # 1. StateManager 생성
        _robot_node = StateManager()

        # 2. SingleThreadedExecutor에 등록
        _executor = SingleThreadedExecutor()
        _executor.add_node(_robot_node)

        # 3. 별도 스레드에서 executor.spin() 실행
        def executor_spin():
            try:
                logger.info("🌀 Executor 스레드 시작")
                _executor.spin()
            except Exception as e:
                logger.error(f"🚨 Executor 오류: {e}")
            finally:
                _executor.shutdown()

        _executor_thread = threading.Thread(target=executor_spin, daemon=True)
        _executor_thread.start()

        logger.info("✅ ROS2 노드 및 Executor 초기화 완료")

    return _robot_node
