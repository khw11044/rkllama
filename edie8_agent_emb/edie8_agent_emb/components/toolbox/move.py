#!/usr/bin/env python3
import sys
import math
import time
import logging
import os
from typing import Any, AsyncIterable, List, Dict, Literal, Optional, Union, Annotated, Tuple
from geometry_msgs.msg import Twist

try:
    from rkllama_core import tool
    from edie8_agent_emb.components.rkllama_core import tool
except ImportError:
    # fallback to langchain if rkllama_core is not available
    print("rkllama_core import 오류 - fallback to LangChain")
    

from edie8_agent_emb.state_manager_emb import get_state

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    force=True,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, float(value)))

def refine_value(value):
    if abs(value) < 0.5:
        value = value * 2
    return value
        

def get_quaternion_from_degree(degree):
    rad = math.radians(degree)
    qz = math.sin(rad / 2)
    qw = math.cos(rad / 2)
    return 0.0, 0.0, qz, qw

def get_yaw_from_quaternion(qx, qy, qz, qw):
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return yaw

def get_quaternion_from_yaw(yaw):
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return 0.0, 0.0, qz, qw

# --------------------------------
# LangChain 도구 함수
# --------------------------------

max_linear_speed = float(2.5)
max_angular_speed = float(5.0)

def decelerate(agent, linear_x: float = 1.0, angular_z: float = 1.0, steps=4, step_time=0.1):
    for i in range(steps, 0, -1):
        factor = i / steps
        twist = Twist()
        twist.linear.x = linear_x * factor
        twist.angular.z = angular_z * factor
        agent.cmd_vel_pub.publish(twist)
        time.sleep(step_time)
    agent.cmd_vel_pub.publish(Twist())  # 최종 정지

def simple_move(agent, linear_x: float = 1.0, angular_z: float = 1.0, duration: float=1.0) -> str:
    start = time.time()
    while True:
        # agent.monitoring_sensor()
        if time.time() - start > duration:
            agent.move_flag = False
            decelerate(agent, linear_x, angular_z)  # ← 부드럽게 멈춤
            return "이동 완료"

        agent.publish_twist_to_cmd_vel(linear_x, angular_z, 0.01)
        time.sleep(0.01)

def stop_mode(agent):
    msg = Twist()
    # 정지 및 상태 초기화
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    agent.cmd_vel_pub.publish(msg)
    agent.cmd_vel_pub.publish(Twist())
    



@tool
def get_robot_pose() -> str:
    """
    Retrieve the robot's current position including x, y, and yaw (in radians).

    Returns:
        A string describing the current position in the format:
        'x=<value>m, y=<value>m, yaw=<value>rad'.
        If odometry data is unavailable, a warning message is returned.
    """
    try:
        agent = get_state()
        
        x, y, z = agent.current_position
        yaw = agent.current_yaw
        qx, qy, qz, qw = agent.current_orientation
        
        if not agent.odom_received:
            logger.warning("Odometry 데이터 미수신")
            return "현재 위치: 데이터 없음 (odom 미수신)"
        return f"현재 위치: x={x:.2f}m, y={y:.2f}m, yaw={yaw:.2f}rad"
    except Exception as e:
        logger.error(f"위치 조회 오류: {str(e)}")
        return f"위치 조회 오류: {str(e)}"




@tool
def simple_move_perform(
    linear_x: Annotated[float, "Linear velocity in m/s. Recommended: [-1.5, -1.0, 0.0, 1.0, 1.5]"] = 1.0,
    angular_z: Annotated[float, "Angular velocity in rad/s. Recommended: [-2.0, -1.5, 0.0, 1.5, 2.0]"] = 1.0,
    duration: Annotated[float, "Duration to move in seconds."] = 10.0
) -> str:
    """
    Executes a simple motion behavior for the robot, such as moving forward, rotating, or backing up.

    This tool is suitable for basic commands like "move forward for 2 seconds".

    - Use angular_z = 0.0 and non-zero linear_x for straight motion (forward or backward).
    - Use linear_x = 0.0 and non-zero angular_z for in-place rotation.
    - 제자리에서 멈추기 위해서 linear_x=0.0 그리고 angular_z=0.0 으로 설정하면 됩니다.
    
    Args:
        linear_x: Linear velocity in meters per second (m/s).
        angular_z: Angular velocity in radians per second (rad/s).
        duration: How long the motion should last (in seconds).

    Returns:
        A string describing the result of the motion command.
    """
    
    linear_x = float(linear_x)
    angular_z = float(angular_z)
    duration = float(duration)
    linear_x = refine_value(linear_x)
    angular_z = refine_value(angular_z)
    
    agent = get_state()

    result = simple_move(agent, linear_x, angular_z, duration)
    return result
 

@tool
def simple_rotation_perform(
    angular_z: Annotated[float, "Angular velocity in rad/s. Recommended: [-2.0, -1.5, 0.0, 1.5, 2.0]"] = 1.0,
    duration: Annotated[float, "Duration to move in seconds."] = 10.0
) -> str:
    """
    Executes a simple motion behavior for the robot, such as rotating.

    This tool is suitable for basic commands like "제자리에서 돌아".
    반드시 명령에 "제자리" 가 있을 경우에만 해당 도구를 사용합니다.
    
    Args:
        angular_z: Angular velocity in radians per second (rad/s).
        duration: How long the motion should last (in seconds).

    Returns:
        A string describing the result of the motion command.
    """
    
    angular_z = float(angular_z)
    duration = float(duration)
    
    angular_z = refine_value(angular_z)
    
    agent = get_state()
    
    result = simple_move(agent, 0.0, angular_z, duration)
    return result
 

@tool
def circle_move_perform(
    linear_x: Annotated[float, "Linear velocity in m/s. Recommended: [0.5, 1.0, 1.5] for circular motion."] = 0.1,
    angular_z: Annotated[float, "Angular velocity in rad/s. Recommended: [-3.0, -2.0, 2.0, 3.0] for circular motion."] = 0.1,
    duration: Annotated[float, "Duration to move in seconds. Recommended: > 10 seconds."] = 10.0
) -> str:
    """
    Makes the robot move in a circular path using linear and angular velocity.

    The robot will draw a circle or arc depending on the velocity ratio.
    "에디야 돌아, 한바퀴 돌아, 돌아봐"라는 명령을 수행하기 위해서 해당 도구를 사용하세요.

    - If linear_x is too small (< 0.1), it will be doubled.
    - If angular_z is too small (< 0.1), it will also be doubled.
    - This ensures a visible circular motion.

    Args:
        linear_x: Linear velocity (m/s).
        angular_z: Angular velocity (rad/s).
        duration: How long the robot should move in a circle (in seconds).

    Returns:
        A string describing the result of the circular movement.
    """
    linear_x = float(linear_x)
    angular_z = float(angular_z)
    duration = float(duration)
    
    linear_x = refine_value(linear_x)
    angular_z = refine_value(angular_z)
        
    agent = get_state()

    result = simple_move(agent, linear_x, angular_z, duration)
    return result
