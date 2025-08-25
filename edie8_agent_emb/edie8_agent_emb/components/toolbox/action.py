# action.py
import time
from typing import Any, AsyncIterable, List, Dict, Literal, Optional, Union, Annotated

from edie8_agent_emb.components.rkllama_core import tool

from edie8_agent_emb.state_manager_emb import get_state
from geometry_msgs.msg import Twist


max_linear_speed = float(2.5)
max_angular_speed = float(5.0)

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, float(value)))

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
def dance_all(
    left_ear: Annotated[list, "List of left ear positions (0.0 ~ 1.0)"],
    right_ear: Annotated[list, "List of right ear positions (0.0 ~ 1.0)"],
    left_leg: Annotated[list, "List of left leg positions (0.0 ~ 3000.0)"],
    right_leg: Annotated[list, "List of right leg positions (0.0 ~ 3000.0)"],
    interval: Annotated[list, "List of timing intervals for each movement (seconds)"],
    linear_x: Annotated[float, "Linear velocity in m/s. Recommended: [-0.5, 0.0, 0.5]"] = 0.0,
    angular_z: Annotated[float, "Angular velocity in rad/s. Recommended: [-2.0, 0.0, 2.0]"] = 0.0,
    duration: Annotated[float, "Duration to perform the dance in seconds. Recommended: [0.0, 0.5, 1.0, 3.0]"] = 0.0
) -> str:
    """
    This tool function makes Edie perform the ultimate dance using ears, legs, and wheels.

    The lengths of the input lists can differ; the function cycles through shorter lists repeatedly to match the longest list length.
    Each element is clamped to its valid range.
    Moves the ears and legs to each specified position sequentially, waiting for the specified interval between movements.
    Finally, resets all positions to default (0.0).

    Args:
        left_ear (list): Left ear positions (floats between 0.0 and 1.0).
        right_ear (list): Right ear positions (floats between 0.0 and 1.0).
        left_leg (list): Left leg positions (floats between 0.0 and 3000.0).
        right_leg (list): Right leg positions (floats between 0.0 and 3000.0).
        interval (list): Timing intervals (seconds) for each movement.
        linear_x (float): Linear velocity in m/s. Defaults to 0.0.
        angular_z (float): Angular velocity in rad/s. Defaults to 0.0.
        duration (float): Duration to perform the dance in seconds. Defaults to 0.0.

    Returns:
        str: Confirmation message indicating completion of the ultimate dance.
    """
    max_len = max(len(left_ear), len(right_ear), len(left_leg), len(right_leg), len(interval))

    agent = get_state()

    agent.publish_twist_to_cmd_vel(linear_x, angular_z, duration)
    
    for i in range(max_len):
        l_ear_pos = clamp(left_ear[i % len(left_ear)], 0.0, 1.0)
        r_ear_pos = clamp(right_ear[i % len(right_ear)], 0.0, 1.0)
        l_leg_pos = clamp(left_leg[i % len(left_leg)], 0.0, 3000.0)
        r_leg_pos = clamp(right_leg[i % len(right_leg)], 0.0, 3000.0)
        timing = clamp(interval[i % len(interval)], 0.0, 2.0)  

        agent.left_ear_position = l_ear_pos
        agent.right_ear_position = r_ear_pos
        agent.left_leg_position = l_leg_pos
        agent.right_leg_position = r_leg_pos
        agent.publish_ear_position()
        agent.publish_leg_position()
        time.sleep(timing)

    # Reset all positions to default
    agent.left_ear_position = 0.0
    agent.right_ear_position = 0.0
    agent.left_leg_position = 0.0
    agent.right_leg_position = 0.0
    agent.publish_ear_position()
    agent.publish_leg_position()

    return f"Dance movements executed {max_len} times with cycling shorter lists."




 
@tool
def action_ears(
    left_pos: Annotated[float, "Left ear position in range [0.0 ~ 1.047]"] = 1.0,
    right_pos: Annotated[float, "Right ear position in range [0.0 ~ 1.047]"] = 1.0,
) -> str:
    """
    Sets the position of the left and right ears.
    
    Args:
        left_pos: Position of the left ear in the range [0.0 ~ 1.047]
        right_pos: Position of the right ear in the range [0.0 ~ 1.047]

    Returns:
        A confirmation string of ear positions.
    """
    
    left_pos = float(left_pos)
    right_pos = float(right_pos)
    
    agent = get_state()
    
    agent.left_ear_position = left_pos
    agent.right_ear_position = right_pos
    agent.publish_ear_position()
    return f"Set ear positions: left={left_pos}, right={right_pos}"

@tool
def reset_ears() -> str:
    """
    Resets both ears to their default position (0.0, 0.0).

    Returns:
        A confirmation string.
    """
    agent = get_state()

    agent.left_ear_position = 0.0
    agent.right_ear_position = 0.0
    agent.publish_ear_position()
    return "Ears reset to default position (0.0, 0.0)."

@tool
def wave_ears(
    left: Annotated[list, "List of left ear positions"],
    right: Annotated[list, "List of right ear positions"],
    interval: Annotated[list, "List of timing intervals for each movement"],
) -> str:
    """
    A tool function to move ears in various patterns.

    Sequentially executes the movements specified in the left and right ear position lists along with the corresponding timing intervals.
    The lengths of the lists can differ; the function cycles through shorter lists repeatedly to match the longest list length.
    Each element in the left, right, and interval lists is automatically clamped to a float value between 0.0 and 1.0.
    Moves the ears to each specified position and waits for the specified interval before moving to the next.
    Finally, resets the ear positions to 0.0.

    Args:
        left (list): List of left ear positions, each a float between 0.0 and 1.0.
        right (list): List of right ear positions, each a float between 0.0 and 1.0.
        interval (list): List of timing intervals (in seconds) for each movement.

    Returns:
        str: Confirmation message indicating completion of ear movements.
    """

    max_len = max(len(left), len(right), len(interval))

    agent = get_state()

    for i in range(max_len):
        l_pos = clamp(left[i % len(left)], 0.0, 1.0)
        r_pos = clamp(right[i % len(right)], 0.0, 1.0)
        timing = clamp(interval[i % len(interval)], 0.0, 2.0)

        agent.left_ear_position = l_pos
        agent.right_ear_position = r_pos
        agent.publish_ear_position()
        time.sleep(timing)

    # Reset ear positions to default
    agent.left_ear_position = 0.0
    agent.right_ear_position = 0.0
    agent.publish_ear_position()

    return f"귀 움직여서 나의 감정을 표현했어요."



@tool
def action_legs(
    left_pos: Annotated[float, "Left leg position [0.0, 1500.0, 3000.0]"] = 0.0,
    right_pos: Annotated[float, "Right leg position [0.0, 1500.0, 3000.0]"] = 0.0,
) -> str:
    """
    Sets the position of the left and right legs.
    
    A value of 0.0 indicates the default leg position, while 3000.0 represents a fully raised leg.

    Args:
        left_pos: Left leg position [0.0, 1500.0, 3000.0]
        right_pos: Right leg position [0.0, 1500.0, 3000.0]

    Returns:
        A confirmation string of leg positions.
    """
    
    left_pos = float(left_pos)
    right_pos = float(right_pos)
    
    agent = get_state()

    agent.left_leg_position = left_pos
    agent.right_leg_position = right_pos
    agent.publish_leg_position()
    return f"Set leg positions: left={left_pos}, right={right_pos}"



@tool
def reset_legs() -> str:
    """
    Resets both legs to their default position (0.0, 0.0).

    Returns:
        A confirmation string.
    """
    agent = get_state()

    agent.left_leg_position = 0.0
    agent.right_leg_position = 0.0
    agent.publish_leg_position()
    return "Legs reset to default position (0.0, 0.0)."


@tool
def wave_legs(
    left: Annotated[list, "List of left leg positions"],
    right: Annotated[list, "List of right leg positions"],
    interval: Annotated[list, "List of timing intervals for each movement"],
) -> str:
    """
    Moves legs according to the specified position lists and timing intervals.

    The lengths of the left, right, and interval lists can differ; the function cycles through shorter lists repeatedly to match the longest list length.
    Each element in the left and right lists is clamped to the valid range [0.0, 3000.0].
    The legs are moved to each specified position sequentially, waiting for the specified interval between movements.
    Finally, leg positions are reset to 0.0.

    Args:
        left (list): List of left leg positions (floats between 0.0 and 3000.0).
        right (list): List of right leg positions (floats between 0.0 and 3000.0).
        interval (list): List of timing intervals (in seconds) for each movement (floats between 0.0 and 1.0).

    Returns:
        str: Confirmation message indicating completion of leg movements.
    """

    max_len = max(len(left), len(right), len(interval))

    agent = get_state()


    for i in range(max_len):
        l_pos = clamp(left[i % len(left)], 0.0, 3000.0)
        r_pos = clamp(right[i % len(right)], 0.0, 3000.0)
        timing = clamp(interval[i % len(interval)], 0.0, 2.0)

        agent.left_leg_position = l_pos
        agent.right_leg_position = r_pos
        agent.publish_leg_position()
        time.sleep(timing)

    agent.left_leg_position = 0.0
    agent.right_leg_position = 0.0
    agent.publish_leg_position()

    return f"Leg movements executed {max_len} times with cycling shorter lists."


@tool
def action_legs_and_ears(
    l_leg: Annotated[float, "Left leg position [0.0, 1500.0, 3000.0]"],
    r_leg: Annotated[float, "Right leg position [0.0, 1500.0, 3000.0]"],
    l_ear: Annotated[float, "Left ear position [0.0 ~ 1.0]"],
    r_ear: Annotated[float, "Right ear position [0.0 ~ 1.0]"],
) -> str:
    """
    Sets the positions of both legs and ears simultaneously.

    Args:
        l_leg: Left leg position [0.0, 1500.0, 3000.0]
        r_leg: Right leg position [0.0, 1500.0, 3000.0]
        l_ear: Left ear position [0.0 ~ 1.0]
        r_ear: Right ear position [0.0 ~ 1.0]

    Returns:
        A confirmation summary of all positions set.
    """
    
    l_leg = float(l_leg)
    r_leg = float(r_leg)
    l_ear = float(l_ear)
    r_ear = float(r_ear)
    
    agent = get_state()


    agent.left_leg_position = l_leg
    agent.right_leg_position = r_leg
    agent.left_ear_position = l_ear
    agent.right_ear_position = r_ear
    agent.publish_leg_position()
    agent.publish_ear_position()
    return (f"Set leg and ear positions:\n"
            f"  Legs: left={l_leg}, right={r_leg}\n"
            f"  Ears: left={l_ear}, right={r_ear}")
