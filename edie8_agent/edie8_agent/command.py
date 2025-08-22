# command.py

import os 
import time 
import uuid
import re
import threading
import asyncio
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from edie8_agent.state_manager import get_state

# Agent 관련 클래스들 임포트
from edie8_agent.components.models.executor import ActionExecutor

# 도구 관련 모듈 임포트
from edie8_agent.components.toolbox.move import get_robot_pose
from edie8_agent.components.toolbox import move as move_tool
from edie8_agent.components.toolbox import action as action_tool
from edie8_agent.components.toolbox import calculation as calculation_tool

# 유틸리티 함수 임포트
from edie8_agent.components.utils.parser import custom_json_parser
from edie8_agent.components.utils.llm import get_llm_server, get_llm_local, get_llm_api, get_llm
from edie8_agent.components.prompts.prompt import command_executor_prompts


def extract_value(field_name, response):
    match = re.search(fr"{field_name}\s*[:\-]?\s*(.+)", response, re.IGNORECASE)
    return match.group(1).strip() if match else ""

class CommandAgent:
    def __init__(self, robot_node, local=False, verbose: bool = True, temperature: float = 0.6):
        self.robot_node = robot_node
        
        # 사용할 LLM 설정 
        self.agent_llm = get_llm(local=local, model_name="gpt-4.1-mini",temperature=temperature)
        
        # 각 노드별 LLM 또는 Agent 설정 
        self.executor = ActionExecutor(
            llm=self.agent_llm,
            add_prompt=command_executor_prompts,
            tool_packages=[action_tool, move_tool, calculation_tool], 
            verbose=verbose,
            session_id="command"
            )
     
    
# -------------------------------- CommandNode -------------------------------------------

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.robot_node = get_state()
        self.command_agent = CommandAgent(self.robot_node, local=False, verbose=True, temperature=0.6)

        # "/edie8/llm/input" 토픽 구독자 생성
        self.subscription = self.create_subscription(
            String,
            "/edie8/llm/input",
            self.listener_callback,
            10
        )
        self.get_logger().info("CommandNode initialized and subscribed to /edie8/llm/input")

    def listener_callback(self, msg):
        user_input = msg.data
        self.get_logger().info(f"Received input: {user_input}")

        # executor를 동기적으로 실행하여 function calling 및 도구 사용 처리
        try:
            response = self.command_agent.executor.invoke(user_input)
            self.get_logger().info(f"Executor response: {response}")
            # 필요시 결과를 다른 토픽으로 발행하거나 추가 처리 가능
        except Exception as e:
            self.get_logger().error(f"Error executing command: {e}")

    def run(self):
        rclpy.spin(self)
        
def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
