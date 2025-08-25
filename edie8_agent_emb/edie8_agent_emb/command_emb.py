# command.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from edie8_agent_emb.state_manager_emb import get_state
from edie8_agent_emb.components.rkllama_core.RKllamaAgent import RKLlamaAgent
from edie8_agent_emb.components.toolbox import move as move_tool
from edie8_agent_emb.components.toolbox import action as action_tool
from edie8_agent_emb.components.toolbox import calculation as calculation_tool
from edie8_agent_emb.components.utils.tool_collector import collect_tools_from_modules
from edie8_agent_emb.components.prompts.agent_prompt import get_prompts, system_prompts
from edie8_agent_emb.components.prompts.prompt import command_executor_prompts



class CommandAgent:
    def __init__(self, robot_node, model_name, num_thread, prompts=None, agent_prompts=None, verbose: bool = True):
        self.robot_node = robot_node
        self.verbose = verbose
        
        if verbose:
            print("🚀 RKLlamaAgent 기반 CommandAgent 초기화 중...")
        
        # 시스템 프롬프트 처리
        final_system_prompt = self._build_system_prompt(prompts, agent_prompts)
        
        # RKLlamaAgent 초기화
        self.agent = RKLlamaAgent(
            model=model_name,
            num_thread=num_thread,
            system_prompt=final_system_prompt
        )
        
        # 툴박스 모듈에서 도구 함수들 자동 수집
        tools = collect_tools_from_modules([action_tool, move_tool, calculation_tool])
        
        # 도구들을 에이전트에 바인딩
        self.agent.bind_tools(tools)
        
        if verbose:
            print(f"✅ CommandAgent 초기화 완료! (등록된 도구 수: {len(tools)})")
            print("📋 등록된 도구들:")
            for tool in tools:
                if hasattr(tool, 'name'):
                    print(f"  • {tool.name}")
                else:
                    print(f"  • {tool.__name__ if hasattr(tool, '__name__') else str(tool)}")
    
    
    def _build_system_prompt(self, prompts, agent_prompts):
        """components/prompts 폴더의 내용을 조합하여 기본 시스템 프롬프트 구성"""
        if prompts is None:
            # 프롬프트 없음 - RKLlamaAgent는 system_prompt 없이도 잘 작동함
            if self.verbose:
                print("📝 시스템 프롬프트 없음 (None)")
            return None
        else:
            
            prompt_parts = []
            
            # 기본 시스템 프롬프트들 추가 (agent_prompt.py에서)
            for role, content in prompts:
                if role == "system":
                    prompt_parts.append(content)
            
            # 로봇별 구체적인 프롬프트 (agent_prompt.py의 get_prompts() 활용)
            robot_prompts = get_prompts()
            prompt_parts.append(str(robot_prompts))
            
            # 명령 실행 관련 프롬프트 (prompt.py의 command_executor_prompts)
            for role, content in agent_prompts:
                if role == "system":
                    # {state} 플레이스홀더 제거하고 일반적인 지침으로 변경
                    clean_content = content.replace("사용자 명령:\n        {state}\n\n        ", "")
                    prompt_parts.append(clean_content)
            
            # 최종 시스템 프롬프트 조합
            final_prompt = "\n\n".join(prompt_parts)
            
            if self.verbose:
                print("📝 생성된 기본 시스템 프롬프트:")
                print("=" * 50)
                print(final_prompt[:300] + "..." if len(final_prompt) > 300 else final_prompt)
                print("=" * 50)
        
            return final_prompt
     
    
# -------------------------------- CommandNode -------------------------------------------

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.robot_node = get_state()
        
        # 시스템 프롬프트 설정
        
        self.command_agent = CommandAgent(
            self.robot_node,
            model_name="Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
            num_thread=4,
            system_prompt=system_prompts,   # 또는 None
            agent_prompt=command_executor_prompts,
            verbose=True
        )
        

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

        # RKLlamaAgent를 사용하여 간단하게 처리
        try:
            response = self.command_agent.agent.invoke(user_input)
            self.get_logger().info(f"🤖 RKLLAMA Response: {response}")
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
