"""
rkllama_adapter.py
RKLlama Core를 edie8_agent에서 사용하기 위한 어댑터
"""

import os
import sys
from typing import List, Optional, Any, Dict
from dotenv import load_dotenv

# rkllama_core 모듈 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../../rkllama_function_calling'))

try:
    from rkllama_core import LLM, RKLlamaAgent, tool
except ImportError as e:
    print(f"rkllama_core import 오류: {e}")
    print("rkllama_function_calling 디렉토리가 올바른 위치에 있는지 확인해주세요.")
    raise

# 환경변수 로드
ros_ws = os.environ.get("ROS_WS")
if ros_ws:
    env_path = os.path.join(ros_ws, "src", "edie8_llm", ".env")
    load_dotenv(dotenv_path=env_path)


class RKLlamaAdapter:
    """
    rkllama_core를 edie8_agent에서 사용하기 위한 어댑터 클래스
    기존 LangChain 기반 코드와 호환성을 제공하면서 rkllama_core의 장점을 활용
    """
    
    def __init__(
        self,
        model: str = "Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
        base_url: str = "http://localhost:8080",
        temperature: float = 0.6,
        system_prompt: Optional[str] = None,
        use_agent: bool = True
    ):
        """
        RKLlama 어댑터 초기화
        
        Args:
            model: 사용할 모델명
            base_url: rkllm 서버 URL
            temperature: 샘플링 온도
            system_prompt: 시스템 프롬프트
            use_agent: Agent 사용 여부 (True: RKLlamaAgent, False: LLM)
        """
        self.model = model
        self.base_url = base_url
        self.temperature = temperature
        self.system_prompt = system_prompt
        self.use_agent = use_agent
        
        # LLM 또는 Agent 초기화
        if use_agent:
            self.llm = RKLlamaAgent(base_url=base_url, model=model)
        else:
            self.llm = LLM(
                model=model,
                base_url=base_url,
                temperature=temperature,
                system_prompt=system_prompt
            )
    
    def bind_tools(self, tools: List[Any]) -> 'RKLlamaAdapter':
        """
        도구들을 바인딩 (Agent 모드에서만 사용)
        
        Args:
            tools: 바인딩할 도구 목록
            
        Returns:
            self (메서드 체이닝 지원)
        """
        if self.use_agent and hasattr(self.llm, 'bind_tools'):
            # LangChain 스타일 도구를 rkllama 스타일로 변환
            converted_tools = []
            for tool_item in tools:
                if hasattr(tool_item, 'name') and hasattr(tool_item, 'func'):
                    # LangChain Tool을 rkllama Tool로 변환
                    from rkllama_core import RKLlamaTool
                    rk_tool = RKLlamaTool(
                        tool_item.func,
                        name=tool_item.name,
                        description=tool_item.description
                    )
                    converted_tools.append(rk_tool)
                else:
                    # 이미 rkllama 호환 도구인 경우
                    converted_tools.append(tool_item)
            
            self.llm.bind_tools(converted_tools)
        
        return self
    
    def invoke(self, message: str, **kwargs) -> str:
        """
        LLM 또는 Agent 호출
        
        Args:
            message: 사용자 메시지
            **kwargs: 추가 인수
            
        Returns:
            응답 텍스트
        """
        if self.use_agent:
            return self.llm.invoke(message, system_prompt=self.system_prompt)
        else:
            return self.llm.invoke(message, **kwargs)
    
    def clear_history(self):
        """대화 기록 초기화"""
        if hasattr(self.llm, 'clear_history'):
            self.llm.clear_history()
    
    def list_tools(self):
        """사용 가능한 도구 목록 출력"""
        if hasattr(self.llm, 'list_tools'):
            self.llm.list_tools()


def get_rkllama_llm(
    model_name: str = "Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
    temperature: float = 0.6,
    base_url: str = "http://localhost:8080",
    system_prompt: Optional[str] = None
) -> RKLlamaAdapter:
    """
    기존 get_llm 함수와 호환되는 rkllama LLM 생성 함수
    
    Args:
        model_name: 모델명
        temperature: 샘플링 온도
        base_url: 서버 URL
        system_prompt: 시스템 프롬프트
        
    Returns:
        RKLlamaAdapter 인스턴스
    """
    return RKLlamaAdapter(
        model=model_name,
        base_url=base_url,
        temperature=temperature,
        system_prompt=system_prompt,
        use_agent=False  # 순수 LLM 모드
    )


def get_rkllama_agent(
    model_name: str = "Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
    temperature: float = 0.6,
    base_url: str = "http://localhost:8080",
    system_prompt: Optional[str] = None
) -> RKLlamaAdapter:
    """
    rkllama Agent 생성 함수
    
    Args:
        model_name: 모델명
        temperature: 샘플링 온도
        base_url: 서버 URL
        system_prompt: 시스템 프롬프트
        
    Returns:
        RKLlamaAdapter 인스턴스 (Agent 모드)
    """
    return RKLlamaAdapter(
        model=model_name,
        base_url=base_url,
        temperature=temperature,
        system_prompt=system_prompt,
        use_agent=True  # Agent 모드
    )


# 기존 코드와의 호환성을 위한 함수들
def get_llm_rkllama(
    local: bool = False, 
    model_name: str = "Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
    temperature: float = 0.6
) -> RKLlamaAdapter:
    """
    기존 get_llm 함수와 호환되는 wrapper
    """
    # local 파라미터는 rkllm에서는 의미없음 (항상 로컬 서버 사용)
    return get_rkllama_llm(
        model_name=model_name,
        temperature=temperature
    )


if __name__ == "__main__":
    # 테스트 코드
    try:
        print("rkllama_adapter 테스트 시작...")
        
        # LLM 모드 테스트
        llm = get_rkllama_llm()
        response = llm.invoke("안녕하세요!")
        print(f"LLM 응답: {response}")
        
        # Agent 모드 테스트
        agent = get_rkllama_agent()
        response = agent.invoke("안녕하세요!")
        print(f"Agent 응답: {response}")
        
        print("rkllama_adapter 테스트 완료!")
        
    except Exception as e:
        print(f"테스트 중 오류 발생: {e}")
