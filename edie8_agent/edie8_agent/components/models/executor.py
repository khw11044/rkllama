import sys
import os
import json
import random
import argparse
from typing import Any, List, Dict, Optional, Union

# rkllama_core 모듈 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../../rkllama_function_calling'))

try:
    from rkllama_core import tool
except ImportError:
    print("rkllama_core import 오류 - fallback to LangChain")
    from langchain.tools import tool

from edie8_agent.components.utils.rkllama_adapter import get_rkllama_agent
from edie8_agent.components.toolbox import ToolBox
from edie8_agent.components.prompts.agent_prompt import system_prompts, get_prompts
from edie8_agent.state_manager import get_state


class ActionExecutor:
    def __init__(
        self, 
        llm=None,
        add_prompt=None,
        tools: Optional[list] = None,
        tool_packages: Optional[list] = None,
        session_id: str = "default", 
        accumulate_chat_history: bool = True,
        verbose: bool = True,
        use_rkllama: bool = True,
    ):
        self.__session_id = session_id
        self.__verbose = verbose
        self.__accumulate_chat_history = accumulate_chat_history
        self.last_answer = None
        self.__agent_state = get_state()
        
        # rkllama_core 사용 여부 결정
        if use_rkllama and 'rkllama_core' in sys.modules:
            self._init_rkllama_executor(add_prompt, tools, tool_packages)
        else:
            self._init_langchain_executor(llm, add_prompt, tools, tool_packages)
    
    def _init_rkllama_executor(self, add_prompt, tools, tool_packages):
        """rkllama_core 기반 executor 초기화"""
        try:
            # 시스템 프롬프트 생성
            system_prompt = self._build_system_prompt(add_prompt)
            
            # rkllama agent 생성
            self.__agent = get_rkllama_agent(
                model_name="Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
                temperature=0.6,
                system_prompt=system_prompt
            )
            
            # 도구 바인딩
            if tool_packages or tools:
                all_tools = []
                if tool_packages:
                    for package in tool_packages:
                        all_tools.extend(self._extract_tools_from_package(package))
                if tools:
                    all_tools.extend(tools)
                
                self.__agent.bind_tools(all_tools)
            
            self.use_rkllama = True
            if self.__verbose:
                print("✅ rkllama_core 기반 ActionExecutor 초기화 완료")
                
        except Exception as e:
            print(f"⚠️ rkllama_core 초기화 실패: {e}")
            print("🔄 LangChain으로 fallback...")
            self._init_langchain_executor(None, add_prompt, tools, tool_packages)
    
    def _init_langchain_executor(self, llm, add_prompt, tools, tool_packages):
        """기존 LangChain 기반 executor 초기화 (fallback)"""
        self.__chat_history = []  
        self.__memory_key = "chat_history"
        self.__scratchpad = "agent_scratchpad"
        
        self.__llm = llm
        self.__prompts = self._get_prompts(add_prompt)
        self.__toolbox = self._get_tools(packages=tool_packages, tools=tools)
        self.__tools = self.__toolbox.get_tools()
        self.__llm_with_tools = self.__llm.bind_tools(self.__tools)
        self.__agent = self._get_agent()
        self.__executor = self._get_executor(verbose=self.__verbose)
        
        self.use_rkllama = False
        if self.__verbose:
            print("✅ LangChain 기반 ActionExecutor 초기화 완료")
    
    def _build_system_prompt(self, add_prompt):
        """시스템 프롬프트 구성"""
        prompt_parts = []
        
        # 기본 시스템 프롬프트 추가
        for role, content in system_prompts:
            if role == "system":
                prompt_parts.append(content)
        
        # 로봇별 프롬프트 추가
        embodied_prompt = get_prompts()
        prompt_parts.append(str(embodied_prompt))
        
        # 추가 프롬프트
        if add_prompt:
            for role, content in add_prompt:
                if role == "system":
                    prompt_parts.append(content)
        
        return "\n\n".join(prompt_parts)
    
    def _extract_tools_from_package(self, package):
        """패키지에서 도구 함수들 추출"""
        tools = []
        for attr_name in dir(package):
            if not attr_name.startswith("_"):
                attr = getattr(package, attr_name)
                if hasattr(attr, '__call__') and hasattr(attr, 'name'):
                    tools.append(attr)
        return tools
        
    
    def _get_tools(
        self,
        packages: Optional[list],
        tools: Optional[list],
    ) -> ToolBox:
        """Create a ROSA tools object with the specified ROS version, tools, packages, and blacklist."""
        rosa_tools = ToolBox()
        if tools:
            rosa_tools.add_tools(tools)
        if packages:
            rosa_tools.add_packages(packages)
        return rosa_tools
    
    def _get_prompts(self, add_prompt) -> ChatPromptTemplate:
        """Create a chat prompt template from the system prompts and robot-specific prompts."""
        # Start with default system prompts
        prompts = system_prompts # action_executor_prompts
        embodied_prompt = get_prompts()
        
        prompts.append(embodied_prompt.as_message())

        template = ChatPromptTemplate.from_messages(
            prompts
            + 
            add_prompt
            +
            [
                MessagesPlaceholder(variable_name=self.__memory_key),
                ("user", "{state}"),
                MessagesPlaceholder(variable_name=self.__scratchpad),
            ]
        )
        return template
    
    def _get_agent(self):
        """Create and return an agent for processing user inputs and generating responses."""
        # agent = create_tool_calling_agent(self.__llm, self.__tools, self.__prompts)

        agent = (
            {
                "state": lambda x: x["state"],
                "agent_scratchpad": lambda x: format_to_openai_tool_messages(x["intermediate_steps"]),
                "chat_history": lambda x: x.get("chat_history", []),
            }
            | self.__prompts
            | self.__llm_with_tools
            | OpenAIToolsAgentOutputParser()
        )
        
        return agent
    

    def _get_executor(self, verbose: bool) -> AgentExecutor:
        """Create and return an executor for processing user inputs and generating responses."""
        executor = AgentExecutor(
            agent=self.__agent,
            tools=self.__tools,
            verbose=verbose,
            return_intermediate_steps=True,
            handle_parsing_errors=True,
            max_iterations=15,
            # max_execution_time=30,
        )

        return executor
    
    def invoke(self, state: str) -> Union[str, Dict]:
        """통합된 invoke 메서드 - rkllama 또는 LangChain 사용"""
        self.last_answer = None
        
        if self.use_rkllama:
            return self._invoke_rkllama(state)
        else:
            return self._invoke_langchain(state)
    
    def _invoke_rkllama(self, state: str) -> Dict:
        """rkllama_core 기반 실행"""
        try:
            if self.__verbose:
                print(f"🚀 rkllama Agent 실행: {state[:50]}...")
            
            # rkllama agent 호출
            response = self.__agent.invoke(state)
            
            if self.__verbose:
                print(f"✅ rkllama 응답 완료")
            
            return {
                "output": response,
                "agent_scratchpad": []  # rkllama는 내부적으로 tool calling 처리
            }
            
        except Exception as e:
            error_msg = f"rkllama 실행 오류: {e}"
            print(f"❌ {error_msg}")
            return {
                "output": error_msg,
                "agent_scratchpad": []
            }
    
    def _invoke_langchain(self, state: str) -> Dict:
        """기존 LangChain 기반 실행"""
        try:
            result = self.__executor.invoke(
                {"state": state, "chat_history": self.__chat_history}
            )   
            
            output_text = result["output"]
            intermediate_steps = result.get("intermediate_steps", [])
            structured_steps = [
                {
                    "log": getattr(action, "log", ""),
                    "observation": observation
                }
                for action, observation in intermediate_steps
            ]
            
            self._record_chat_history(state, output_text)
            
            return {
                "output": output_text,
                "agent_scratchpad": structured_steps
            }
            
        except Exception as e:
            error_msg = f"LangChain 실행 오류: {e}"
            print(f"❌ {error_msg}")
            return {
                "output": error_msg,
                "agent_scratchpad": []
            }
    
    def chat(self, state: str) -> str:
        """단순 채팅용 메서드"""
        result = self.invoke(state)
        if isinstance(result, dict):
            return result.get("output", "")
        return result
    
    def invoke_iter(self, state: str) -> Dict:
        """반복 실행 메서드 (중단 조건 지원)"""
        if self.use_rkllama:
            # rkllama는 내부적으로 완전한 실행을 수행하므로 일반 invoke 사용
            return self.invoke(state)
        else:
            # LangChain 기반 반복 실행
            return self._invoke_langchain_iter(state)
    
    def _invoke_langchain_iter(self, state: str) -> Dict:
        """LangChain 기반 반복 실행"""
        structured_steps = []
        try:
            for step in self.__executor.iter({"state": state, "chat_history": self.__chat_history}):
                # 중단 조건 체크
                if hasattr(self.__agent_state, 'command_case') and self.__agent_state.command_case == 0:
                    if self.__verbose:
                        print("⚠️ 중간 중단 조건 감지! 즉시 break")
                    return {
                        "output": "새로운 명령이 입력되었습니다. 즉시 행동을 멈춥니다.",
                        "agent_scratchpad": []
                    }
                
                if output := step.get("intermediate_step"):
                    action, value = output[0]
                    if self.__verbose:
                        print(f"🔧 도구 사용: {getattr(action, 'log', 'Unknown tool')}")
                    structured_step = {
                        "log": getattr(action, "log", ""),
                        "observation": value
                    }
                    structured_steps.append(structured_step)

            output_text = step.get("output", "")
            self._record_chat_history(state, output_text)
            
            return {
                "output": output_text,
                "agent_scratchpad": structured_steps
            }
            
        except Exception as e:
            error_msg = f"LangChain iter 실행 오류: {e}"
            print(f"❌ {error_msg}")
            return {
                "output": error_msg,
                "agent_scratchpad": structured_steps
            }

    def _record_chat_history(self, query: str, response: str):
        """대화 기록 저장"""
        if self.__accumulate_chat_history and hasattr(self, '_ActionExecutor__chat_history'):
            try:
                from langchain_core.messages import AIMessage, HumanMessage
                self.__chat_history.extend(
                    [HumanMessage(content=query), AIMessage(content=response)]
                )
            except ImportError:
                # LangChain 미사용시 단순 문자열로 저장
                if not hasattr(self, '_simple_history'):
                    self._simple_history = []
                self._simple_history.append({"user": query, "assistant": response})
    
    def clear_history(self):
        """대화 기록 초기화"""
        if self.use_rkllama:
            if hasattr(self.__agent, 'clear_history'):
                self.__agent.clear_history()
        else:
            if hasattr(self, '_ActionExecutor__chat_history'):
                self.__chat_history = []
        
        if hasattr(self, '_simple_history'):
            self._simple_history = []
    
    def list_tools(self):
        """사용 가능한 도구 목록"""
        if self.use_rkllama:
            if hasattr(self.__agent, 'list_tools'):
                self.__agent.list_tools()
            else:
                print("📋 rkllama agent의 도구 목록을 확인할 수 없습니다.")
        else:
            if hasattr(self, '_ActionExecutor__tools'):
                print("📋 사용 가능한 도구:")
                for tool in self.__tools:
                    print(f"  • {tool.name}: {getattr(tool, 'description', 'No description')}")
    
    

# === 메인 실행부 ===
if __name__ == "__main__":
    

    verbose = True 
    
    chat_ID = str(random.randrange(1,100)) # input("ID를 입력해주세요: ")
    
    model_name = "qwen3:1.7b" # qwen3:4b
    temperature = 0.6
    
    llm = ChatOllama(
                model=model_name, 
                temperature=temperature,
                # base_url = base_url
                )
    
    executor = ActionExecutor(llm=llm, session_id=chat_ID, verbose=verbose)

    
    
    cur_state = {
        'THOUGHT': '주변을 더 꼼꼼히 살펴볼 필요가 있어 보입니다. 다양한 객체와 장소들이 눈에 들어오네요.',
        'EMOTION': '흥미',
        'GOAL': '단순 이동하기 - 주변을 더 꼼꼼히 살펴보기',
        'PLAN': ['x축 방향으로 1m 이동', 'y축 방향으로 1m 이동'],
        'STEP': ['x축 방향으로 1m 이동']
    }
    
    state_input = "\n".join([f"{k}: {v}" for k, v in cur_state.items()])
    
    final_answer = executor.invoke(state_input)
    
    # while True:
    #     # 새 텍스트 입력 전에 이전에 공유했던 카메라 피드 창이 열려있다면 닫습니다.
        
    #     # stream 옵션에 따라 호출
    #     final_answer = executor.invoke(thought, goal, plan, step, thinking)
        
    #     print(final_answer)
