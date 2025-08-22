from langchain.tools import tool
from typing import Any, AsyncIterable, List, Dict, Literal, Optional, Union
from langchain_community.chat_message_histories import ChatMessageHistory
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_core.prompts import ChatPromptTemplate

from langchain.agents import create_tool_calling_agent, AgentExecutor
from langchain_community.callbacks import get_openai_callback  
from langchain_core.messages import AIMessage, HumanMessage  
from langchain.prompts import MessagesPlaceholder
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain.agents.format_scratchpad.openai_tools import (
    format_to_openai_tool_messages,
)
from langchain_core.runnables import RunnablePassthrough
import json
import random
import argparse

from langchain_ollama import ChatOllama

from edie8_agent.components.toolbox import ToolBox
from edie8_agent.components.prompts.agent_prompt import system_prompts, get_prompts

from edie8_agent.state_manager import get_state


class ActionExecutor:
    def __init__(
        self, 
        llm,
        add_prompt,
        tools: Optional[list] = None,
        tool_packages: Optional[list] = None,
        session_id: str = "default", 
        accumulate_chat_history: bool = True,
        verbose: bool = True,
    ):
        self.__chat_history = []  
        self.__memory_key = "chat_history"
        self.__scratchpad = "agent_scratchpad"
        self.__accumulate_chat_history = accumulate_chat_history
        self.__session_id = session_id
        self.__verbose = verbose
        self.last_answer = None                         # 마지막 응답을 저장할 변수
        
        self.__llm = llm
        self.__prompts = self._get_prompts(add_prompt)     # 프롬프트 템플릿 생성
        self.__toolbox = self._get_tools(packages=tool_packages, tools=tools)
        self.__tools = self.__toolbox.get_tools()
        self.__llm_with_tools = self.__llm.bind_tools(self.__tools)     # LLM과 도구 결합
        self.__agent = self._get_agent()
        self.__executor = self._get_executor(verbose=self.__verbose)
        self.__agent_state = get_state()
        
    
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
    
    def chat(self, state: str) -> str:
        self.last_answer = None  # 이전 결과 초기화
        
        response_stream = self.__executor.invoke(
                {"state": state},
                config={"configurable": {"session_id": self.__session_id, "stream": False}},
            )
            
        return response_stream
    
    def invoke(self, state: str) -> str:
        
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
    
    def invoke_iter(self, state: str) -> str:
        structured_steps = []
        for step in self.__executor.iter({"state": state, "chat_history": self.__chat_history}):
            print(f"\ncommand_case:{self.__agent_state.command_case} <----------------------")
            if self.__agent_state.command_case==0:
                print("중간 중단 조건 감지! 즉시 break")
                # break
                return {
                    "output": "새로운 명령이 입력되었습니다. 즉시 행동을 멈춥니다.",
                    "agent_scratchpad": []
                }
            
            if output := step.get("intermediate_step"):
                action, value = output[0]
                print(f"\n다음과 같은 도구를 사용하였습니다:{action.log}")
                structured_step = {
                        "log": action.log,
                        "observation": value
                    }
                structured_steps.append(structured_step)

        output_text = step["output"]
        self._record_chat_history(state, output_text)
        
        return {
            "output": output_text,
            "agent_scratchpad": structured_steps
        }


    def _record_chat_history(self, query: str, response: str):
        """Record the chat history if accumulation is enabled."""
        if self.__accumulate_chat_history:
            self.__chat_history.extend(
                [HumanMessage(content=query), AIMessage(content=response)]
            )
    
    

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
        