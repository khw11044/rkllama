# ============================================
# rkllama_core/RKllamaAgent.py
# ============================================

"""
RKLlama Agent Module with CPU thread control
Main agent class for handling tool-enabled conversations
"""

import json
import requests
import time
import psutil
from typing import Any, Dict, List, Optional, Union

from rkllama_core.tools import RKLlamaTool


class RKLlamaAgent:
    """RKLlama Agent with tool support and performance optimization"""
    
    def __init__(
        self, 
        base_url: str = "http://localhost:8080", 
        model: str = "Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
        num_thread: Optional[int] = None,
        num_gpu_layers: Optional[int] = None,
        context_length: Optional[int] = None,
        system_prompt=None
    ):
        """
        Initialize RKLlama Agent with performance optimization
        
        Args:
            base_url: RKLlama server URL
            model: Model name
            num_thread: Number of threads for CPU computation (auto-detect if None)
            num_gpu_layers: Number of layers to offload to GPU
            context_length: Maximum context length
        """
        self.base_url = base_url
        self.model = model
        self.tools: List[RKLlamaTool] = []
        self.conversation_history: List[Dict[str, Any]] = []
        
        # 성능 파라미터들
        self.num_thread = self._determine_num_threads(num_thread)
        self.num_gpu_layers = num_gpu_layers
        self.context_length = context_length
    
    def _determine_num_threads(self, num_thread: Optional[int]) -> int:
        """CPU 스레드 수 결정 (LLM 클래스와 동일한 로직)"""
        if num_thread is not None:
            if num_thread <= 0:
                raise ValueError("num_thread must be positive")
            return num_thread
        
        # 자동 감지: 물리 코어 수 사용 (권장)
        try:
            physical_cores = psutil.cpu_count(logical=False)
            if physical_cores:
                return physical_cores
        except:
            pass
        
        # fallback: 논리 코어 수
        try:
            logical_cores = psutil.cpu_count(logical=True)
            if logical_cores:
                return logical_cores
        except:
            pass
        
        # 최종 fallback
        return 4
    
    def bind_tools(self, tools: List[Union[RKLlamaTool, callable]]) -> 'RKLlamaAgent':
        """Bind tools to the agent"""
        self.tools = []
        for tool_item in tools:
            if isinstance(tool_item, RKLlamaTool):
                self.tools.append(tool_item)
            elif callable(tool_item):
                # Auto-wrap callable as tool
                self.tools.append(RKLlamaTool(tool_item))
            else:
                raise ValueError(f"Invalid tool type: {type(tool_item)}")
        return self
    
    def get_tool_schemas(self) -> List[Dict[str, Any]]:
        """Get all tool schemas for API call"""
        return [tool.to_schema() for tool in self.tools]
    
    def get_tool_by_name(self, name: str) -> Optional[RKLlamaTool]:
        """Get tool by name"""
        for tool in self.tools:
            if tool.name == name:
                return tool
        return None
    
    def get_performance_info(self) -> Dict[str, Any]:
        """성능 관련 정보 조회"""
        try:
            return {
                "physical_cores": psutil.cpu_count(logical=False),
                "logical_cores": psutil.cpu_count(logical=True),
                "configured_threads": self.num_thread,
                "configured_gpu_layers": self.num_gpu_layers,
                "configured_context_length": self.context_length,
                "memory_total_gb": round(psutil.virtual_memory().total / (1024**3), 2),
                "memory_available_gb": round(psutil.virtual_memory().available / (1024**3), 2),
                "cpu_percent": psutil.cpu_percent(interval=1),
                "model": self.model,
                "base_url": self.base_url,
                "num_tools": len(self.tools)
            }
        except Exception as e:
            return {"error": f"Failed to get performance info: {e}"}
    
    def invoke(self, message: str, system_prompt: Optional[str] = None) -> str:
        """
        Enhanced invoke method with performance optimization
        """
        import json, time, requests
        start_time = time.time()

        # ----- 메시지 구성 -----
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        elif not self.conversation_history:
            messages.append({"role": "system",
                            "content": "You are Edie, a helpful AI assistant. Use the available tools when needed to answer questions accurately."})
        messages.extend(self.conversation_history)
        messages.append({"role": "user", "content": message})

        # 페이로드 구성 (성능 파라미터 포함)
        payload = {
            "model": self.model,
            "messages": messages
        }
        
        # 도구 스키마 추가
        if self.tools:
            payload["tools"] = self.get_tool_schemas()
        
        # 성능 파라미터 추가
        options = {}
        if self.num_thread:
            options["num_thread"] = self.num_thread
        if self.num_gpu_layers is not None:
            options["num_gpu_layers"] = self.num_gpu_layers
        if self.context_length is not None:
            options["context_length"] = self.context_length
            
        if options:
            payload["options"] = options

        url = f"{self.base_url}/v1/chat/completions"
        headers = {"Content-Type": "application/json"}

        # ===== 1차 요청 =====
        rsp = requests.post(url, headers=headers, json=payload)
        if rsp.status_code != 200:
            raise RuntimeError(f"API Error {rsp.status_code}: {rsp.text}")

        data = rsp.json()
        assistant_msg = data["choices"][0]["message"]

        # 툴콜 처리
        if assistant_msg.get("tool_calls"):
            print("🔧 도구 호출을 감지했습니다...")
            messages.append(assistant_msg)

            for tc in assistant_msg["tool_calls"]:
                tool_name = tc["function"]["name"]
                args_raw = tc["function"].get("arguments", {})
                try:
                    args = json.loads(args_raw) if isinstance(args_raw, str) else (args_raw or {})
                except json.JSONDecodeError:
                    args = {}

                print(f"   📞 {tool_name}({args})")
                tool = self.get_tool_by_name(tool_name)
                if tool:
                    try:
                        result = tool.invoke(**args)
                        if not isinstance(result, str):
                            result = json.dumps(result, ensure_ascii=False)
                    except Exception as e:
                        result = json.dumps({"error": f"Tool execution error: {e}"}, ensure_ascii=False)
                else:
                    result = json.dumps({"error": f"Tool '{tool_name}' not found"}, ensure_ascii=False)

                tool_msg = {"role": "tool", "name": tool_name, "content": result}
                if "id" in tc:
                    tool_msg["tool_call_id"] = tc["id"]
                messages.append(tool_msg)

            # ===== 2차 요청 (툴 결과 포함) =====
            payload["messages"] = messages
            rsp2 = requests.post(url, headers=headers, json=payload)
            if rsp2.status_code != 200:
                raise RuntimeError(f"API Error (tool followup) {rsp2.status_code}: {rsp2.text}")

            data2 = rsp2.json()
            final_reply = data2["choices"][0]["message"].get("content", "")
        else:
            # 툴콜 없음: 바로 본문 사용
            final_reply = assistant_msg.get("content", "")

        # 대화 기록 갱신
        self.conversation_history.append({"role": "user", "content": message})
        self.conversation_history.append({"role": "assistant", "content": final_reply})
        if len(self.conversation_history) > 20:
            self.conversation_history = self.conversation_history[-20:]

        print(f"⏱️ 응답 시간: {time.time() - start_time:.2f}초")

        # 항상 문자열만 리턴 보장
        if final_reply is None:
            final_reply = ""
        assert isinstance(final_reply, str), f"invoke() must return str, got {type(final_reply)}"
        return final_reply
    
    def clear_history(self):
        """Clear conversation history"""
        self.conversation_history = []
    
    def list_tools(self):
        """List all available tools"""
        print("📋 사용 가능한 도구:")
        for tool in self.tools:
            print(f"  • {tool.name}: {tool.description}")

