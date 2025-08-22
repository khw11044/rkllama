# ============================================
# rkllama_core/llm.py (새 파일)
# ============================================

"""
RKLlama LLM Module
LangChain-style LLM interface for rkllama
"""

import json
import requests
import time
from typing import Any, Dict, List, Optional, Union
from dataclasses import dataclass


@dataclass
class LLMResponse:
    """LLM 응답을 담는 데이터 클래스"""
    content: str
    model: str
    usage: Optional[Dict[str, Any]] = None
    response_time: Optional[float] = None


class LLM:
    """
    LangChain-style LLM interface for rkllama
    
    Usage:
        from rkllama_core import LLM
        
        llm = LLM(model="Qwen2.5-3B-Instruct_W8A8_G128_RK3588")
        response = llm.invoke("지구의 자전 주기는?")
        print(response)
    """
    
    def __init__(
        self, 
        model: str = "Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
        base_url: str = "http://localhost:8080",
        temperature: float = 0.7,
        max_tokens: Optional[int] = None,
        stream: bool = False,
        system_prompt: Optional[str] = None
    ):
        """
        Initialize RKLlama LLM
        
        Args:
            model: Model name to use
            base_url: RKLlama server URL
            temperature: Sampling temperature (0.0 to 2.0)
            max_tokens: Maximum tokens to generate
            stream: Whether to stream responses
            system_prompt: Default system prompt
        """
        self.model = model
        self.base_url = base_url
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.stream = stream
        self.system_prompt = system_prompt
        
    def invoke(
        self, 
        message: str, 
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None
    ) -> str:
        """
        Invoke the LLM with a message
        
        Args:
            message: User message
            system_prompt: Optional system prompt (overrides default)
            temperature: Optional temperature (overrides default)
            max_tokens: Optional max tokens (overrides default)
            
        Returns:
            Response content as string
        """
        start_time = time.time()
        
        # 메시지 구성
        messages = []
        
        # 시스템 프롬프트 추가
        final_system_prompt = system_prompt or self.system_prompt
        if final_system_prompt:
            messages.append({"role": "system", "content": final_system_prompt})
            
        # 사용자 메시지 추가
        messages.append({"role": "user", "content": message})
        
        # 요청 페이로드 구성
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature or self.temperature,
            "stream": False  # invoke는 항상 non-streaming
        }
        
        if max_tokens or self.max_tokens:
            payload["max_tokens"] = max_tokens or self.max_tokens
            
        # API 호출
        url = f"{self.base_url}/v1/chat/completions"
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(url, headers=headers, json=payload)
            
            if response.status_code != 200:
                raise RuntimeError(f"API Error {response.status_code}: {response.text}")
                
            data = response.json()
            
            # 응답 파싱
            if "choices" in data and data["choices"]:
                content = data["choices"][0]["message"]["content"]
                
                # 응답 시간 계산
                response_time = time.time() - start_time
                
                return content
            else:
                raise RuntimeError("Invalid response format")
                
        except requests.exceptions.RequestException as e:
            raise RuntimeError(f"Network error: {e}")
        except json.JSONDecodeError as e:
            raise RuntimeError(f"JSON parsing error: {e}")
    
    def stream_invoke(
        self,
        message: str,
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None
    ):
        """
        Stream invoke the LLM with a message
        
        Args:
            message: User message
            system_prompt: Optional system prompt
            temperature: Optional temperature
            max_tokens: Optional max tokens
            
        Yields:
            Response chunks as strings
        """
        # 메시지 구성 (invoke와 동일)
        messages = []
        
        final_system_prompt = system_prompt or self.system_prompt
        if final_system_prompt:
            messages.append({"role": "system", "content": final_system_prompt})
            
        messages.append({"role": "user", "content": message})
        
        # 요청 페이로드 구성
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature or self.temperature,
            "stream": True
        }
        
        if max_tokens or self.max_tokens:
            payload["max_tokens"] = max_tokens or self.max_tokens
            
        # API 호출 (스트리밍)
        url = f"{self.base_url}/v1/chat/completions"
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(url, headers=headers, json=payload, stream=True)
            
            if response.status_code != 200:
                raise RuntimeError(f"API Error {response.status_code}: {response.text}")
            
            # 스트리밍 응답 처리
            for line in response.iter_lines():
                if line:
                    line_text = line.decode('utf-8')
                    if line_text.startswith('data: '):
                        data_text = line_text[6:]  # 'data: ' 제거
                        if data_text.strip() == '[DONE]':
                            break
                        try:
                            data = json.loads(data_text)
                            if "choices" in data and data["choices"]:
                                delta = data["choices"][0].get("delta", {})
                                if "content" in delta:
                                    yield delta["content"]
                        except json.JSONDecodeError:
                            continue
                            
        except requests.exceptions.RequestException as e:
            raise RuntimeError(f"Network error: {e}")
    
    def __call__(self, message: str, **kwargs) -> str:
        """
        Make the LLM callable
        
        Args:
            message: User message
            **kwargs: Additional arguments for invoke
            
        Returns:
            Response content as string
        """
        return self.invoke(message, **kwargs)
    
    def with_config(self, **config) -> 'LLM':
        """
        Create a new LLM instance with updated configuration
        
        Args:
            **config: Configuration updates
            
        Returns:
            New LLM instance
        """
        new_config = {
            'model': self.model,
            'base_url': self.base_url,
            'temperature': self.temperature,
            'max_tokens': self.max_tokens,
            'stream': self.stream,
            'system_prompt': self.system_prompt
        }
        new_config.update(config)
        return LLM(**new_config)
        
    def bind(self, **kwargs) -> 'LLM':
        """
        Bind parameters to create a new LLM instance (LangChain compatibility)
        
        Args:
            **kwargs: Parameters to bind
            
        Returns:
            New LLM instance with bound parameters
        """
        return self.with_config(**kwargs)