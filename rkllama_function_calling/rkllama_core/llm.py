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
import os
import psutil
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
    LangChain-style LLM interface for rkllama with CPU thread control
    
    Usage:
        from rkllama_core import LLM
        
        # CPU 코어 수 자동 감지 사용
        llm = LLM(model="Qwen2.5-3B-Instruct_W8A8_G128_RK3588")
        
        # CPU 코어 수 수동 설정 (물리 코어 수 권장)
        llm = LLM(model="Qwen2.5-3B-Instruct_W8A8_G128_RK3588", num_thread=4)
        
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
        system_prompt: Optional[str] = None,
        num_thread: Optional[int] = 4,
        num_gpu_layers: Optional[int] = None,
        context_length: Optional[int] = None,
        rope_freq_base: Optional[float] = None,
        rope_freq_scale: Optional[float] = None
    ):
        """
        Initialize RKLlama LLM with CPU thread control
        
        Args:
            model: Model name to use
            base_url: RKLlama server URL
            temperature: Sampling temperature (0.0 to 2.0)
            max_tokens: Maximum tokens to generate
            stream: Whether to stream responses
            system_prompt: Default system prompt
            num_thread: Number of threads to use during computation.
                       By default, rkllama will detect this for optimal performance.
                       It is recommended to set this value to the number of physical 
                       CPU cores your system has (as opposed to the logical number of cores).
                       If None, will be auto-detected.
            num_gpu_layers: Number of layers to offload to GPU (if available)
            context_length: Maximum context length
            rope_freq_base: RoPE frequency base
            rope_freq_scale: RoPE frequency scale
        """
        self.model = model
        self.base_url = base_url
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.stream = stream
        self.system_prompt = system_prompt
        
        # CPU 스레드 설정
        self.num_thread = self._determine_num_threads(num_thread)
        
        # 추가 성능 파라미터들
        self.num_gpu_layers = num_gpu_layers
        self.context_length = context_length
        self.rope_freq_base = rope_freq_base
        self.rope_freq_scale = rope_freq_scale
        
        # 모델 설정 캐시 (성능 최적화)
        self._model_config_cache = None
        
    def _determine_num_threads(self, num_thread: Optional[int]) -> int:
        """
        CPU 스레드 수 결정
        
        Args:
            num_thread: 사용자 지정 스레드 수
            
        Returns:
            최종 사용할 스레드 수
        """
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
        
        # 최종 fallback: 환경 변수 또는 기본값
        return int(os.getenv('OMP_NUM_THREADS', '4'))
    
    def get_system_info(self) -> Dict[str, Any]:
        """
        시스템 정보 조회
        
        Returns:
            시스템 정보 딕셔너리
        """
        try:
            return {
                "physical_cores": psutil.cpu_count(logical=False),
                "logical_cores": psutil.cpu_count(logical=True),
                "current_threads": self.num_thread,
                "memory_total_gb": round(psutil.virtual_memory().total / (1024**3), 2),
                "memory_available_gb": round(psutil.virtual_memory().available / (1024**3), 2),
                "cpu_percent": psutil.cpu_percent(interval=1),
                "model": self.model,
                "base_url": self.base_url
            }
        except Exception as e:
            return {"error": f"Failed to get system info: {e}"}
    
    def _build_request_payload(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        stream: bool = False
    ) -> Dict[str, Any]:
        """
        요청 페이로드 구성 (성능 파라미터 포함)
        
        Args:
            messages: 메시지 리스트
            temperature: 온도 설정
            max_tokens: 최대 토큰 수
            stream: 스트리밍 여부
            
        Returns:
            요청 페이로드
        """
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature or self.temperature,
            "stream": stream
        }
        
        # 최대 토큰 수
        if max_tokens or self.max_tokens:
            payload["max_tokens"] = max_tokens or self.max_tokens
        
        # 성능 관련 파라미터들 추가
        performance_params = {}
        
        # CPU 스레드 수
        if self.num_thread:
            performance_params["num_thread"] = self.num_thread
            
        # GPU 레이어 수
        if self.num_gpu_layers is not None:
            performance_params["num_gpu_layers"] = self.num_gpu_layers
            
        # 컨텍스트 길이
        if self.context_length is not None:
            performance_params["context_length"] = self.context_length
            
        # RoPE 파라미터들
        if self.rope_freq_base is not None:
            performance_params["rope_freq_base"] = self.rope_freq_base
            
        if self.rope_freq_scale is not None:
            performance_params["rope_freq_scale"] = self.rope_freq_scale
        
        # 성능 파라미터가 있으면 추가
        if performance_params:
            payload["options"] = performance_params
            
        return payload
    
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
        payload = self._build_request_payload(messages, temperature, max_tokens, stream=False)
            
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
        payload = self._build_request_payload(messages, temperature, max_tokens, stream=True)
            
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
            'system_prompt': self.system_prompt,
            'num_thread': self.num_thread,
            'num_gpu_layers': self.num_gpu_layers,
            'context_length': self.context_length,
            'rope_freq_base': self.rope_freq_base,
            'rope_freq_scale': self.rope_freq_scale
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