"""
툴박스 함수 자동 수집 유틸리티
"""
import inspect
from typing import List, Any
from edie8_agent_emb.components.rkllama_core.tools import RKLlamaTool


def collect_tools_from_modules(modules: List[Any]) -> List[RKLlamaTool]:
    """
    주어진 모듈들에서 @tool 데코레이터가 적용된 함수들을 자동으로 수집
    
    Args:
        modules: 툴 함수들이 포함된 모듈 리스트
        
    Returns:
        RKLlamaTool 객체들의 리스트
    """
    tools = []
    
    for module in modules:
        # 모듈의 모든 멤버를 검사
        for name, obj in inspect.getmembers(module):
            # RKLlamaTool 객체인 경우 (이미 @tool 데코레이터가 적용된 경우)
            if isinstance(obj, RKLlamaTool):
                tools.append(obj)
            # 함수이고 callable인 경우도 포함 (일반 함수를 툴로 변환)
            elif (inspect.isfunction(obj) and 
                  callable(obj) and 
                  not name.startswith('_')):  # private 함수는 제외
                
                tools.append(obj)  # 모든 public 함수 포함
                    
    return tools


def collect_tools_from_functions(functions: List[callable]) -> List[RKLlamaTool]:
    """
    주어진 함수들을 RKLlamaTool로 변환
    
    Args:
        functions: 툴 함수들의 리스트
        
    Returns:
        RKLlamaTool 객체들의 리스트
    """
    tools = []
    
    for func in functions:
        if isinstance(func, RKLlamaTool):
            tools.append(func)
        elif callable(func):
            # 일반 함수인 경우 RKLlamaTool로 래핑하지 않고 그대로 전달
            # (RKLlamaAgent.bind_tools에서 자동으로 처리함)
            tools.append(func)
            
    return tools
