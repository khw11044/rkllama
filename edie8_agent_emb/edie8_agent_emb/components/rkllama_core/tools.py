# ============================================
# rkllama_core/tools.py
# ============================================

"""
RKLlama Tools Module
Tool decorator and built-in tools for rkllama function calling
"""

import json
import inspect
from typing import Any, Dict, List, Callable, Optional, Union
from dataclasses import dataclass
from datetime import datetime


@dataclass
class ToolMetadata:
    """Tool metadata container"""
    name: str
    description: str
    function: Callable
    parameters: Dict[str, Any]


class RKLlamaTool:
    """RKLlama tool wrapper"""
    
    def __init__(self, func: Callable, name: Optional[str] = None, description: Optional[str] = None):
        self.func = func
        self.name = name or func.__name__
        self.description = description or func.__doc__ or f"Tool: {self.name}"
        self.parameters = self._extract_parameters()
    
    def _extract_parameters(self) -> Dict[str, Any]:
        """Extract function parameters and create JSON schema"""
        sig = inspect.signature(self.func)
        properties = {}
        required = []
        
        for param_name, param in sig.parameters.items():
            param_info = {
                "type": self._python_type_to_json_type(param.annotation),
                "description": f"Parameter {param_name}"
            }
            
            # Get description from docstring if available
            if self.func.__doc__:
                # Simple docstring parsing
                lines = self.func.__doc__.split('\n')
                for line in lines:
                    if param_name in line and ':' in line:
                        param_info["description"] = line.split(':', 1)[1].strip()
            
            properties[param_name] = param_info
            
            # Check if parameter is required (no default value)
            if param.default == inspect.Parameter.empty:
                required.append(param_name)
        
        return {
            "type": "object",
            "properties": properties,
            "required": required
        }
    
    def _python_type_to_json_type(self, python_type) -> str:
        """Convert Python type to JSON schema type"""
        if python_type == inspect.Parameter.empty:
            return "string"
        
        type_mapping = {
            int: "integer",
            float: "number", 
            str: "string",
            bool: "boolean",
            list: "array",
            dict: "object"
        }
        
        return type_mapping.get(python_type, "string")
    
    def __call__(self, *args, **kwargs):
        """Make the tool callable"""
        return self.func(*args, **kwargs)
    
    def to_schema(self) -> Dict[str, Any]:
        """Convert to OpenAI function calling schema"""
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.parameters
            }
        }
    
    def invoke(self, **kwargs) -> Any:
        """Invoke the tool with keyword arguments"""
        return self.func(**kwargs)


def tool(func: Optional[Callable] = None, *, name: Optional[str] = None, description: Optional[str] = None):
    """
    Decorator to create RKLlama tools
    
    Usage:
        @tool
        def add(a: int, b: int) -> int:
            '''Adds two numbers together.
            
            Args:
                a: First number to add
                b: Second number to add
            '''
            return a + b
    
    Or with custom name and description:
        @tool(name="calculator_add", description="Advanced addition function")
        def add(a: int, b: int) -> int:
            return a + b
    """
    def decorator(f: Callable) -> RKLlamaTool:
        return RKLlamaTool(f, name=name, description=description)
    
    if func is None:
        return decorator
    else:
        return decorator(func)