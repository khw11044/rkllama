
# ============================================
# toolbox.py (수정된 버전)
# ============================================

import json
from datetime import datetime
from rkllama_core import tool

@tool
def add(a: int, b: int) -> int:
    """Adds two numbers together.
    
    Args:
        a: First number to add
        b: Second number to add
    """
    return a + b


@tool
def multiply(a: int, b: int) -> int:
    """Multiplies two numbers together.
    
    Args:
        a: First number to multiply
        b: Second number to multiply
    """
    return a * b


@tool
def get_current_time(timezone: str = "Asia/Seoul") -> str:
    """Get the current time in specified timezone.
    
    Args:
        timezone: Timezone string (default: Asia/Seoul)
    """
    current_time = datetime.now()
    return json.dumps({
        "timezone": timezone,
        "current_time": current_time.strftime("%Y-%m-%d %H:%M:%S"),
        "day_of_week": current_time.strftime("%A"),
        "timestamp": current_time.timestamp()
    }, ensure_ascii=False)


@tool
def calculate(expression: str) -> str:
    """Evaluate a mathematical expression safely.
    
    Args:
        expression: Mathematical expression to evaluate (e.g., "2+2*3")
    """
    try:
        # Safe evaluation - only allow basic math
        allowed_chars = set('0123456789+-*/().')
        if not all(c in allowed_chars or c.isspace() for c in expression):
            return json.dumps({"error": "Invalid characters in expression"}, ensure_ascii=False)
        
        result = eval(expression)
        return json.dumps({
            "expression": expression,
            "result": result
        }, ensure_ascii=False)
    except Exception as e:
        return json.dumps({"error": f"Calculation error: {str(e)}"}, ensure_ascii=False)


@tool
def weather(city: str) -> str:
    """Get weather information for a city.
    
    Args:
        city: Name of the city
    """
    import random
    conditions = ["sunny", "cloudy", "rainy", "snowy"]
    temp = random.randint(-10, 35)
    condition = random.choice(conditions)
    
    return f"Weather in {city}: {temp}°C, {condition}"


@tool
def search_web(query: str) -> str:
    """Search the web for information.
    
    Args:
        query: Search query
    """
    # 시뮬레이션된 웹 검색
    return f"Search results for '{query}': Found 3 relevant articles about {query}."