
```
rkllama_function_calling/
├── setup.py                  # 패키지 설정 (legacy)
├── pyproject.toml            # 현대적 패키지 설정
├── README.md                 # 프로젝트 문서
├── LICENSE                   # MIT 라이센스
├── requirements.txt          # 기본 의존성
├── requirements-dev.txt      # 개발 의존성
├── MANIFEST.in              # 패키지에 포함할 파일들
├── .gitignore               # Git 무시 파일들
├── main.py                  # 데모/예제 스크립트
├── toolbox.py               # 내장 도구들
└── rkllama_core/            # 핵심 라이브러리
    ├── __init__.py
    ├── tools.py
    └── RKllamaAgent.py
```


# 🚀 설치 및 사용법

## 1. 개발 모드로 설치

```
# 개발 모드로 설치 (수정사항이 바로 반영됨)
pip install -e .

# 또는 개발 의존성까지 설치
pip install -e ".[dev]"

```

## 2. 패키지 빌드

```
# 배포용 패키지 빌드
python setup.py sdist bdist_wheel

# 또는 modern way

pip install build

python -m build

```

## 3. 설치된 패키지 사용

```python
# 이제 어디서든 import 가능!
from rkllama_core import tool, RKLlamaAgent

@tool
def my_function():
    return "Hello from installed package!"

agent = RKLlamaAgent()
agent.bind_tools([my_function])
```


## 4. 명령줄에서 데모 실행
```bash
# 설치 후 어디서든 실행 가능
rkllama-demo
```



# ============================================
# README.md
# ============================================

# RKLlama Core Tools

LangChain-style tool decorator for rkllama function calling on Rockchip NPU devices.

## Features

- 🔧 **Easy Tool Creation**: Use `@tool` decorator just like LangChain
- 🚀 **Rockchip Optimized**: Built specifically for rkllama and Rockchip NPU
- 🎯 **Function Calling**: Full support for OpenAI-compatible function calling
- 📦 **Built-in Tools**: Ready-to-use tools for common tasks
- 🔄 **Conversation History**: Automatic multi-turn conversation management
- 🎨 **Extensible**: Easy to add custom tools

## Installation

### From Source (Development)

```bash
git clone <your-repo-url>
cd rkllama_function_calling
pip install -e .
```

### From PyPI (Future)

```bash
pip install rkllama-core
```

## Quick Start

### 1. Define Tools

```python
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
def get_weather(city: str) -> str:
    """Get weather for a city.
    
    Args:
        city: Name of the city
    """
    # Your weather API logic here
    return f"Weather in {city}: 22°C, sunny"
```

### 2. Create Agent and Bind Tools

```python
from rkllama_core import RKLlamaAgent

agent = RKLlamaAgent(base_url="http://localhost:8080")
agent.bind_tools([add, get_weather])
```

### 3. Chat with Function Calling

```python
response = agent.invoke("What's 5 + 3? Also, what's the weather in Seoul?")
print(response)
```

## Built-in Tools

The package comes with several built-in tools:

- `add(a, b)` - Add two numbers
- `multiply(a, b)` - Multiply two numbers  
- `calculate(expression)` - Safe math evaluation
- `get_current_time(timezone)` - Get current time

## Examples

### Basic Usage

```python
from rkllama_core import RKLlamaAgent
from toolbox import add, multiply, get_current_time

agent = RKLlamaAgent()
agent.bind_tools([add, multiply, get_current_time])

# Single question
response = agent.invoke("What's 25 * 4 + 10?")

# Interactive mode
while True:
    user_input = input("You: ")
    if user_input.lower() == 'exit':
        break
    print("AI:", agent.invoke(user_input))
```

### Custom Tools

```python
@tool
def search_database(query: str, limit: int = 10) -> str:
    """Search company database.
    
    Args:
        query: Search query
        limit: Max results
    """
    # Your database search logic
    return f"Found {limit} results for '{query}'"

@tool  
def send_email(to: str, subject: str, body: str) -> str:
    """Send an email.
    
    Args:
        to: Recipient email
        subject: Email subject  
        body: Email content
    """
    # Your email sending logic
    return f"Email sent to {to}"
```

## Requirements

- Python 3.8+
- rkllama server running on Rockchip NPU device
- Function calling capable model loaded in rkllama

## Configuration

### Agent Configuration

```python
agent = RKLlamaAgent(
    base_url="http://localhost:8080",  # rkllama server URL
    model="your-model-name"            # Model name
)
```

### Tool Schema

Tools automatically generate OpenAI-compatible schemas from:
- Function signatures (parameter types)
- Docstrings (descriptions)
- Type hints (JSON schema types)

## API Reference

### @tool Decorator

```python
@tool(name="custom_name", description="Custom description")
def my_function(param: type) -> return_type:
    """Function description.
    
    Args:
        param: Parameter description
    """
    return result
```

### RKLlamaAgent Class

- `bind_tools(tools)` - Bind tools to agent
- `invoke(message, system_prompt=None)` - Send message and get response
- `clear_history()` - Clear conversation history
- `list_tools()` - List available tools

## Development

### Install Development Dependencies

```bash
pip install -e ".[dev]"
```

### Run Tests

```bash
pytest
```

### Code Formatting

```bash
black .
flake8 .
```

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by [LangChain](https://github.com/langchain-ai/langchain)
- Built for [rkllama](https://github.com/NotPunchnox/rkllama)
- Optimized for Rockchip NPU devices