# ============================================
# main.py (수정된 버전)
# ============================================

from rkllama_core import RKLlamaAgent
from toolbox import add, multiply, get_current_time, calculate, weather, search_web

if __name__ == "__main__":
    # Agent 생성 및 도구 바인딩
    agent = RKLlamaAgent()
    tools = [add, multiply, get_current_time, calculate, weather, search_web]
    agent.bind_tools(tools)
    
    print("🚀 RKLlama Core Tools Demo")
    print("Available tools:", [tool.name for tool in agent.tools])
    
    # 단일 질문 테스트
    # response = agent.invoke("What's 25 times 4 plus 10?")
    # print("🤖 Response:", response)
    
    reply = agent.invoke("What's 25 times 4 plus 10?")
    print("🤖 Response:", reply if isinstance(reply, str) else str(reply))

    # 인터랙티브 모드
    print("\n💬 Interactive mode (type 'exit' to quit):")
    while True:
        try:
            user_input = input("\n🧑 You: ").strip()
            
            if user_input.lower() in ["exit", "quit"]:
                break
            
            if user_input == "!tools":
                agent.list_tools()
                continue
            
            if user_input == "!clear":
                agent.clear_history()
                print("🗑️ 대화 기록이 삭제되었습니다.")
                continue
            
            if not user_input:
                continue
            
            print("🤖 RKLLAMA:", agent.invoke(user_input))
            
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
