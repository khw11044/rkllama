from rkllama_core import LLM

# # LLM 인스턴스 생성
llm = LLM(model="Qwen2.5-3B-Instruct_W8A8_G128_RK3588", num_thread=2)


# 간단한 질문
response = llm.invoke("지구의 자전 주기는?")
print(f"🤖 응답: {response}")

# 직접 호출 (callable)
response2 = llm("파이썬의 장점 3가지만 말해줘")
print(f"🤖 응답: {response2}")

# --------------------------------------------------------------

# system_prompt = "당신은 친근한 AI 어시스턴트입니다. 항상 존댓말을 사용하세요."

# llm = LLM(
#         model="Qwen2.5-3B-Instruct_W8A8_G128_RK3588",
#         system_prompt=system_prompt
#     )
    
# response = llm.invoke("안녕하세요!")
# print(f"🤖 응답: {response}")


# --------------------------------------------------------------

# from rkllama_core import LLM, tool, RKLlamaAgent

# # 도구 사용하는 예시
# @tool
# def get_weather(city: str) -> str:
#     """날씨 정보를 가져옵니다."""
#     import random
#     temps = random.randint(-10, 35)
#     conditions = random.choice(["맑음", "흐림", "비", "눈"])
#     return f"{city}의 날씨: {temps}°C, {conditions}"

# agent = RKLlamaAgent()
# agent.bind_tools([get_weather])
# response = agent.invoke("서울 날씨가 어때?")
# print(f"🤖 Agent + Tools: {response}")