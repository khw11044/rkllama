
import json
import ast
import re

def custom_json_parser(response):
    if isinstance(response, str):
        raw = response.strip()
    else:
        raw = getattr(response, "content", "").strip()

    raw = re.sub(r"<think>.*?</think>", "", raw, flags=re.DOTALL)
    match = re.search(r"```json\s*(\{.*?\})\s*```", raw, flags=re.DOTALL)
    if not match:
        match = re.search(r"(\{[\s\S]*?\})", raw)
    if not match:
        print("⚠️ JSON 블록을 찾을 수 없음\n", raw)
        return raw

    json_string = match.group(1).strip()

    try:
        return json.loads(json_string)
    except json.JSONDecodeError:
        try:
            return ast.literal_eval(json_string)
        except Exception as e:
            print("⚠️ JSON/Python 파싱 실패:", e)
            print("🔹 원본:\n", json_string)
            return json_string
