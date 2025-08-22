#!/usr/bin/env python3
"""
rkllama_core 마이그레이션 테스트 스크립트
edie8_agent가 rkllama_core와 제대로 통합되었는지 확인
"""

import sys
import os

# rkllama_function_calling 모듈 패스 추가
sys.path.append('./rkllama_function_calling')

def test_rkllama_import():
    """rkllama_core 임포트 테스트"""
    try:
        from rkllama_core import LLM, RKLlamaAgent, tool
        print("✅ rkllama_core 임포트 성공")
        return True
    except ImportError as e:
        print(f"❌ rkllama_core 임포트 실패: {e}")
        return False

def test_adapter_import():
    """rkllama_adapter 임포트 테스트"""
    try:
        from edie8_agent.components.utils.rkllama_adapter import RKLlamaAdapter, get_rkllama_agent
        print("✅ rkllama_adapter 임포트 성공")
        return True
    except ImportError as e:
        print(f"❌ rkllama_adapter 임포트 실패: {e}")
        return False

def test_tools_import():
    """마이그레이션된 도구들 임포트 테스트"""
    try:
        from edie8_agent.components.toolbox import calculation as calc
        from edie8_agent.components.toolbox import move as move
        from edie8_agent.components.toolbox import action as action
        
        # 도구들이 rkllama @tool 데코레이터를 사용하는지 확인
        print("📋 계산 도구들:")
        calc_tools = [attr for attr in dir(calc) if not attr.startswith('_') and hasattr(getattr(calc, attr), 'name')]
        for tool_name in calc_tools[:3]:  # 처음 3개만 표시
            print(f"  • {tool_name}")
        
        print("📋 움직임 도구들:")
        move_tools = [attr for attr in dir(move) if not attr.startswith('_') and hasattr(getattr(move, attr), 'name')]
        for tool_name in move_tools[:3]:
            print(f"  • {tool_name}")
            
        print("📋 액션 도구들:")  
        action_tools = [attr for attr in dir(action) if not attr.startswith('_') and hasattr(getattr(action, attr), 'name')]
        for tool_name in action_tools[:3]:
            print(f"  • {tool_name}")
            
        print("✅ 모든 도구 임포트 성공")
        return True
    except ImportError as e:
        print(f"❌ 도구 임포트 실패: {e}")
        return False

def test_executor_init():
    """ActionExecutor 초기화 테스트"""
    try:
        from edie8_agent.components.models.executor import ActionExecutor
        from edie8_agent.components.prompts.prompt import command_executor_prompts
        from edie8_agent.components.toolbox import move as move_tool
        from edie8_agent.components.toolbox import action as action_tool
        from edie8_agent.components.toolbox import calculation as calculation_tool
        
        print("🔄 ActionExecutor 초기화 테스트...")
        
        # rkllama 모드로 초기화 시도
        executor = ActionExecutor(
            add_prompt=command_executor_prompts,
            tool_packages=[action_tool, move_tool, calculation_tool], 
            verbose=True,
            session_id="test",
            use_rkllama=True
        )
        
        print("✅ ActionExecutor 초기화 성공")
        
        # 도구 목록 표시 테스트
        print("\n📋 바인딩된 도구 목록:")
        executor.list_tools()
        
        return True
        
    except Exception as e:
        print(f"❌ ActionExecutor 초기화 실패: {e}")
        print(f"상세 오류: {type(e).__name__}: {e}")
        return False

def test_command_agent():
    """CommandAgent 테스트"""
    try:
        from edie8_agent.command import CommandAgent
        
        print("🔄 CommandAgent 초기화 테스트...")
        
        # Mock robot_node (실제 ROS2 없이 테스트)
        class MockRobotNode:
            pass
        
        mock_robot = MockRobotNode()
        
        agent = CommandAgent(
            robot_node=mock_robot,
            local=False,
            verbose=True,
            temperature=0.6,
            use_rkllama=True
        )
        
        print("✅ CommandAgent 초기화 성공")
        return True
        
    except Exception as e:
        print(f"❌ CommandAgent 초기화 실패: {e}")
        print(f"상세 오류: {type(e).__name__}: {e}")
        return False

def main():
    """메인 테스트 실행"""
    print("🚀 rkllama_core 마이그레이션 테스트 시작")
    print("=" * 50)
    
    tests = [
        ("rkllama_core 임포트", test_rkllama_import),
        ("rkllama_adapter 임포트", test_adapter_import),
        ("도구 임포트", test_tools_import),
        ("ActionExecutor 초기화", test_executor_init),
        ("CommandAgent 초기화", test_command_agent),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🧪 {test_name} 테스트 중...")
        try:
            if test_func():
                passed += 1
            print(f"✅ {test_name} 완료")
        except Exception as e:
            print(f"❌ {test_name} 예외 발생: {e}")
        print("-" * 30)
    
    print(f"\n📊 테스트 결과: {passed}/{total} 성공")
    
    if passed == total:
        print("🎉 모든 테스트 통과! rkllama_core 마이그레이션이 성공적으로 완료되었습니다.")
        print("\n💡 다음 단계:")
        print("1. rkllm 서버가 http://localhost:8080에서 실행 중인지 확인")
        print("2. ROS2 환경에서 실제 테스트 진행")
        print("3. 로봇과의 실제 상호작용 테스트")
        return True
    else:
        print("⚠️ 일부 테스트가 실패했습니다. 로그를 확인해주세요.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
