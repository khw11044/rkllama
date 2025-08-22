# input.py (토픽 퍼블리셔 역할만 담당)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InputPublisherNode(Node):
    def __init__(self):
        super().__init__('llm_input_publisher')
        self.llm_input_pub = self.create_publisher(String, "/edie8/llm/input", 10)
        self.get_logger().info("📝 LLM Input Publisher Node is ready!")
        self.listen_input()

    def listen_input(self):
        """터미널에서 사용자 입력을 받아 /edie8/llm/input 토픽으로 발행"""
        while rclpy.ok():
            try:
                user_input = input("\n🧑 사용자 입력: ")
                if user_input.strip():
                    msg = String()
                    msg.data = user_input.strip()
                    self.llm_input_pub.publish(msg)
                    self.get_logger().info(f"입력 발행: {user_input.strip()}")
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"❌ 입력 처리 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InputPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
