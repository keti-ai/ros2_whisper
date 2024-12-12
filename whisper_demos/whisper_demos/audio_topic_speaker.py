import rclpy
from rclpy.node import Node
from whisper_idl.msg import AudioTranscript  # 메시지 타입
from std_msgs.msg import String  # 출력 토픽

class RecentSTTNode(Node):
    def __init__(self):
        super().__init__('recent_stt_node')
        self.subscription = self.create_subscription(
            AudioTranscript,
            '/whisper/transcript_stream',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(String, '/recent_stt', 10)

    def callback(self, msg):
        # 최근 변환된 텍스트만 필터링
        active_index = msg.active_index
        recent_words = msg.words[active_index:]  # active_index 이후 단어 추출

        if recent_words:  # 최근 단어가 있으면 출력
            recent_text = ' '.join(recent_words)
            self.get_logger().info(f"Recent STT: {recent_text}")
            self.publisher.publish(String(data=recent_text))  # 새로운 토픽으로 발행

def main(args=None):
    rclpy.init(args=args)
    node = RecentSTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
