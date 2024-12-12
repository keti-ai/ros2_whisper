import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import numpy as np

class AudioAnalyzer(Node):
    def __init__(self):
        super().__init__('audio_analyzer')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/audio_listener/audio',
            self.audio_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )

    def audio_callback(self, msg):
        self.get_logger().info(f"Received audio data of length: {len(msg.data)}")

def main(args=None):
    rclpy.init(args=args)
    node = AudioAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
