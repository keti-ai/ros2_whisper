import os
from gtts import gTTS
import playsound as ps
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from whisper_idl.action import TTS as TTSAction  # .action 파일에서 생성된 메시지
from rclpy.executors import MultiThreadedExecutor

class TTSActionServer(Node):
    def __init__(self):
        super().__init__('tts_action_server')

        # 액션 서버 생성
        self._action_server = ActionServer(
            self,
            TTSAction,
            'tts_action',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: {goal_handle.request.text}')
        text = goal_handle.request.text

        # 피드백 초기화
        feedback_msg = TTSAction.Feedback()

        try:
            # 음성 합성 처리
            tts = gTTS(text=text, lang='ko', slow=False)
            tts.save("tmp_tts.mp3")

            # 피드백 전송 (합성 중)
            feedback_msg.percentage = 50.0
            goal_handle.publish_feedback(feedback_msg)

            # 스피커 출력
            ps.playsound("tmp_tts.mp3")

            # 피드백 완료
            feedback_msg.percentage = 100.0
            goal_handle.publish_feedback(feedback_msg)

            # 성공 결과 반환
            goal_handle.succeed()
            return TTSAction.Result(success=True)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            goal_handle.abort()
            return TTSAction.Result(success=False)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    tts_action_server = TTSActionServer()

    executor.add_node(tts_action_server)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        tts_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
