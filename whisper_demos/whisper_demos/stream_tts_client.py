import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter as RclpyParameter
from whisper_idl.msg import AudioTranscript
from whisper_idl.action import TTS as TTSAction


# 제외할 단어 목록
EXCLUDED_WORDS = {'감사합니다', '고맙습니다', '아멘', '하나님', '.', '네', '이제'}


class STTToTTSClient(Node):

    def __init__(self):
        super().__init__('stt_to_tts_client')

        # TTS action client
        self._action_client = ActionClient(self, TTSAction, 'tts_action')

        # Subscribe to the audio transcript topic
        self.subscription = self.create_subscription(
            AudioTranscript,
            '/whisper/transcript_stream',
            self.transcript_callback,
            10)

        self.subscription  # prevent unused variable warning

        self.previous_index = 0

        # 파라미터 설정을 위한 서비스 클라이언트 생성
        self.parameter_client = self.create_client(SetParameters, '/whisper/inference/set_parameters')

    def transcript_callback(self, msg: AudioTranscript):
        if self.previous_index == 0:
            self.previous_index = msg.seg_start_words_id[-1]

        # 기존 단어 목록과 비교하여 새로 생성된 세그먼트만 처리
        new_segments = []
        for seg_i, seg_start in enumerate(msg.seg_start_words_id):
            # 현재 세그먼트가 이전 인덱스 이후인지 확인
            if seg_start >= self.previous_index:
                new_segments.append(seg_i)

        if not new_segments:
            # self.get_logger().info("No new segments to process.")
            return

        new_sentences = []

        # Threshold 설정
        threshold = 1.0  # 필요에 따라 조정

        # 새로 추가된 세그먼트를 순회하며 문장 생성
        for seg_i in new_segments:
            seg_begin = msg.seg_start_words_id[seg_i]
            if seg_i == len(msg.seg_start_words_id) - 1:
                seg_end = len(msg.words)  # 마지막 세그먼트의 끝
            else:
                seg_end = msg.seg_start_words_id[seg_i + 1]

            # 세그먼트의 단어들 필터링
            segment_words = []
            for i in range(seg_begin, seg_end):
                # likelyhood = msg.probs[i] * msg.occ[i]  # Likelyhood 계산
                # if i >= msg.active_index:  # active_index 이전의 단어는 고정된 텍스트
                #     if likelyhood >= threshold:
                #         word = msg.words[i].strip()
                #         if word not in EXCLUDED_WORDS:
                #             segment_words.append(word)

                likelyhood = msg.probs[i] * msg.occ[i]
                word = msg.words[i].strip()
                if word not in EXCLUDED_WORDS:
                    if i < msg.active_index:
                        if likelyhood >= threshold:
                            segment_words.append(word)
                    # else:
                    #     segment_words.append(word)

            # 문장 생성
            if segment_words:
                new_sentences.append(' '.join(segment_words))

        # 문장을 합쳐 최종 텍스트 생성
        final_text = ''.join(new_sentences)

        if not final_text.strip():  # 처리할 문장이 없으면 리턴
            # self.get_logger().info("No valid new text to process.")
            return

        self.get_logger().info(f"New STT Text: {final_text}")

        self.set_whisper_active(False)

        # TTS 서버로 전송
        self.send_goal_to_tts(final_text)

        # 이전 인덱스 업데이트
        self.previous_index = len(msg.words)

    def send_goal_to_tts(self, text):
        goal_msg = TTSAction.Goal(text=text)
        self.get_logger().info("Waiting for TTS action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("TTS action server not available!")
            return

        self.get_logger().info(f"Sending goal to TTS server: {text}")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("TTS goal rejected")
            return

        self.get_logger().info("TTS goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"TTS Success")
        else:
            self.get_logger().error(f"TTS Failed")

        self.set_whisper_active(active=True)

    def set_whisper_active(self, active: bool):
        """Whisper active 상태를 설정"""
        if not self.parameter_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Parameter service not available!")
            return

        # SetParametersRequest 생성
        param_request = SetParameters.Request()
        param = Parameter()
        param.name = 'active'
        param.value.type = ParameterType.PARAMETER_BOOL  # 수정된 부분
        param.value.bool_value = active
        param_request.parameters = [param]

        # 비동기 서비스 호출
        future = self.parameter_client.call_async(param_request)
        future.add_done_callback(self.parameter_callback)

    def parameter_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Parameter set response: {response}")
        except Exception as e:
            self.get_logger().error(f"Failed to set parameter: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    stt_to_tts_client = STTToTTSClient()

    rclpy.spin(stt_to_tts_client)

    # Destroy the node explicitly
    stt_to_tts_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
