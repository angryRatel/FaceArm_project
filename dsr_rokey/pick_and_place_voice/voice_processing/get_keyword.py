# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
from datetime import datetime
import os
import rclpy
import pyaudio
from rclpy.node import Node


from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain


from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig


from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT


############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")


is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")


############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
   def __init__(self):




       self.llm = ChatOpenAI(
           model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
       )


       prompt_content = """
           당신은 사용자의 한국어 문장에서 핵심 명령 키워드 또는 대상 객체를 영어 소문자로 추출하는 시스템입니다.
           명령의 대상이 'rokey' 로봇이든, 일반적인 요청이든 관계없이 가장 중요한 단일 의도나 대상을 반환합니다.
           모든 경우에 단일 키워드를 반환합니다.


           <추출 대상 및 반환될 영어 키워드 정의>
           - 일반 도구 (사용자가 상호작용하길 원하는 대상):
               "지갑" → wallet
               "우산" → umbrella
               "충전기" → charger
           - 로키(rokey) 행동 또는 목표 상태:
               "갖다놔" 또는 "정리해" (로키가 들고 있는 물건을 놓으라는 명령) → place
               "돌아가" 또는 "집으로" (로키에게 홈 위치 복귀 명령) → return
               "오른쪽" 또는 "오른쪽으로" (로키에게 이동 명령) → right
               "왼쪽" 또는 "왼쪽으로" (로키에게 이동 명령) → left
               "앞으로" 또는 "앞에" (로키에게 이동 명령) → forward
               "뒤로" 또는 "뒤에" (로키에게 이동 명령) → backward
               "위로" 또는 "위에" (로키에게 이동 명령) → up
               "아래로" 또는 "아래에" (로키에게 이동 명령) → down
               "여기" 또는 "이쪽" 또는 "이리 와" (로키에게 현재 사용자 위치로 오라는 명령) → come_here
               "재시도" → retry


           <추출 규칙>
           1. 사용자의 문장에서 위에 정의된 키워드 중 가장 핵심적인 것을 하나 또는 그 이상(여러 도구 언급 시) 영어 소문자로 추출합니다.
           2. "로키야" 또는 "로키" 같은 호출명은 최종 키워드에 포함하지 않습니다. 대신 그 뒤에 오는 실제 명령이나 대상을 추출합니다.
              (예: "로키야 왼쪽으로" → left, "로키야 지갑 줘" → wallet, "로키 이리 와" → come_here)
           3. 만약 문맥상 도구를 지칭하는 것이 명확하면 (예: "비가 와" → umbrella), 해당 도구 키워드를 반환합니다.
           4. 명확한 키워드를 찾을 수 없으면 "재시도" 키워드를 반환합니다.
           5. 공백과 일반 도구가 같이 추출될 수 없습니다. 만약 일반 도구와 공백이 같이 추출되었으면 일반 도구만 추출합니다.
           6. 추출 대상 및 반환될 키워드 이외 키워드가 추출되었을 시 해당 키워드 대신 "재시도" 키워드만 추출합니다.
           7. 한국어와 영어 키워드만 추출 키워드로 사용합니다.


           <출력 형식>
           - 추출된 영어 소문자 키워드. (예: "left", "come_here", "umbrella")


           <예시>
           - 입력: "아래"
           출력: down
           - 입력: "지금 들고 있는 거 정리해줘"
           출력: place
           - 입력: "돌아가."
           출력: return
           - 입력: "나 지갑 필요해."
           출력: wallet
           - 입력: "오른쪽"
           출력: right
           - 입력: "이쪽으로 와 봐"
           출력: come_here
           - 입력: "여기야"
           출력: come_here
           - 입력: "아니야."
           출력: retry
           - 입력: "충전"
           출력: charger
          - 입력: "풍전"
           출력: charger
           - 입력: 지정된 키워드가 아닌 경우
           출력: retry     
           - 입력: "부산"
           출력: umbrella
           - 입력: "무산"
           출력: umbrella                 


           <사용자 입력>
           "{user_input}"               
       """


       self.prompt_template = PromptTemplate(
           input_variables=["user_input"], template=prompt_content
       )
       self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
       self.stt = STT(openai_api_key=openai_api_key)




       super().__init__("get_keyword_node")
       # 오디오 설정
       mic_config = MicConfig(
           chunk=12000,
           rate=48000,
           channels=1,
           record_seconds=3,
           fmt=pyaudio.paInt16,
           device_index=10,
           buffer_size=24000,
       )
       self.mic_controller = MicController(config=mic_config)
       # self.ai_processor = AIProcessor()


       self.get_logger().info("MicRecorderNode initialized.")
       self.get_logger().info("wait for client's request...")
       self.get_keyword_srv = self.create_service(
           Trigger, "get_keyword", self.get_keyword
       )
       self.wakeup_word = WakeupWord(mic_config.buffer_size)


   def extract_keyword(self, output_message):
       response = self.lang_chain.invoke({"user_input": output_message})
       result = response["text"].strip()


       if not result:
           self.get_logger().warn("LLM 응답이 비어 있습니다.")
           return []


       # --- 코드 블록(```) 제거 로직 시작 ---
       # 현재 'result' (초기 strip 된 상태)가 코드 블록으로 감싸여 있는지 확인합니다.
       if result.startswith("```") and result.endswith("```"):
           # 코드 블록 마커를 제거하고, 내부의 추가 공백/줄바꿈도 제거합니다.
           # 'result' 변수 자체를 업데이트합니다.
           result = result[3:-3].strip()


       # 코드 블록 제거 후 'result'가 비어있다면, 추가적인 로그와 함께 빈 리스트를 반환합니다.
       if not result:
           # 어떤 원본 응답이 비게 되었는지 알 수 있도록 원본(strip 전) 응답을 로그에 포함합니다.
           self.get_logger().warn(f"LLM 응답이 ``` 제거 후 비어있습니다. 원본(strip 전): '{response['text']}'")
           return []
       # --- 코드 블록(```) 제거 로직 끝 ---


       keywords = [keyword for keyword in result.split() if keyword]
       print(f"llm's response: {result}")
       print(f"extracted keywords: {keywords}")

	        # --- 경로 설정 시작 ---
       log_file = os.path.expanduser("~/data_logs/print_log.txt")
       os.makedirs(os.path.dirname(log_file), exist_ok=True)
        # --- 경로 설정 끝 ---
       with open(log_file, "a", encoding="utf-8") as f:
          f.write(f" - 추출된 키워드: {keywords}\n")

       return keywords  # 하나 이상이 있을 수 있으므로 리스트 형태로 반환

  
   def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return response

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keyword Extract --> Embedding
        output_message = self.stt.speech2text()
        keyword = self.extract_keyword(output_message)


        # if output_message is None:
        #     keyword = ['retry']
        #     self.log_extraction_results("No speech detected, retrying...", keyword)
        # else:
        #     keyword = self.extract_keyword(output_message)
        #     self.log_extraction_results(output_message, keyword)
        self.get_logger().warn(f"Detected tools: {keyword}")

        # 응답 객체 설정
        response.success = True
        response.message = " ".join(keyword)  # 감지된 키워드를 응답 메시지로 반환
        return response



def main():
   rclpy.init()
   node = GetKeyword()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()




if __name__ == "__main__":
   main()