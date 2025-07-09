import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from datetime import datetime

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

from pydub import AudioSegment
from pydub.playback import play

package_path = get_package_share_directory("pick_and_place_voice")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
RELATIVE_MOVE_VELOCITY, RELATIVE_MOVE_ACC = 30, 30 
MOVE_STEP = 85.0
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = 23
MIN_DEPTH = 2.0

ROKEY_ACTIONS = ["place", "return", "right", "left", "forward", "backward", "up", "down", "come_here"]

# DSR Init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

TARGET_POS = [350.0, 100, 200, 90, -87, 85]

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, posx, get_tool_force
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# Gripper Init
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

def check_grip():
    if gripper.get_status()[1]==0:
        return
    song = AudioSegment.from_mp3("06.mp3") # 물건을 잡아주세요
    play(song)
    threshold = 5.0
    initial_force = get_tool_force() # force_ext: WORLD좌표계 기준 툴의 외력
    stable_start_time = time.time()
    while True:
        current_force = get_tool_force()
        force_diff = [abs(c - i) for c, i in zip(current_force, initial_force)]
        if any(diff > threshold for diff in force_diff):
            print(f"[INFO] 외력 변화 감지: {force_diff}")
            gripper.open_gripper()
            break
        else:
            elapsed = time.time() - stable_start_time
            if elapsed >= 5.0:
                print("[WARN] 외력 변화 없음 - 5초간 정지 상태로 간주. 작동 중지.")
                song = AudioSegment.from_mp3("07.mp3")
                play(song)

                break
    time.sleep(0.1)

class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        # 좌표 구독 설정 (move_to_face 용)
        self.xy_coords = [0.0, 0.0]
        self.received = False
        self.create_subscription(Float64MultiArray, "/remapped_coord", self.xy_callback, 10)

    def xy_callback(self, msg):
        if not self.received and len(msg.data) >= 2:
            self.xy_coords = msg.data[:2]
            self.received = True
            self.get_logger().info(f"[INFO] 좌표 수신 완료: x={self.xy_coords[0]:.2f}, z={self.xy_coords[1]:.2f}")

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]
    
    def execute_relative_move(self, direction):
        from DSR_ROBOT2 import movel, mwait
        from DR_common2 import posx
        try: from DSR_ROBOT2 import DR_BASE, DR_MV_MOD_REL # 사용자가 DSR_ROBOT2에 있다고 함
        except ImportError: self.get_logger().error("DSR_ROBOT2에서 DR_BASE 또는 DR_MV_MOD_REL 상수를 임포트할 수 없습니다."); return
        
        delta_x, delta_y, delta_z = 0.0, 0.0, 0.0
        if direction == "left": delta_x = -MOVE_STEP
        elif direction == "right": delta_x = MOVE_STEP
        elif direction == "forward": delta_y = -MOVE_STEP # 사용자 정의 방향
        elif direction == "backward": delta_y = MOVE_STEP# 사용자 정의 방향
        elif direction == "up": delta_z = MOVE_STEP
        elif direction == "down": delta_z = -MOVE_STEP
        else: self.get_logger().warn(f"알 수 없는 상대이동 방향: {direction}"); return
        
        relative_target_pos = posx(delta_x, delta_y, delta_z, 0, 0, 0)
        self.get_logger().info(f"'{direction}' 방향으로 {MOVE_STEP}mm 상대 이동 (기준: DR_BASE): X={delta_x}, Y={delta_y}, Z={delta_z}")
        try:
            movel(relative_target_pos, vel=RELATIVE_MOVE_VELOCITY, acc=RELATIVE_MOVE_ACC, ref=DR_BASE, mod=DR_MV_MOD_REL) 
            mwait()
            self.get_logger().info(f"'{direction}' 방향 상대 이동 완료.")
        except Exception as e: self.get_logger().error(f"상대 이동 중 오류: {e}")


    def robot_control(self):
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)
        

        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()
            spoken_text = get_keyword_result.message
            target_list = spoken_text.split()

            if "come_here" in target_list:
                self.get_logger().info("[INFO] 'face' 명령 감지 → move_to_face 실행")
                song = AudioSegment.from_mp3("01.mp3")
                play(song)
                self.move_to_face()
                check_grip()
                return
            
            elif any(direction in target_list for direction in ["right", "left", "forward", "backward", "up", "down"]):
                song = AudioSegment.from_mp3("01.mp3")
                play(song)
                self.execute_relative_move(target_list[0])
                check_grip()
                return
            
            elif any(direction in target_list for direction in ["charger", "wallet", "umbrella"]):
                # 물건 가져다 드릴게요
                song = AudioSegment.from_mp3("02.mp3")
                play(song)

                # 물건 집기
                target_pos = self.get_target_pos(target_list[0])
                if target_pos is None:
                    self.get_logger().warn("물체 감지 실패. 다시 시도하세요.")
                    return self.robot_control()
 
                global TARGET_POS
                TARGET_POS = target_pos.copy()
                print(f"target_pos : {target_pos}")
                print(f"recent_target_pos : {TARGET_POS}")


                if target_pos is None:
                    self.get_logger().warn("No target position")
                else:
                    self.get_logger().info(f"target position: {target_pos}")
                    self.pick_and_place_target(target_pos)

                # 사용자에게 옮기기
                # self.move_to_face()
                return
            
            elif "place" in target_list:
                song = AudioSegment.from_mp3("03.mp3")
                play(song)
                print(f"recent_target_pos : {TARGET_POS}")
                z_add = [TARGET_POS[0],TARGET_POS[1],TARGET_POS[2]+DEPTH_OFFSET,TARGET_POS[3],TARGET_POS[4],TARGET_POS[5]]
                if z_add[2] <= 120:
                    z_add[2] = 120
                # --- 경로 설정 시작 ---
                log_file = os.path.expanduser("~/data_logs/print_log.txt")
                os.makedirs(os.path.dirname(log_file), exist_ok=True)
                # --- 경로 설정 끝 ---
                with open(log_file, "a", encoding="utf-8") as f:
                    f.write(f" - place location: {z_add[:3]}\n------------------------------\n")
                mwait()
                gripper.close_gripper(200)
                while gripper.get_status()[0]:
                    time.sleep(0.5)
                mwait()
                movel(z_add,VELOCITY,ACC)
                gripper.open_gripper()
                while gripper.get_status()[0]:
                    time.sleep(0.5)
                mwait()
                self.init_robot()


            elif "return" in target_list:
                song = AudioSegment.from_mp3("01.mp3")
                play(song)
                self.init_robot()

            elif "retry" in target_list:
                song = AudioSegment.from_mp3("04.mp3")
                play(song)
                self.robot_control()

        else:
            self.get_logger().warn(f"{get_keyword_future.result().message}")

    def get_target_pos(self, target):
        retries = 3  # 재시도 횟수
        attempt = 0   # 현재 시도 횟수
        
        while attempt < retries:
            self.get_position_request.target = target
            self.get_logger().info("call depth position service with object_detection node")
            get_position_future = self.get_position_client.call_async(self.get_position_request)
            rclpy.spin_until_future_complete(self, get_position_future)

            if get_position_future.result():
                result = get_position_future.result().depth_position.tolist()
                self.get_logger().info(f"Received depth position: {result}")
                if sum(result) == 0:
                    self.get_logger().info("물체 감지되지 않음. 재시도 중...")
                    attempt += 1  # 재시도 횟수 증가
                    time.sleep(2)  # 2초 후에 다시 시도
                    continue  # 재시도
                else:
                    gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                    robot_posx = get_current_posx()[0]
                    td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

                    if td_coord[2] and sum(td_coord) != 0:
                        td_coord[2] += DEPTH_OFFSET
                        td_coord[2] = max(td_coord[2], MIN_DEPTH)

                    target_pos = list(td_coord[:3]) + robot_posx[3:]
                    return target_pos
            else:
                self.get_logger().info("Service 호출 실패.")
                return None

        # 재시도 후에도 감지 실패하면 None 반환
        self.get_logger().info("물체를 찾을 수 없습니다. 재시도 횟수 초과. 음성대기 상태 ")
        return self.robot_control()

    def init_robot(self):
        JReady = [31.3, 13, 54.75, 0, 112.15, 31.36]
        movej(JReady, vel=VELOCITY, acc=ACC)
        # gripper.open_gripper()
        # mwait()

    def execute_relative_move(self, direction):
        from DSR_ROBOT2 import movel, mwait
        from DR_common2 import posx
        try: from DSR_ROBOT2 import DR_BASE, DR_MV_MOD_REL # 사용자가 DSR_ROBOT2에 있다고 함
        except ImportError: self.get_logger().error("DSR_ROBOT2에서 DR_BASE 또는 DR_MV_MOD_REL 상수를 임포트할 수 없습니다."); return
        
        delta_x, delta_y, delta_z = 0.0, 0.0, 0.0
        if direction == "left": delta_x = -MOVE_STEP
        elif direction == "right": delta_x = MOVE_STEP
        elif direction == "forward": delta_y = -MOVE_STEP # 사용자 정의 방향
        elif direction == "backward": delta_y = MOVE_STEP# 사용자 정의 방향
        elif direction == "up": delta_z = MOVE_STEP
        elif direction == "down": delta_z = -MOVE_STEP
        else: self.get_logger().warn(f"알 수 없는 상대이동 방향: {direction}"); return
        
        relative_target_pos = posx(delta_x, delta_y, delta_z, 0, 0, 0)
        self.get_logger().info(f"'{direction}' 방향으로 {MOVE_STEP}mm 상대 이동 (기준: DR_BASE): X={delta_x}, Y={delta_y}, Z={delta_z}")
        try:
            movel(relative_target_pos, vel=RELATIVE_MOVE_VELOCITY, acc=RELATIVE_MOVE_ACC, ref=DR_BASE, mod=DR_MV_MOD_REL) 
            mwait()
            self.get_logger().info(f"'{direction}' 방향 상대 이동 완료.")
        except Exception as e: self.get_logger().error(f"상대 이동 중 오류: {e}")

    def move_to_face(self):
        self.received = False
        self.get_logger().info("[INFO] 얼굴 좌표를 기다리는 중...")

        timeout_counter = 0
        while rclpy.ok() and not self.received:
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout_counter += 1
            if timeout_counter > 100:  # 10초 이상 기다렸다면 중단
                self.get_logger().error("[ERROR] 얼굴 좌표를 수신하지 못했습니다.")
                return

        pos = posx([
            self.xy_coords[0],  # x
            -252.0,             # y 고정
            self.xy_coords[1],
            # 540,                # z 고정
            91, -88.00, 90.09   # 오리엔테이션 고정
        ])
        self.get_logger().info("→ 얼굴 좌표로 이동")
        movel(pos, vel=VELOCITY, acc=ACC)

    def pick_and_place_target(self, target_pos):
        real_pose = posx([target_pos[0],
                          target_pos[1],
                          target_pos[2]+DEPTH_OFFSET,
                          target_pos[3],
                          target_pos[4],
                          target_pos[5]])
        if real_pose[2] <= 120:
            real_pose[2] = 120
         # --- 경로 설정 시작 ---
        log_file = os.path.expanduser("~/data_logs/print_log.txt")
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        # --- 경로 설정 끝 ---
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(f" - object location: {real_pose[:3]}\n------------------------------\n")
        movel(real_pose, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper(270)
        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()
        self.move_to_face()
        check_grip()
        # gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

def main(args=None):
    node = RobotController()
    gripper.open_gripper()
    try:
        while rclpy.ok():
            node.robot_control()
    except KeyboardInterrupt:
        print("\n[INFO] 사용자 종료 (Ctrl+C).")
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()