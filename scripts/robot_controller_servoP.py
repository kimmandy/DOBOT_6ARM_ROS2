# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import time
# import threading

# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from geometry_msgs.msg import PointStamped
# from dobot_msgs_v4.srv import EnableRobot, ServoP

# # =========================
# # 토픽 / 서비스
# # =========================
# TOPIC_TARGET = "/arm_target_mm" # 카메라 노드에서 publish하는 목표 좌표 (mm 단위)
# SRV_ENABLE = "/dobot_bringup_ros2/srv/EnableRobot"
# SRV_SERVO = "/dobot_bringup_ros2/srv/ServoP"

# # =========================
# # 로봇 BASE 자세
# # =========================
# BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
# BASE_RPY_DEG = np.array([ 178.7294,   -1.9186,  -96.5768], dtype=np.float64)
# BASE_MOVE_T = 5.0 # 시작시 로봇은 항상 베이스 자세로 이동

# # =========================
# # ServoP 스트림 설정
# # =========================
# FIXED_RPY_DEG = BASE_RPY_DEG.copy()
# SEND_HZ = 30.0
# DT = 1.0 / SEND_HZ   # ≈ 0.0333s
# SERVO_T = DT  # t = 전송주기 일치

# PRINT_EVERY_SEC = 1.0
# HOLD_TIMEOUT_SEC = 0.5 # 토픽 끊기면 이 시간 이후 송신 중단


# # =========================
# # 보간 설정
# # INTERP_ALPHA: skip 발생 시 last_sent → latest 사이 보간 비율
# #   1.0 = 즉시 latest (보간 없음, 기존과 동일)
# #   0.5 = 중간점 (권장, 급격한 이동 방지)
# #   0.3 = 더 보수적 (느리지만 안전)
# # =========================
# INTERP_ALPHA = 0.4


# class ArmTargetToServoP(Node):

#     def __init__(self):
#         super().__init__("arm_target_to_servop")

#         self._lock = threading.Lock()
#         self.latest_xyz = None
#         self.last_rx_time = 0.0
#         self._last_future = None

#         # 보간용 상태
#         self._last_sent_xyz = None   # 마지막으로 실제 전송한 좌표
#         self._skip_count = 0      # 연속 skip 횟수 

#         # 통계
#         self.sent = 0
#         self.skipped_busy = 0
#         self.dropped_no_target = 0
#         self.dropped_timeout = 0
#         self.stat_interp = 0   # 보간 발생 횟수

#         self._sent_last = 0
#         self._skip_last = 0
#         self._drop_nt_last = 0
#         self._drop_to_last = 0
#         self._interp_last = 0
#         self.last_print_t = time.perf_counter()

#         # 서비스 클라이언트
#         self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
#         self.servo_cli  = self.create_client(ServoP, SRV_SERVO)

#         self.get_logger().info("Dobot 서비스 대기 중...")
#         self.enable_cli.wait_for_service()
#         self.servo_cli.wait_for_service()
#         self.get_logger().info("서비스 연결 완료.")

#         # QoS: camera_node publisher와 반드시 일치
#         qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=1,
#         )
#         self.sub = self.create_subscription(
#             PointStamped, TOPIC_TARGET, self.on_target, qos
#         )
#         self.get_logger().info(
#             f"[{TOPIC_TARGET}] 구독  QoS=BEST_EFFORT/KEEP_LAST"
#         )

#         # 고정 주기 타이머
#         self.timer = self.create_timer(DT, self.on_timer)
#         self.get_logger().info(
#             f"ServoP 스트림: {SEND_HZ:.0f}Hz  t={SERVO_T:.4f}s  "
#             f"INTERP_ALPHA={INTERP_ALPHA}"
#         )


#     # ── 폴링 방식 future 대기 
#     def _wait_future(self, fut, timeout: float = 10.0) -> bool:
#         start = time.perf_counter()
#         while not fut.done():
#             if time.perf_counter() - start > timeout:
#                 self.get_logger().error("Future 타임아웃")
#                 return False
#             time.sleep(0.01)
#         return True
    


#     def enable_and_move_base(self):
#         """spin 스레드가 살아있는 상태에서 호출"""
#         fut = self.enable_cli.call_async(EnableRobot.Request())
#         if not self._wait_future(fut):
#             return
#         self.get_logger().info("로봇 활성화 완료. BASE 자세로 이동 중...")

#         req = ServoP.Request()
#         req.a, req.b, req.c = float(BASE_TCP_MM[0]), float(BASE_TCP_MM[1]), float(BASE_TCP_MM[2])
#         req.d, req.e, req.f = float(BASE_RPY_DEG[0]), float(BASE_RPY_DEG[1]), float(BASE_RPY_DEG[2])
#         req.param_value = [f"t={BASE_MOVE_T:.4f}"]

#         fut2 = self.servo_cli.call_async(req)
#         if not self._wait_future(fut2):
#             return
#         try:
#             res = fut2.result()
#             if res and hasattr(res, "error_id") and res.error_id != 0:
#                 self.get_logger().error(f"BASE 이동 에러: {res.error_id}")
#             else:
#                 self.get_logger().info("BASE 자세 완료.")
#         except Exception as e:
#             self.get_logger().warn(f"BASE 이동 결과 확인 실패: {e}")
#         time.sleep(0.3)



#     # ── 토픽 수신 
#     def on_target(self, msg: PointStamped):
#         with self._lock:
#             self.latest_xyz   = np.array(
#                 [msg.point.x, msg.point.y, msg.point.z], dtype=np.float64
#             )
#             self.last_rx_time = time.perf_counter()



#     # ── ServoP Request 생성 
#     def _build_req(self, xyz: np.ndarray) -> ServoP.Request:
#         req = ServoP.Request()

#         # 좌표 설정
#         req.a, req.b, req.c = float(xyz[0]), float(xyz[1]), float(xyz[2])
#         req.d, req.e, req.f = float(FIXED_RPY_DEG[0]), float(FIXED_RPY_DEG[1]), float(FIXED_RPY_DEG[2])
#         req.param_value = [f"t={SERVO_T:.4f}"]
#         return req


#     # ── 고정 주기 타이머 
#     def on_timer(self):
#         now = time.perf_counter()
    

#         with self._lock:
#             latest_xyz   = self.latest_xyz.copy() if self.latest_xyz is not None else None
#             last_rx_time = self.last_rx_time

#         # target 없음 -> 카메라에서 못받으면
#         if latest_xyz is None:
#             self.dropped_no_target += 1
#             self._print_stats(now)
#             return

#         # 토픽 타임아웃
#         if (now - last_rx_time) > HOLD_TIMEOUT_SEC:
#             self.dropped_timeout += 1
#             self._print_stats(now)
#             return

#         # ServoP 처리 중: skip 횟수만 누적 (이전과 달리 return 하지 않음)
#         if self._last_future is not None and not self._last_future.done():
#             self.skipped_busy += 1
#             self._skip_count  += 1
#             self._print_stats(now)
#             return

#         # 이전 ServoP 에러 체크
#         if self._last_future is not None:
#             try:
#                 res = self._last_future.result()
#                 if res and hasattr(res, "error_id") and res.error_id != 0:
#                     self.get_logger().warn(f"ServoP 에러: {res.error_id}")
#             except Exception as e:
#                 self.get_logger().error(f"ServoP 결과 확인 실패: {e}")

#         # 보간: skip이 있었으면 last_sent와 latest 사이 중간점으로 전송
#         if self._skip_count > 0 and self._last_sent_xyz is not None:
#             # skip이 많을수록 더 보수적으로 (천천히 따라가기)
#             # skip=1 → alpha=0.5, skip=2 → alpha=0.33, skip=3 → alpha=0.25
#             alpha = INTERP_ALPHA / (self._skip_count + 1)
#             alpha = max(0.1, min(alpha, INTERP_ALPHA))  # 0.1~INTERP_ALPHA 사이로 clamp

#             send_xyz = (
#                 (1.0 - alpha) * self._last_sent_xyz
#                 + alpha       * latest_xyz
#             )
#             self.stat_interp += 1
#             self.get_logger().debug(
#                 f"[INTERP] skip={self._skip_count}  alpha={alpha:.2f}  "
#                 f"last=({self._last_sent_xyz[0]:.1f},{self._last_sent_xyz[1]:.1f},{self._last_sent_xyz[2]:.1f})  "
#                 f"latest=({latest_xyz[0]:.1f},{latest_xyz[1]:.1f},{latest_xyz[2]:.1f})  "
#                 f"send=({send_xyz[0]:.1f},{send_xyz[1]:.1f},{send_xyz[2]:.1f})"
#             )
#         else:
#             send_xyz = latest_xyz

#         self._skip_count = 0
#         self._last_sent_xyz = send_xyz.copy()

#         self._last_future = self.servo_cli.call_async(self._build_req(send_xyz))
#         self.sent += 1
#         self._print_stats(now)

#     # ── 통계 출력 
#     def _print_stats(self, now: float):
#         if now - self.last_print_t < PRINT_EVERY_SEC:
#             return

#         sent_s = self.sent - self._sent_last
#         skip_s = self.skipped_busy- self._skip_last
#         drop_nt_s = self.dropped_no_target - self._drop_nt_last
#         drop_to_s = self.dropped_timeout - self._drop_to_last
#         interp_s = self.stat_interp - self._interp_last
#         self._sent_last = self.sent
#         self._skip_last = self.skipped_busy
#         self._drop_nt_last = self.dropped_no_target
#         self._drop_to_last = self.dropped_timeout
#         self._interp_last = self.stat_interp
#         self.last_print_t = now

#         with self._lock:
#             xyz = self.latest_xyz
#         xyz_str = (
#             f"({xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f})"
#             if xyz is not None else "None"
#         )

#         self.get_logger().info(
#             f"[STATS] sent/s={sent_s}  skip/s={skip_s}  interp/s={interp_s}  "
#             f"drop_no_target/s={drop_nt_s}  drop_timeout/s={drop_to_s}  "
#             f"xyz={xyz_str}"
#         )


# # =========================
# # main
# # =========================

# def main():
#     rclpy.init()
#     node = ArmTargetToServoP()

#     # spin 스레드 먼저 시작 → enable_and_move_base의 future 처리 가능
#     spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     spin_thread.start()

#     node.enable_and_move_base()

 
#     print(f"[robot_controller] {SEND_HZ:.0f}Hz  t={SERVO_T:.4f}s  "
#           f"INTERP_ALPHA={INTERP_ALPHA}")
#     print("카메라 노드의 토픽을 기다리는 중...")
#     print("Ctrl+C 로 종료")

#     try:
#         while spin_thread.is_alive():
#             time.sleep(0.1)
#     except KeyboardInterrupt:
#         print("\n[QUIT] Ctrl+C")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         print("robot_controller 종료.")


# if __name__ == "__main__":
#     main()






#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PointStamped
from dobot_msgs_v4.srv import EnableRobot, ServoP

# =========================
# 설정 상수
# =========================
TOPIC_TARGET = "/arm_target_mm"
SRV_ENABLE = "/dobot_bringup_ros2/srv/EnableRobot"
SRV_SERVO = "/dobot_bringup_ros2/srv/ServoP"

BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
BASE_RPY_DEG = np.array([ 178.7294, -1.9186, -96.5768], dtype=np.float64)
BASE_MOVE_T = 5.0

SEND_HZ = 30.0
DT = 1.0 / SEND_HZ   # 약 0.0333s
# SERVO_T가 DT보다 약간 커야 로봇이 다음 점을 받기 전까지 속도를 유지합니다 (Blending)
SERVO_T = DT * 1.2   

HOLD_TIMEOUT_SEC = 0.5 
PRINT_EVERY_SEC = 1.0
INTERP_ALPHA = 0.4  # 목표 추종 강도 (0.1 ~ 1.0)

class ArmTargetToServoP(Node):
    def __init__(self):
        super().__init__("arm_target_to_servop")

        self._lock = threading.Lock()
        self.latest_xyz = None
        self.last_rx_time = 0.0
        self._last_sent_xyz = None
        self._last_future = None

        # 통계용
        self.sent = 0
        self.skipped_busy = 0
        self.dropped_no_target = 0
        self.last_print_t = time.perf_counter()

        # 서비스 클라이언트
        self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
        self.servo_cli  = self.create_client(ServoP, SRV_SERVO)

        self.get_logger().info("Dobot 서비스 연결 대기 중...")
        self.enable_cli.wait_for_service()
        self.servo_cli.wait_for_service()

        # QoS 설정 (카메라 노드와 매칭)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(PointStamped, TOPIC_TARGET, self.on_target, qos)
        
        # 주기적 실행 타이머
        self.timer = self.create_timer(DT, self.on_timer)
        self.get_logger().info(f"제어 노드 시작: {SEND_HZ}Hz / ServoT: {SERVO_T:.4f}s")

    def on_target(self, msg: PointStamped):
        with self._lock:
            self.latest_xyz = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
            self.last_rx_time = time.perf_counter()

    def _wait_future(self, fut, timeout=10.0):
        start = time.perf_counter()
        while not fut.done():
            if time.perf_counter() - start > timeout: return False
            time.sleep(0.01)
        return True

    def enable_and_move_base(self):
        self.enable_cli.call_async(EnableRobot.Request())
        time.sleep(1.0) # 활성화 대기
        
        req = ServoP.Request()
        req.a, req.b, req.c = map(float, BASE_TCP_MM)
        req.d, req.e, req.f = map(float, BASE_RPY_DEG)
        req.param_value = [f"t={BASE_MOVE_T}"]
        
        self.get_logger().info("BASE 위치로 이동합니다...")
        fut = self.servo_cli.call_async(req)
        self._wait_future(fut)
        self._last_sent_xyz = BASE_TCP_MM.copy()
        self.get_logger().info("준비 완료.")

    def on_timer(self):
        now = time.perf_counter()

        with self._lock:
            latest = self.latest_xyz.copy() if self.latest_xyz is not None else None
            rx_t = self.last_rx_time

        # 1. 유효성 검사
        if latest is None:
            self.dropped_no_target += 1
            return
        if (now - rx_t) > HOLD_TIMEOUT_SEC:
            return

        # 2. 이전 명령 처리 상태 확인 (동기식 유지)
        if self._last_future is not None and not self._last_future.done():
            self.skipped_busy += 1
            return

        # 3. 보간 및 타겟 계산
        # 현재 위치(last_sent)에서 목표 위치(latest)로 부드럽게 이동
        if self._last_sent_xyz is not None:
            send_xyz = (1.0 - INTERP_ALPHA) * self._last_sent_xyz + INTERP_ALPHA * latest
        else:
            send_xyz = latest

        # 4. ServoP 요청 생성 및 전송
        req = ServoP.Request()
        req.a, req.b, req.c = map(float, send_xyz)
        req.d, req.e, req.f = map(float, BASE_RPY_DEG)
        req.param_value = [f"t={SERVO_T:.4f}"]

        self._last_future = self.servo_cli.call_async(req)
        self._last_sent_xyz = send_xyz.copy()
        self.sent += 1

        self._print_stats(now)

    def _print_stats(self, now):
        if now - self.last_print_t < PRINT_EVERY_SEC: return
        self.get_logger().info(f"[RUN] Sent: {self.sent} | Skip(Busy): {self.skipped_busy} | Target: {self._last_sent_xyz.round(1)}")
        self.last_print_t = now

def main():
    rclpy.init()
    node = ArmTargetToServoP()
    
    # 별도 스레드에서 spin 실행 (서비스 응답 처리를 위해 필수)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    node.enable_and_move_base()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()