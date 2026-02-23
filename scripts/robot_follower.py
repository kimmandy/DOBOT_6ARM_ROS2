# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import time
# import numpy as np

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped

# from dobot_msgs_v4.srv import EnableRobot, ServoP

# # =========================
# # 설정
# # =========================
# TOPIC_TARGET = "/arm_target_mm"
# SRV_ENABLE = "/dobot_bringup_ros2/srv/EnableRobot"
# SRV_SERVO  = "/dobot_bringup_ros2/srv/ServoP"

# # 로봇 자세(고정)
# FIXED_RPY_DEG = np.array([178.7294, -1.9186, -96.5768], dtype=np.float64)

# # ★ 카메라 FPS에 맞춰 고정 주기 송신 (t와 주기 일치)
# SEND_HZ = 30.0
# DT = 1.0 / SEND_HZ
# SERVO_T = DT

# # 통계 출력 주기(초)
# PRINT_EVERY_SEC = 1.0

# # pose 끊기거나 토픽 끊겼을 때 마지막 값 유지 시간(초)
# HOLD_TIMEOUT_SEC = 0.5

# # =========================
# # 노드
# # =========================
# class ArmTargetToServoP(Node):
#     def __init__(self):
#         super().__init__("arm_target_to_servop")

#         # 최신 target 저장
#         self.latest_xyz = None
#         self.latest_rxryrz = FIXED_RPY_DEG.copy()
#         self.last_rx_time = 0.0

#         # busy 체크용
#         self._last_future = None

#         # 통계
#         self.sent = 0
#         self.skipped_busy = 0
#         self.dropped_no_target = 0
#         self.last_print_t = time.perf_counter()
#         self._sent_last = 0
#         self._skip_last = 0
#         self._drop_last = 0

#         # 토픽 구독
#         self.sub = self.create_subscription(PointStamped, TOPIC_TARGET, self.on_target, 10)

#         # 서비스 클라
#         self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
#         self.servo_cli  = self.create_client(ServoP, SRV_SERVO)

#         self.get_logger().info("Waiting for Dobot services...")
#         self.enable_cli.wait_for_service()
#         self.servo_cli.wait_for_service()
#         self.get_logger().info("Services ready.")

#         # EnableRobot
#         fut = self.enable_cli.call_async(EnableRobot.Request())
#         rclpy.spin_until_future_complete(self, fut)
#         self.get_logger().info("Robot enabled.")

#         # 고정 주기 타이머: DT마다 전송
#         self.timer = self.create_timer(DT, self.on_timer)

#         self.get_logger().info(
#             f"Sub {TOPIC_TARGET} -> ServoP stream @ {SEND_HZ:.1f}Hz  (t={SERVO_T:.4f}s)"
#         )
#         self.get_logger().info(
#             f"HOLD_TIMEOUT_SEC={HOLD_TIMEOUT_SEC:.2f}s (토픽 끊기면 잠깐 마지막 값 유지)"
#         )

#     def on_target(self, msg: PointStamped):
#         self.latest_xyz = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
#         self.last_rx_time = time.perf_counter()

#     def _build_req(self, xyz_mm: np.ndarray) -> ServoP.Request:
#         req = ServoP.Request()
#         req.a, req.b, req.c = float(xyz_mm[0]), float(xyz_mm[1]), float(xyz_mm[2])
#         req.d, req.e, req.f = float(self.latest_rxryrz[0]), float(self.latest_rxryrz[1]), float(self.latest_rxryrz[2])
#         req.param_value = [f"t={SERVO_T:.4f}"]
#         return req

#     def on_timer(self):
#         now = time.perf_counter()

#         # target이 없으면 전송 안 함
#         if self.latest_xyz is None:
#             self.dropped_no_target += 1
#             self._print_stats(now)
#             return

#         # 토픽이 끊긴 경우: HOLD_TIMEOUT까지만 마지막 값 유지
#         if (now - self.last_rx_time) > HOLD_TIMEOUT_SEC:
#             # 여기서 “멈춰라” 같은 명령이 별도로 없어서 안전하게 송신 중단
#             self.dropped_no_target += 1
#             self._print_stats(now)
#             return

#         # 이전 ServoP가 아직 처리중이면 드랍(큐 쌓지 않음)
#         if self._last_future is not None and (not self._last_future.done()):
#             self.skipped_busy += 1
#             self._print_stats(now)
#             return

#         # 전송
#         self._last_future = self.servo_cli.call_async(self._build_req(self.latest_xyz))
#         self.sent += 1
#         self._print_stats(now)

#     def _print_stats(self, now: float):
#         if (now - self.last_print_t) < PRINT_EVERY_SEC:
#             return

#         sent_s = self.sent - self._sent_last
#         skip_s = self.skipped_busy - self._skip_last
#         drop_s = self.dropped_no_target - self._drop_last

#         self._sent_last = self.sent
#         self._skip_last = self.skipped_busy
#         self._drop_last = self.dropped_no_target
#         self.last_print_t = now

#         last = self.latest_xyz
#         if last is None:
#             self.get_logger().info(f"[STATS] sent/s={sent_s} skip/s={skip_s} drop/s={drop_s} last=None")
#         else:
#             self.get_logger().info(
#                 f"[STATS] sent/s={sent_s} skip/s={skip_s} drop/s={drop_s} "
#                 f"last=({last[0]:.1f},{last[1]:.1f},{last[2]:.1f})"
#             )

# def main():
#     rclpy.init()
#     node = ArmTargetToServoP()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()





#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from dobot_msgs_v4.srv import EnableRobot, ServoP

# =========================
# 토픽/서비스
# =========================
TOPIC_TARGET = "/arm_target_mm"
SRV_ENABLE = "/dobot_bringup_ros2/srv/EnableRobot"
SRV_SERVO  = "/dobot_bringup_ros2/srv/ServoP"

# =========================
# 로봇 BASE 이동(1회)
# =========================
BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
BASE_RPY_DEG = np.array([178.7294, -1.9186, -96.5768], dtype=np.float64)
BASE_MOVE_T = 3.0

# =========================
# ServoP 스트림 설정
# =========================
FIXED_RPY_DEG = BASE_RPY_DEG.copy()

SEND_HZ = 30.0
DT = 1.0 / SEND_HZ
SERVO_T = DT  # ★ t=전송주기 일치

PRINT_EVERY_SEC = 1.0
HOLD_TIMEOUT_SEC = 0.5  # 토픽 끊기면 이 시간 이후 송신 중단(안전)

class ArmTargetToServoP(Node):
    def __init__(self):
        super().__init__("arm_target_to_servop_with_base_move")

        self.latest_xyz = None
        self.last_rx_time = 0.0

        self._last_future = None

        self.sent = 0
        self.skipped_busy = 0
        self.dropped_no_target = 0

        self.last_print_t = time.perf_counter()
        self._sent_last = 0
        self._skip_last = 0
        self._drop_last = 0

        self.sub = self.create_subscription(PointStamped, TOPIC_TARGET, self.on_target, 10)

        self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
        self.servo_cli  = self.create_client(ServoP, SRV_SERVO)

        self.get_logger().info("Waiting for Dobot services...")
        self.enable_cli.wait_for_service()
        self.servo_cli.wait_for_service()
        self.get_logger().info("Services ready.")

        # Enable + BASE 이동(블로킹)
        self._enable_and_move_base_blocking()

        # 고정 주기 타이머: DT마다 전송
        self.timer = self.create_timer(DT, self.on_timer)

        self.get_logger().info(
            f"Sub {TOPIC_TARGET} -> ServoP stream @ {SEND_HZ:.1f}Hz  (t={SERVO_T:.4f}s)"
        )

    def _enable_and_move_base_blocking(self):
        fut = self.enable_cli.call_async(EnableRobot.Request())
        rclpy.spin_until_future_complete(self, fut)
        self.get_logger().info("Robot enabled. Moving to BASE...")

        req = ServoP.Request()
        req.a, req.b, req.c = float(BASE_TCP_MM[0]), float(BASE_TCP_MM[1]), float(BASE_TCP_MM[2])
        req.d, req.e, req.f = float(BASE_RPY_DEG[0]), float(BASE_RPY_DEG[1]), float(BASE_RPY_DEG[2])
        req.param_value = [f"t={float(BASE_MOVE_T):.4f}"]

        fut2 = self.servo_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut2)

        try:
            res = fut2.result()
            if res is not None and hasattr(res, "error_id") and res.error_id != 0:
                self.get_logger().error(f"BASE move error_id={res.error_id}")
            else:
                self.get_logger().info("BASE move done.")
        except Exception as e:
            self.get_logger().warn(f"BASE move result check failed: {e}")

        time.sleep(0.3)

    def on_target(self, msg: PointStamped):
        self.latest_xyz = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
        self.last_rx_time = time.perf_counter()

    def _build_req(self, xyz_mm: np.ndarray) -> ServoP.Request:
        req = ServoP.Request()
        req.a, req.b, req.c = float(xyz_mm[0]), float(xyz_mm[1]), float(xyz_mm[2])
        req.d, req.e, req.f = float(FIXED_RPY_DEG[0]), float(FIXED_RPY_DEG[1]), float(FIXED_RPY_DEG[2])
        req.param_value = [f"t={SERVO_T:.4f}"]
        return req

    def on_timer(self):
        now = time.perf_counter()

        # target 없으면 송신 안 함
        if self.latest_xyz is None:
            self.dropped_no_target += 1
            self._print_stats(now)
            return

        # 토픽 끊긴 경우: HOLD_TIMEOUT까지만 유지 후 송신 중단
        if (now - self.last_rx_time) > HOLD_TIMEOUT_SEC:
            self.dropped_no_target += 1
            self._print_stats(now)
            return

        # 이전 ServoP가 아직 처리 중이면 이번 tick은 skip (큐 쌓지 않음)
        if self._last_future is not None and (not self._last_future.done()):
            self.skipped_busy += 1
            self._print_stats(now)
            return

        self._last_future = self.servo_cli.call_async(self._build_req(self.latest_xyz))
        self.sent += 1
        self._print_stats(now)

    def _print_stats(self, now: float):
        if (now - self.last_print_t) < PRINT_EVERY_SEC:
            return

        sent_s = self.sent - self._sent_last
        skip_s = self.skipped_busy - self._skip_last
        drop_s = self.dropped_no_target - self._drop_last

        self._sent_last = self.sent
        self._skip_last = self.skipped_busy
        self._drop_last = self.dropped_no_target
        self.last_print_t = now

        last = self.latest_xyz
        if last is None:
            self.get_logger().info(f"[STATS] sent/s={sent_s} skip/s={skip_s} drop/s={drop_s} last=None")
        else:
            self.get_logger().info(
                f"[STATS] sent/s={sent_s} skip/s={skip_s} drop/s={drop_s} "
                f"last=({last[0]:.1f},{last[1]:.1f},{last[2]:.1f})"
            )

def main():
    rclpy.init()
    node = ArmTargetToServoP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
