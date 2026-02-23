#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_controller_servoJ.py
===========================
ServoJ 방식: 관절각 스트리밍으로 실시간 팔 추종

인터페이스 (dobot_msgs_v4/srv/ServoJ)
  Request : float64 a,b,c,d,e,f  /  string[] param_value
  Response: int32 res
"""

import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PointStamped
from dobot_msgs_v4.srv import EnableRobot, ServoJ

# =========================
# 토픽 / 서비스
# =========================
TOPIC_TARGET = "/arm_target_mm"
SRV_ENABLE   = "/dobot_bringup_ros2/srv/EnableRobot"
SRV_SERVO_J  = "/dobot_bringup_ros2/srv/ServoJ"

# =========================
# 로봇 BASE 자세
# =========================
BASE_TCP_MM  = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
BASE_RPY_DEG = np.array([ 178.7294,   -1.9186,  -96.5768], dtype=np.float64)
BASE_MOVE_T  = 3.0

# =========================
# ServoJ 스트림 설정
# =========================
SEND_HZ          = 30.0
DT               = 1.0 / SEND_HZ
SERVO_T          = DT

PRINT_EVERY_SEC  = 1.0
HOLD_TIMEOUT_SEC = 0.5
INTERP_ALPHA     = 0.5


class ArmTargetToServoJ(Node):

    def __init__(self):
        super().__init__("arm_target_to_servoj")

        self._lock          = threading.Lock()
        self.latest_xyz     = None
        self.last_rx_time   = 0.0
        self._last_future   = None
        self._last_sent_xyz = None
        self._skip_count    = 0

        self.sent              = 0
        self.skipped_busy      = 0
        self.dropped_no_target = 0
        self.dropped_timeout   = 0
        self.stat_interp       = 0

        self._sent_last    = 0
        self._skip_last    = 0
        self._drop_nt_last = 0
        self._drop_to_last = 0
        self._interp_last  = 0
        self.last_print_t  = time.perf_counter()

        self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
        self.servoj_cli = self.create_client(ServoJ, SRV_SERVO_J)

        self.get_logger().info("Dobot 서비스 대기 중...")
        self.enable_cli.wait_for_service()
        self.servoj_cli.wait_for_service()
        self.get_logger().info("서비스 연결 완료.")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            PointStamped, TOPIC_TARGET, self.on_target, qos
        )

        self.timer = self.create_timer(DT, self.on_timer)
        self.get_logger().info(
            f"ServoJ: {SEND_HZ:.0f}Hz  t={SERVO_T:.4f}s  INTERP_ALPHA={INTERP_ALPHA}"
        )

    def _wait_future(self, fut, timeout=10.0):
        start = time.perf_counter()
        while not fut.done():
            if time.perf_counter() - start > timeout:
                self.get_logger().error("Future 타임아웃")
                return False
            time.sleep(0.01)
        return True

    def enable_and_move_base(self):
        fut = self.enable_cli.call_async(EnableRobot.Request())
        if not self._wait_future(fut):
            return
        self.get_logger().info("로봇 활성화 완료. BASE 자세로 이동 중...")

        req = ServoJ.Request()
        req.a = float(BASE_TCP_MM[0])
        req.b = float(BASE_TCP_MM[1])
        req.c = float(BASE_TCP_MM[2])
        req.d = float(BASE_RPY_DEG[0])
        req.e = float(BASE_RPY_DEG[1])
        req.f = float(BASE_RPY_DEG[2])
        req.param_value = [f"t={BASE_MOVE_T:.4f}"]

        fut2 = self.servoj_cli.call_async(req)
        if not self._wait_future(fut2):
            return
        try:
            res = fut2.result()
            # ★ 응답 필드: res (error_id 아님)
            if res is not None and res.res != 0:
                self.get_logger().error(f"BASE 이동 에러: res={res.res}")
            else:
                self.get_logger().info("BASE 자세 완료.")
        except Exception as e:
            self.get_logger().warn(f"BASE 이동 결과 확인 실패: {e}")
        time.sleep(0.3)

    def on_target(self, msg: PointStamped):
        with self._lock:
            self.latest_xyz   = np.array(
                [msg.point.x, msg.point.y, msg.point.z], dtype=np.float64
            )
            self.last_rx_time = time.perf_counter()

    def _build_req(self, xyz: np.ndarray) -> ServoJ.Request:
        req = ServoJ.Request()
        req.a = float(xyz[0])
        req.b = float(xyz[1])
        req.c = float(xyz[2])
        req.d = float(BASE_RPY_DEG[0])
        req.e = float(BASE_RPY_DEG[1])
        req.f = float(BASE_RPY_DEG[2])
        req.param_value = [f"t={SERVO_T:.4f}"]
        return req

    def on_timer(self):
        now = time.perf_counter()

        with self._lock:
            latest_xyz   = self.latest_xyz.copy() if self.latest_xyz is not None else None
            last_rx_time = self.last_rx_time

        if latest_xyz is None:
            self.dropped_no_target += 1
            self._print_stats(now)
            return

        if (now - last_rx_time) > HOLD_TIMEOUT_SEC:
            self.dropped_timeout += 1
            self._print_stats(now)
            return

        if self._last_future is not None and not self._last_future.done():
            self.skipped_busy += 1
            self._skip_count  += 1
            self._print_stats(now)
            return

        # 이전 응답 에러 체크
        if self._last_future is not None:
            try:
                res = self._last_future.result()
                if res is not None and res.res != 0:
                    self.get_logger().warn(f"ServoJ 에러: res={res.res}")
            except Exception as e:
                self.get_logger().error(f"ServoJ 결과 확인 실패: {e}")

        # 보간
        if self._skip_count > 0 and self._last_sent_xyz is not None:
            alpha    = max(0.1, min(INTERP_ALPHA / (self._skip_count + 1), INTERP_ALPHA))
            send_xyz = (1.0 - alpha) * self._last_sent_xyz + alpha * latest_xyz
            self.stat_interp += 1
        else:
            send_xyz = latest_xyz

        self._skip_count    = 0
        self._last_sent_xyz = send_xyz.copy()

        self._last_future = self.servoj_cli.call_async(self._build_req(send_xyz))
        self.sent += 1
        self._print_stats(now)

    def _print_stats(self, now):
        if now - self.last_print_t < PRINT_EVERY_SEC:
            return

        sent_s    = self.sent              - self._sent_last
        skip_s    = self.skipped_busy      - self._skip_last
        drop_nt_s = self.dropped_no_target - self._drop_nt_last
        drop_to_s = self.dropped_timeout   - self._drop_to_last
        interp_s  = self.stat_interp       - self._interp_last

        self._sent_last    = self.sent
        self._skip_last    = self.skipped_busy
        self._drop_nt_last = self.dropped_no_target
        self._drop_to_last = self.dropped_timeout
        self._interp_last  = self.stat_interp
        self.last_print_t  = now

        with self._lock:
            xyz = self.latest_xyz
        xyz_str = f"({xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f})" if xyz is not None else "None"

        self.get_logger().info(
            f"[STATS] sent/s={sent_s}  skip/s={skip_s}  interp/s={interp_s}  "
            f"drop_no_target/s={drop_nt_s}  drop_timeout/s={drop_to_s}  xyz={xyz_str}"
        )


def main():
    rclpy.init()
    node = ArmTargetToServoJ()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.enable_and_move_base()

    print(f"[ServoJ] {SEND_HZ:.0f}Hz  t={SERVO_T:.4f}s  INTERP_ALPHA={INTERP_ALPHA}")
    print("Ctrl+C 로 종료")

    try:
        while spin_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[QUIT]")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
