#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_controller_movJ.py
=========================
MovJ 방식: 관절 최단경로 이동으로 팔 추종

인터페이스 (dobot_msgs_v4/srv/MovJ)
  Request : bool mode / float64 a,b,c,d,e,f / string[] param_value
  Response: string robot_return / int32 res

★ mode 의미
  False = 사용자 좌표계 (일반적으로 False 사용)
  True  = 관절 좌표계

★ MovL과 차이
  MovL: 직선 경로 보장, 속도 느림
  MovJ: 관절 최단경로 (호 궤적), 속도 빠름
"""

import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PointStamped
from dobot_msgs_v4.srv import EnableRobot, MovJ

# =========================
# 토픽 / 서비스
# =========================
TOPIC_TARGET = "/arm_target_mm"
SRV_ENABLE   = "/dobot_bringup_ros2/srv/EnableRobot"
SRV_MOVJ     = "/dobot_bringup_ros2/srv/MovJ"

# =========================
# 로봇 BASE 자세
# =========================
BASE_TCP_MM  = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
BASE_RPY_DEG = np.array([ 178.7294,   -1.9186,  -96.5768], dtype=np.float64)

# =========================
# MovJ 설정
# =========================
MOVE_THRESHOLD_MM = 20.0   # 이 거리 이상 움직였을 때만 새 명령 전송
MOVE_SPEED        = 40     # MovJ는 MovL보다 빠르게 설정해도 안정적
MOVE_ACC          = 40
MODE              = False  # False=사용자 좌표계, True=관절 좌표계

SEND_HZ           = 10.0
DT                = 1.0 / SEND_HZ

PRINT_EVERY_SEC   = 1.0
HOLD_TIMEOUT_SEC  = 1.0


class ArmTargetToMovJ(Node):

    def __init__(self):
        super().__init__("arm_target_to_movj")

        self._lock          = threading.Lock()
        self.latest_xyz     = None
        self.last_rx_time   = 0.0
        self._last_future   = None
        self._last_sent_xyz = None

        self.sent               = 0
        self.skipped_busy       = 0
        self.skipped_threshold  = 0
        self.dropped_no_target  = 0
        self.dropped_timeout    = 0

        self._sent_last     = 0
        self._skip_last     = 0
        self._skip_thr_last = 0
        self._drop_nt_last  = 0
        self._drop_to_last  = 0
        self.last_print_t   = time.perf_counter()

        self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
        self.movj_cli   = self.create_client(MovJ, SRV_MOVJ)

        self.get_logger().info("Dobot 서비스 대기 중...")
        self.enable_cli.wait_for_service()
        self.movj_cli.wait_for_service()
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
            f"MovJ: {SEND_HZ:.0f}Hz  threshold={MOVE_THRESHOLD_MM:.0f}mm  "
            f"speed={MOVE_SPEED}  acc={MOVE_ACC}  mode={MODE}"
        )

    def _wait_future(self, fut, timeout=15.0):
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

        req = MovJ.Request()
        req.mode = MODE
        req.a = float(BASE_TCP_MM[0])
        req.b = float(BASE_TCP_MM[1])
        req.c = float(BASE_TCP_MM[2])
        req.d = float(BASE_RPY_DEG[0])
        req.e = float(BASE_RPY_DEG[1])
        req.f = float(BASE_RPY_DEG[2])
        req.param_value = [f"speed={MOVE_SPEED}", f"acc={MOVE_ACC}"]

        fut2 = self.movj_cli.call_async(req)
        if not self._wait_future(fut2):
            return
        try:
            res = fut2.result()
            # ★ 응답 필드: res (0=성공)
            if res is not None and res.res != 0:
                self.get_logger().error(f"BASE 이동 에러: res={res.res}")
            else:
                self.get_logger().info("BASE 자세 완료.")
        except Exception as e:
            self.get_logger().warn(f"BASE 이동 결과 확인 실패: {e}")

        self._last_sent_xyz = BASE_TCP_MM.copy()
        time.sleep(0.5)

    def on_target(self, msg: PointStamped):
        with self._lock:
            self.latest_xyz   = np.array(
                [msg.point.x, msg.point.y, msg.point.z], dtype=np.float64
            )
            self.last_rx_time = time.perf_counter()

    def _build_req(self, xyz: np.ndarray) -> MovJ.Request:
        req = MovJ.Request()
        req.mode = MODE
        req.a = float(xyz[0])
        req.b = float(xyz[1])
        req.c = float(xyz[2])
        req.d = float(BASE_RPY_DEG[0])
        req.e = float(BASE_RPY_DEG[1])
        req.f = float(BASE_RPY_DEG[2])
        req.param_value = [f"speed={MOVE_SPEED}", f"acc={MOVE_ACC}"]
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

        # 이전 명령 완료 대기
        if self._last_future is not None and not self._last_future.done():
            self.skipped_busy += 1
            self._print_stats(now)
            return

        # 거리 임계값
        if self._last_sent_xyz is not None:
            dist = float(np.linalg.norm(latest_xyz - self._last_sent_xyz))
            if dist < MOVE_THRESHOLD_MM:
                self.skipped_threshold += 1
                self._print_stats(now)
                return

        # 이전 응답 에러 체크
        if self._last_future is not None:
            try:
                res = self._last_future.result()
                if res is not None and res.res != 0:
                    self.get_logger().warn(f"MovJ 에러: res={res.res}")
            except Exception as e:
                self.get_logger().error(f"MovJ 결과 확인 실패: {e}")

        self._last_future   = self.movj_cli.call_async(self._build_req(latest_xyz))
        self._last_sent_xyz = latest_xyz.copy()
        self.sent += 1
        self._print_stats(now)

    def _print_stats(self, now):
        if now - self.last_print_t < PRINT_EVERY_SEC:
            return

        sent_s     = self.sent              - self._sent_last
        skip_s     = self.skipped_busy      - self._skip_last
        skip_thr_s = self.skipped_threshold - self._skip_thr_last
        drop_nt_s  = self.dropped_no_target - self._drop_nt_last
        drop_to_s  = self.dropped_timeout   - self._drop_to_last

        self._sent_last     = self.sent
        self._skip_last     = self.skipped_busy
        self._skip_thr_last = self.skipped_threshold
        self._drop_nt_last  = self.dropped_no_target
        self._drop_to_last  = self.dropped_timeout
        self.last_print_t   = now

        with self._lock:
            xyz = self.latest_xyz
        xyz_str = f"({xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f})" if xyz is not None else "None"

        self.get_logger().info(
            f"[STATS] sent/s={sent_s}  skip_busy/s={skip_s}  "
            f"skip_threshold/s={skip_thr_s}  "
            f"drop_no_target/s={drop_nt_s}  drop_timeout/s={drop_to_s}  xyz={xyz_str}"
        )


def main():
    rclpy.init()
    node = ArmTargetToMovJ()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.enable_and_move_base()

    print(f"[MovJ] {SEND_HZ:.0f}Hz  threshold={MOVE_THRESHOLD_MM:.0f}mm  "
          f"speed={MOVE_SPEED}  acc={MOVE_ACC}")
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
