#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_controller_servoP.py
===========================
수정 내역:
  [FIX 1] _cleanup_pending: 앞에서만 제거 → 전체 순회로 변경
           (중간에 완료된 future를 놓치던 버그 수정)
  [FIX 2] step advance 시 _pending에 무한 append되던 버그 수정
           (step 전송 future는 별도 관리, pending 카운트에 미포함)
  [FIX 3] _rtt_samples 접근 시 lock 추가 (스레드 안전)

구조:
  카메라 ──[Topic 30Hz]──▶ robot_controller ──[Service fire-and-forget]──▶ CCBox ──▶ Dobot
  카메라 → robot_controller: 토픽 (단방향, 응답 없음)
  robot_controller → CCBox:  서비스 (도봇 드라이버 제약)
                              단, fire-and-forget으로 토픽처럼 동작

개선:
  Fire-and-Forget: 응답 대기 없이 즉시 다음 명령 전송 가능
  Lookahead Step:  pending 가득 찰 때 목표 방향으로 조금씩 전진
  RTT 측정:        실제 ServoP 왕복 시간 10회마다 출력
"""

import time
import threading
import collections

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PointStamped
from dobot_msgs_v4.srv import EnableRobot, ServoP

# =========================================================
# 토픽 / 서비스
# =========================================================
TOPIC_TARGET = "/arm_target_mm"
SRV_ENABLE   = "/dobot_bringup_ros2/srv/EnableRobot"
SRV_SERVO    = "/dobot_bringup_ros2/srv/ServoP"

# =========================================================
# 로봇 BASE 자세
# =========================================================
BASE_TCP_MM  = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
BASE_RPY_DEG = np.array([ 178.7294,   -1.9186,  -96.5768], dtype=np.float64)
BASE_MOVE_T  = 5.0

# =========================================================
# ServoP 스트림 설정
# =========================================================
FIXED_RPY_DEG    = BASE_RPY_DEG.copy()
SEND_HZ          = 30.0
DT               = 1.0 / SEND_HZ   # 0.0333s
SERVO_T          = DT               # CCBox 보간 시간 = 전송 주기

PRINT_EVERY_SEC  = 1.0
HOLD_TIMEOUT_SEC = 1.0   # 0.5 → 1.0 (MediaPipe 처리 지연 여유)

# =========================================================
# Fire-and-Forget 설정
#
# MAX_PENDING: 동시 처리 중인 ServoP future 최대 수
#   2 = 권장
#   3 = CCBox가 느릴 때
#   1 = 엄격 (기존 방식과 유사)
# =========================================================
MAX_PENDING = 2

# =========================================================
# Lookahead Step Advance 설정
#
# pending >= MAX_PENDING 일 때 동작
# last_sent → latest 방향으로 STEP_RATIO 비율 전진
#
# STEP_RATIO: 0.3 권장 / 0.5 빠름 / 0.2 보수적
# MAX_STEP_MM: 한 사이클 최대 이동 거리 (mm)
# =========================================================
STEP_RATIO  = 0.3
MAX_STEP_MM = 15.0


class ArmTargetToServoP(Node):

    def __init__(self):
        super().__init__("arm_target_to_servop")

        self._lock          = threading.Lock()
        self.latest_xyz     = None
        self.last_rx_time   = 0.0
        self._last_sent_xyz = None

        # [FIX 1,2] 정상 전송 future만 pending으로 관리
        # step advance future는 별도로 관리 (pending 카운트에 미포함)
        self._pending: collections.deque = collections.deque()
        self._step_futures: list         = []   # step advance future 목록

        # [FIX 3] RTT 측정 (lock으로 보호)
        self._rtt_lock    = threading.Lock()
        self._rtt_samples: list = []

        # 통계
        self.stat_sent    = 0
        self.stat_stepped = 0
        self.stat_drop_nt = 0
        self.stat_drop_to = 0

        self._s_sent    = 0
        self._s_stepped = 0
        self._s_drop_nt = 0
        self._s_drop_to = 0
        self.last_print = time.perf_counter()

        # 서비스 클라이언트
        self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
        self.servo_cli  = self.create_client(ServoP,      SRV_SERVO)

        self.get_logger().info("Dobot 서비스 대기 중...")
        self.enable_cli.wait_for_service()
        self.servo_cli.wait_for_service()
        self.get_logger().info("서비스 연결 완료.")

        # 토픽 구독 (카메라 → robot_controller, 단방향)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            PointStamped, TOPIC_TARGET, self._on_target, qos
        )
        self.get_logger().info(
            f"[{TOPIC_TARGET}] 토픽 구독  QoS=BEST_EFFORT/KEEP_LAST(1)"
        )

        # 고정 주기 타이머
        self.timer = self.create_timer(DT, self._on_timer)
        self.get_logger().info(
            f"Fire-and-Forget ServoP  {SEND_HZ:.0f}Hz  t={SERVO_T:.4f}s  "
            f"MAX_PENDING={MAX_PENDING}  "
            f"STEP={STEP_RATIO}  MAX_STEP={MAX_STEP_MM}mm"
        )

    # ── future 폴링 대기 (초기화 전용) ───────────────────────
    def _wait_future(self, fut, timeout=10.0):
        start = time.perf_counter()
        while not fut.done():
            if time.perf_counter() - start > timeout:
                self.get_logger().error("Future 타임아웃")
                return False
            time.sleep(0.01)
        return True

    # ── 초기화: 활성화 + BASE 자세 이동 ──────────────────────
    def enable_and_move_base(self):
        fut = self.enable_cli.call_async(EnableRobot.Request())
        if not self._wait_future(fut):
            return
        self.get_logger().info("로봇 활성화 완료. BASE 자세로 이동 중...")

        req = ServoP.Request()
        req.a = float(BASE_TCP_MM[0])
        req.b = float(BASE_TCP_MM[1])
        req.c = float(BASE_TCP_MM[2])
        req.d = float(BASE_RPY_DEG[0])
        req.e = float(BASE_RPY_DEG[1])
        req.f = float(BASE_RPY_DEG[2])
        req.param_value = [f"t={BASE_MOVE_T:.4f}"]

        fut2 = self.servo_cli.call_async(req)
        if not self._wait_future(fut2):
            return
        try:
            res = fut2.result()
            if res and hasattr(res, "error_id") and res.error_id != 0:
                self.get_logger().error(f"BASE 이동 에러: {res.error_id}")
            else:
                self.get_logger().info("BASE 자세 완료.")
        except Exception as e:
            self.get_logger().warn(f"BASE 이동 결과 확인 실패: {e}")

        self._last_sent_xyz = BASE_TCP_MM.copy()
        time.sleep(0.3)

    # ── 토픽 수신 콜백 ────────────────────────────────────────
    def _on_target(self, msg: PointStamped):
        with self._lock:
            self.latest_xyz   = np.array(
                [msg.point.x, msg.point.y, msg.point.z], dtype=np.float64
            )
            self.last_rx_time = time.perf_counter()

    # ── ServoP 요청 생성 ──────────────────────────────────────
    def _build_req(self, xyz: np.ndarray) -> ServoP.Request:
        req = ServoP.Request()
        req.a = float(xyz[0])
        req.b = float(xyz[1])
        req.c = float(xyz[2])
        req.d = float(FIXED_RPY_DEG[0])
        req.e = float(FIXED_RPY_DEG[1])
        req.f = float(FIXED_RPY_DEG[2])
        req.param_value = [f"t={SERVO_T:.4f}"]
        return req

    # ── ServoP 전송 공통 로직 ─────────────────────────────────
    def _make_done_cb(self, t_send: float):
        """RTT 측정 + 에러 체크 콜백 생성"""
        def _done_cb(f):
            rtt_ms = (time.perf_counter() - t_send) * 1000
            # [FIX 3] lock으로 보호
            with self._rtt_lock:
                self._rtt_samples.append(rtt_ms)
                if len(self._rtt_samples) >= 10:
                    arr = list(self._rtt_samples)
                    self._rtt_samples.clear()
                else:
                    arr = None
            if arr is not None:
                self.get_logger().info(
                    f"[RTT] avg={np.mean(arr):.1f}ms  "
                    f"min={np.min(arr):.1f}ms  "
                    f"max={np.max(arr):.1f}ms"
                )
            try:
                res = f.result()
                if res and hasattr(res, "error_id") and res.error_id != 0:
                    self.get_logger().warn(f"ServoP 에러: {res.error_id}")
            except Exception as e:
                self.get_logger().error(f"ServoP 결과 확인 실패: {e}")
        return _done_cb

    # ── 정상 전송 (pending 관리) ──────────────────────────────
    def _fire_normal(self, xyz: np.ndarray, t_send: float):
        fut = self.servo_cli.call_async(self._build_req(xyz))
        fut.add_done_callback(self._make_done_cb(t_send))
        self._pending.append(fut)

    # ── [FIX 2] step advance 전송 (pending 카운트 미포함) ─────
    def _fire_step(self, xyz: np.ndarray, t_send: float):
        fut = self.servo_cli.call_async(self._build_req(xyz))
        fut.add_done_callback(self._make_done_cb(t_send))
        # step future는 별도 목록에만 추가 (pending 카운트에 영향 없음)
        self._step_futures.append(fut)

    # ── [FIX 1] pending 정리 (전체 순회) ─────────────────────
    def _cleanup_pending(self):
        # 앞에서부터만 제거하던 버그 수정
        # → 완료된 것은 위치 상관없이 모두 제거
        remaining = collections.deque()
        for f in self._pending:
            if not f.done():
                remaining.append(f)
        self._pending = remaining

        # step futures도 완료된 것 정리
        self._step_futures = [f for f in self._step_futures if not f.done()]

    # ── 고정 주기 타이머 (핵심 루프) ─────────────────────────
    def _on_timer(self):
        now = time.perf_counter()

        with self._lock:
            latest_xyz   = self.latest_xyz.copy() if self.latest_xyz is not None else None
            last_rx_time = self.last_rx_time

        # target 없음
        if latest_xyz is None:
            self.stat_drop_nt += 1
            self._print_stats(now)
            return

        # 토픽 타임아웃
        if (now - last_rx_time) > HOLD_TIMEOUT_SEC:
            self.stat_drop_to += 1
            self._print_stats(now)
            return

        # 완료된 future 정리
        self._cleanup_pending()
        pending_count = len(self._pending)

        # ── 경우 1: pending 여유 있음 → 최신 target 즉시 전송 ──
        if pending_count < MAX_PENDING:
            send_xyz            = latest_xyz
            self._last_sent_xyz = send_xyz.copy()
            self._fire_normal(send_xyz, now)
            self.stat_sent += 1

        # ── 경우 2: pending 가득 참 → step advance ─────────────
        # [FIX 2] _fire_step 사용 → pending 카운트 증가 없음
        else:
            if self._last_sent_xyz is not None:
                direction = latest_xyz - self._last_sent_xyz
                dist      = float(np.linalg.norm(direction))

                if dist > 0.5:
                    step_dist = min(dist * STEP_RATIO, MAX_STEP_MM)
                    step      = direction / dist * step_dist
                    send_xyz  = self._last_sent_xyz + step

                    self._last_sent_xyz = send_xyz.copy()
                    self._fire_step(send_xyz, now)
                    self.stat_stepped += 1

        self._print_stats(now)

    # ── 통계 출력 ─────────────────────────────────────────────
    def _print_stats(self, now: float):
        if now - self.last_print < PRINT_EVERY_SEC:
            return

        sent_s    = self.stat_sent    - self._s_sent
        step_s    = self.stat_stepped - self._s_stepped
        drop_nt_s = self.stat_drop_nt - self._s_drop_nt
        drop_to_s = self.stat_drop_to - self._s_drop_to

        self._s_sent    = self.stat_sent
        self._s_stepped = self.stat_stepped
        self._s_drop_nt = self.stat_drop_nt
        self._s_drop_to = self.stat_drop_to
        self.last_print = now

        with self._lock:
            xyz = self.latest_xyz
        xyz_str = (
            f"({xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f})"
            if xyz is not None else "None"
        )

        self.get_logger().info(
            f"[STATS] sent/s={sent_s}  step/s={step_s}  "
            f"pending={len(self._pending)}  "
            f"drop_no_target/s={drop_nt_s}  drop_timeout/s={drop_to_s}  "
            f"xyz={xyz_str}"
        )


# =========================================================
# main
# =========================================================

def main():
    rclpy.init()
    node = ArmTargetToServoP()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.enable_and_move_base()

    print("=" * 55)
    print(f" Fire-and-Forget ServoP  {SEND_HZ:.0f}Hz  t={SERVO_T:.4f}s")
    print(f" MAX_PENDING={MAX_PENDING}  "
          f"STEP={STEP_RATIO}  MAX_STEP={MAX_STEP_MM}mm")
    print("=" * 55)
    print(" sent/s   : 정상 전송 (최신 target)")
    print(" step/s   : step advance (CCBox 처리 지연 시)")
    print(" pending  : 현재 처리 중인 정상 future 수")
    print(" [RTT]    : 실제 ServoP 왕복 시간 (10회마다)")
    print("=" * 55)
    print(" 카메라 토픽 대기 중...  Ctrl+C 로 종료")

    try:
        while spin_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[QUIT] Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("robot_controller 종료.")


if __name__ == "__main__":
    main()