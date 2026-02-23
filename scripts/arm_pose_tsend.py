# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import time
# import csv
# from datetime import datetime, timezone, timedelta

# import cv2
# import numpy as np
# import mediapipe as mp
# import pyrealsense2 as rs

# import rclpy
# from rclpy.node import Node
# from dobot_msgs_v4.srv import EnableRobot, ServoP

# # =========================
# # 0) 안전 스위치 (기본 OFF)
# # =========================
# ENABLE_FOLLOW_MOVE = False   # True로 바꿔야만 실제 로봇 추종 시작
# ENABLE_BASE_MOVE = True      # 항상 BASE로 이동 (요구사항)

# # =========================
# # 1) 로봇 기본자세 (사용자 제공)
# # =========================
# BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
# BASE_RPY_DEG = np.array([178.7294, -1.9186, -96.5768], dtype=np.float64)
# BASE_MOVE_T = 3.0

# # =========================
# # 2) Dobot 서비스 경로
# # =========================
# SRV_ENABLE = "/dobot_bringup_ros2/srv/EnableRobot"
# SRV_SERVO  = "/dobot_bringup_ros2/srv/ServoP"

# # =========================
# # 3) RealSense/MediaPipe
# # =========================
# OUTPUT_CSV = "right_arm_keypoints.csv"

# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 720
# FPS = 30
# DEPTH_AVG_KERNEL = 3

# SHOW_FPS_OVERLAY = True
# FPS_ALPHA = 0.90

# mp_pose = mp.solutions.pose
# RIGHT_SHOULDER = 12
# RIGHT_ELBOW = 14
# RIGHT_WRIST = 16

# # =========================
# # 4) 안정화(3초) 조건
# # =========================
# STABLE_TOL_MM = 10.0
# STABLE_NOISE_BAND_MM = 10.0
# STABLE_SECONDS = 3.0
# MAX_JUMP_MM = 30.0

# # =========================
# # 5) follow 출력/전송 제어
# # =========================
# PRINT_GATE_MM = 40.0
# FOLLOW_DEADBAND_MM = 3.0

# # 데이터 송신
# SEND_HZ = 20.0               # 송신 빈도(Hz). 10~30 권장

# # =========================
# # 6) 팔 -> 로봇 목표 매핑
# # =========================
# Kx = 0.8
# Ky = 0.8
# Kz = 0.8
# BEND_GAIN = 0.35

# # =========================
# # 7) 부드러움/안전 제한
# # =========================
# MAX_STEP_MM = 2.0            # 실제 이동 켤 때는 2~3 추천
# # 사람 팔의 데이터 최대 억제 범위
# # 이거를 지우고 새롭게 짜보기
# ALPHA_POS = 0.25
# DEADBAND_MM = 1.0

# # 로봇이 처리하는 속도 
# FOLLOW_T = 0.1              # ServoP t= (스트리밍용 0.08~0.20)

# WORKSPACE_MIN = np.array([-600.0, -500.0, 150.0], dtype=np.float64)
# WORKSPACE_MAX = np.array([-270.0,  550.0, 600.0], dtype=np.float64)

# # =========================
# # 스트리밍 안정성/안전 정책
# # =========================
# RX_TIMEOUT_SEC = 0.25         # 이 시간 이상 포즈/뎁스 갱신 끊기면 soft-stop
# SOFTSTOP_ALPHA = 0.15         # BASE로 복귀 EMA 속도(작을수록 천천히)
# SOFTSTOP_T_MULT = 3.0         # 끊겼을 때 t를 늘려 감속 효과

# # =========================
# # 유틸
# # =========================
# def norm3(v) -> float:
#     return float(np.linalg.norm(v))

# def ema(prev, cur, alpha):
#     return (1.0 - alpha) * prev + alpha * cur

# def apply_deadband_vec(v_mm: np.ndarray, band_mm: float) -> np.ndarray:
#     out = v_mm.copy()
#     for i in range(3):
#         a = abs(out[i])
#         if a <= band_mm:
#             out[i] = 0.0
#         else:
#             out[i] = np.sign(out[i]) * (a - band_mm)
#     return out

# def clamp_xyz(xyz: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
#     return np.minimum(np.maximum(xyz, lo), hi)

# def safe_depth_deproject(depth_frame, intr, x_px, y_px, h, w, ksize=3):
#     x_px = int(np.clip(x_px, 0, w - 1))
#     y_px = int(np.clip(y_px, 0, h - 1))

#     k = ksize // 2
#     depths = []
#     for yy in range(max(0, y_px - k), min(h, y_px + k + 1)):
#         for xx in range(max(0, x_px - k), min(w, x_px + k + 1)):
#             d = depth_frame.get_distance(xx, yy)
#             if d > 0:
#                 depths.append(d)

#     depth_m = float(np.mean(depths)) if depths else 0.0
#     if depth_m <= 0:
#         return None, 0.0

#     X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [x_px, y_px], depth_m)
#     return (np.array([X, Y, Z], dtype=np.float64) * 1000.0), depth_m

# # =========================
# # Dobot Bridge (REALTIME STREAM)
# # =========================
# class DobotBridge(Node):
#     def __init__(self):
#         # 중요: node_name 반드시 필요 (여기 없으면 네가 본 에러 그대로 남)
#         super().__init__("dobot_mp_pose_follow_stream_realtime")

#         self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
#         self.servo_p_cli = self.create_client(ServoP, SRV_SERVO)

#         self.get_logger().info("Dobot 서비스 대기 중...")
#         self.enable_cli.wait_for_service()
#         self.servo_p_cli.wait_for_service()
#         self.get_logger().info("Dobot 서비스 연결 완료")

#         # EnableRobot은 1회성이므로 블로킹 OK
#         req = EnableRobot.Request()
#         fut = self.enable_cli.call_async(req)
#         rclpy.spin_until_future_complete(self, fut)
#         self.get_logger().info("로봇 활성화 완료")

#         # --- streaming state ---
#         self.move_enabled = bool(ENABLE_FOLLOW_MOVE)
#         self.follow_active = False

#         self.latest_cmd = BASE_TCP_MM.copy()
#         self.latest_t = float(FOLLOW_T)

#         self.last_pose_ok_t = time.perf_counter()

#         self._last_future = None

#         self.send_period = 1.0 / max(1e-6, float(SEND_HZ))
#         self._timer = self.create_timer(self.send_period, self._on_send_timer)

#     def set_move_enabled(self, enabled: bool):
#         self.move_enabled = bool(enabled)

#     def set_follow_active(self, active: bool):
#         self.follow_active = bool(active)

#     def update_cmd(self, xyz_mm: np.ndarray, t: float, pose_ok: bool):
#         self.latest_cmd = np.array(xyz_mm, dtype=np.float64)
#         self.latest_t = float(t)
#         if pose_ok:
#             self.last_pose_ok_t = time.perf_counter()

#     def move_to_blocking(self, x, y, z, rx, ry, rz, t):
#         # BASE 이동 등 1회성은 블로킹으로 OK
#         req = ServoP.Request()
#         req.a = float(x);  req.b = float(y);  req.c = float(z)
#         req.d = float(rx); req.e = float(ry); req.f = float(rz)
#         req.param_value = [f"t={float(t)}"]
#         fut = self.servo_p_cli.call_async(req)
#         rclpy.spin_until_future_complete(self, fut)

#     def _servo_stream_async(self, x, y, z, rx, ry, rz, t):
#         # 실시간 스트리밍: 절대 기다리지 않음(블로킹 제거)
#         req = ServoP.Request()
#         req.a = float(x);  req.b = float(y);  req.c = float(z)
#         req.d = float(rx); req.e = float(ry); req.f = float(rz)
#         req.param_value = [f"t={float(t)}"]
#         self._last_future = self.servo_p_cli.call_async(req)

#     def _on_send_timer(self):
#         # 타이머가 고정 주기로 계속 보내는 구조
#         if not (self.follow_active and self.move_enabled):
#             return

#         now = time.perf_counter()
#         pose_gap = now - self.last_pose_ok_t

#         if pose_gap > RX_TIMEOUT_SEC:
#             # 포즈/뎁스 끊김 -> BASE로 천천히 복귀(soft stop)하면서 계속 송신
#             self.latest_cmd = ema(self.latest_cmd, BASE_TCP_MM, SOFTSTOP_ALPHA)
#             t_send = float(self.latest_t * SOFTSTOP_T_MULT)
#         else:
#             t_send = float(self.latest_t)

#         cmd = self.latest_cmd
#         self._servo_stream_async(
#             cmd[0], cmd[1], cmd[2],
#             BASE_RPY_DEG[0], BASE_RPY_DEG[1], BASE_RPY_DEG[2],
#             t_send
#         )

# # =========================
# # main
# # =========================
# def main():
#     rclpy.init()
#     node = DobotBridge()

#     # A) BASE 이동은 항상
#     if ENABLE_BASE_MOVE:
#         print("▶ Moving robot to BASE pose...")
#         node.move_to_blocking(
#             BASE_TCP_MM[0], BASE_TCP_MM[1], BASE_TCP_MM[2],
#             BASE_RPY_DEG[0], BASE_RPY_DEG[1], BASE_RPY_DEG[2],
#             BASE_MOVE_T
#         )
#         time.sleep(0.4)
#         print("BASE pose reached.")

#     # B) 카메라 시작
#     pipeline = rs.pipeline()
#     cfg = rs.config()
#     cfg.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
#     cfg.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
#     profile = pipeline.start(cfg)

#     align = rs.align(rs.stream.color)
#     intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

#     print("RealSense started.")
#     print(f"조건: {STABLE_SECONDS:.1f}s 안정 → AUTO_ZERO 후 FOLLOW")
#     print(f"FOLLOW 송신: {SEND_HZ:.1f} Hz (Timer), t={FOLLOW_T:.2f} (ENABLE_FOLLOW_MOVE={ENABLE_FOLLOW_MOVE})")
#     print(f"WORKSPACE: min={WORKSPACE_MIN.tolist()}, max={WORKSPACE_MAX.tolist()}")
#     print("Keys: r=RESET, m=toggle move(ON/OFF), q=quit")

#     # CSV
#     header = ["frame_idx","timestamp_iso","stable_s","follow_active","dr_norm","cmdX","cmdY","cmdZ","pose_ok","fps"]
#     csv_file = open(OUTPUT_CSV, "w", newline="")
#     csv_writer = csv.writer(csv_file)
#     csv_writer.writerow(header)

#     KST = timezone(timedelta(hours=9))

#     pose = mp_pose.Pose(
#         static_image_mode=False,
#         model_complexity=1,
#         min_detection_confidence=0.5,
#         min_tracking_confidence=0.5
#     )

#     prev_time = time.perf_counter()
#     fps_smooth = 0.0
#     frame_idx = 0

#     stable_start_t = None
#     stable_s = 0.0

#     r0 = None
#     bend_ratio0 = None
#     follow_active = False

#     prev_r = None
#     prev_dr = None

#     filt_target = BASE_TCP_MM.copy()
#     prev_cmd = BASE_TCP_MM.copy()
#     last_print_cmd = BASE_TCP_MM.copy()

#     move_enabled = ENABLE_FOLLOW_MOVE
#     node.set_move_enabled(move_enabled)

#     try:
#         while True:
#             # 타이머 송신 포함 ROS 콜백 처리
#             rclpy.spin_once(node, timeout_sec=0.0)

#             now = time.perf_counter()
#             dt = now - prev_time
#             prev_time = now
#             if dt > 0:
#                 fps = 1.0 / dt
#                 fps_smooth = FPS_ALPHA * fps_smooth + (1 - FPS_ALPHA) * fps

#             frames = pipeline.wait_for_frames()
#             frames = align.process(frames)
#             depth_frame = frames.get_depth_frame()
#             color_frame = frames.get_color_frame()
#             if not depth_frame or not color_frame:
#                 continue

#             img = np.asanyarray(color_frame.get_data())
#             h, w = img.shape[:2]

#             rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#             results = pose.process(rgb)

#             timestamp = datetime.now(KST).isoformat()

#             if SHOW_FPS_OVERLAY:
#                 cv2.putText(img, f"FPS: {fps_smooth:.1f}", (10, 30),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

#             dr_norm = 0.0
#             pose_ok = False

#             if not results.pose_landmarks:
#                 cv2.putText(img, "No pose landmarks (soft-stop)", (10, 60),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
#             else:
#                 def lm_px(idx):
#                     lm = results.pose_landmarks.landmark[idx]
#                     x_px = int(np.clip(lm.x * w, 0, w - 1))
#                     y_px = int(np.clip(lm.y * h, 0, h - 1))
#                     return x_px, y_px

#                 sh_px = lm_px(RIGHT_SHOULDER)
#                 el_px = lm_px(RIGHT_ELBOW)
#                 wr_px = lm_px(RIGHT_WRIST)

#                 sh_mm, _ = safe_depth_deproject(depth_frame, intr, sh_px[0], sh_px[1], h, w, DEPTH_AVG_KERNEL)
#                 el_mm, _ = safe_depth_deproject(depth_frame, intr, el_px[0], el_px[1], h, w, DEPTH_AVG_KERNEL)
#                 wr_mm, _ = safe_depth_deproject(depth_frame, intr, wr_px[0], wr_px[1], h, w, DEPTH_AVG_KERNEL)

#                 if sh_mm is None or el_mm is None or wr_mm is None:
#                     cv2.putText(img, "Depth invalid (soft-stop)", (10, 60),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
#                 else:
#                     pose_ok = True

#                     for (name, px) in [("shoulder", sh_px), ("elbow", el_px), ("wrist", wr_px)]:
#                         cv2.circle(img, (px[0], px[1]), 6, (0,255,0), -1)
#                         cv2.putText(img, name, (px[0]+6, px[1]-6),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
#                     cv2.line(img, sh_px, el_px, (0,255,0), 3)
#                     cv2.line(img, el_px, wr_px, (0,255,0), 3)

#                     se = norm3(el_mm - sh_mm) + 1e-6
#                     ew = norm3(wr_mm - el_mm) + 1e-6
#                     bend_ratio = ew / se

#                     r = (wr_mm - sh_mm)

#                     # ===== 안정화(3초) → AUTO_ZERO =====
#                     if r0 is None:
#                         if prev_r is None:
#                             prev_r = r.copy()
#                             stable_start_t = None
#                             stable_s = 0.0
#                         else:
#                             r_delta = r - prev_r
#                             r_delta = apply_deadband_vec(r_delta, STABLE_NOISE_BAND_MM)
#                             if norm3(r_delta) < STABLE_NOISE_BAND_MM:
#                                 r_delta[:] = 0.0

#                             jn = norm3(r_delta)
#                             if jn > MAX_JUMP_MM:
#                                 r_delta = r_delta * (MAX_JUMP_MM / jn)

#                             if norm3(r_delta) <= STABLE_TOL_MM:
#                                 if stable_start_t is None:
#                                     stable_start_t = time.perf_counter()
#                                 stable_s = time.perf_counter() - stable_start_t
#                                 if stable_s >= STABLE_SECONDS:
#                                     r0 = r.copy()
#                                     bend_ratio0 = bend_ratio
#                                     follow_active = True

#                                     prev_dr = None
#                                     filt_target = BASE_TCP_MM.copy()
#                                     prev_cmd = BASE_TCP_MM.copy()
#                                     last_print_cmd = BASE_TCP_MM.copy()

#                                     node.set_follow_active(True)

#                                     print(f"[AUTO_ZERO] stable {stable_s:.2f}s → follow_active=1")
#                             else:
#                                 stable_start_t = None
#                                 stable_s = 0.0

#                             prev_r = r.copy()

#                         cv2.putText(img, f"STABLE_WAIT {stable_s:.1f}/{STABLE_SECONDS:.1f}s", (10, 60),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

#                     # ===== FOLLOW =====
#                     else:
#                         raw_dr = (r - r0)
#                         raw_dr = apply_deadband_vec(raw_dr, FOLLOW_DEADBAND_MM)
#                         if norm3(raw_dr) < FOLLOW_DEADBAND_MM:
#                             raw_dr[:] = 0.0

#                         if prev_dr is None:
#                             dr = raw_dr
#                         else:
#                             jump = raw_dr - prev_dr
#                             jn = norm3(jump)
#                             if jn > MAX_JUMP_MM:
#                                 jump = jump * (MAX_JUMP_MM / jn)
#                             dr = prev_dr + jump
#                         prev_dr = dr.copy()

#                         dr_norm = norm3(dr)

#                         dx =  Kx * dr[2]
#                         dy = -Ky * dr[0]   # left=+Y
#                         dz = -Kz * dr[1]

#                         bend_delta = (bend_ratio - bend_ratio0) if bend_ratio0 is not None else 0.0
#                         depth_scale = 1.0 + BEND_GAIN * np.clip(bend_delta, -0.8, 0.8)
#                         depth_scale = float(np.clip(depth_scale, 0.5, 1.5))
#                         dx *= depth_scale

#                         raw_target = BASE_TCP_MM + np.array([dx, dy, dz], dtype=np.float64)
#                         raw_target = clamp_xyz(raw_target, WORKSPACE_MIN, WORKSPACE_MAX)

#                         filt_target = ema(filt_target, raw_target, ALPHA_POS)
#                         delta = filt_target - prev_cmd
#                         dist = norm3(delta)
#                         if dist > MAX_STEP_MM:
#                             delta = delta * (MAX_STEP_MM / dist)
#                         cmd = prev_cmd + delta

#                         if norm3(cmd - prev_cmd) < DEADBAND_MM:
#                             cmd = prev_cmd.copy()
#                         prev_cmd = cmd.copy()

#                         if norm3(prev_cmd - last_print_cmd) >= PRINT_GATE_MM:
#                             print(f"[{frame_idx}] cmd(mm) X:{prev_cmd[0]:.1f} Y:{prev_cmd[1]:.1f} Z:{prev_cmd[2]:.1f} | |dr|={dr_norm:.1f}")
#                             last_print_cmd = prev_cmd.copy()

#                         # 여기서는 "보내지 말고", 최신 cmd만 업데이트
#                         if follow_active:
#                             node.update_cmd(prev_cmd, FOLLOW_T, pose_ok=True)

#                         cv2.putText(img, f"FOLLOW:1 move={int(move_enabled)}", (10, 60),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if move_enabled else (0,255,255), 2)
#                         cv2.putText(img, f"cmd X:{prev_cmd[0]:.1f} Y:{prev_cmd[1]:.1f} Z:{prev_cmd[2]:.1f}", (10, 90),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

#             # pose_ok False라도 타이머가 soft-stop할 수 있게 갱신
#             if follow_active and (not pose_ok):
#                 node.update_cmd(node.latest_cmd, FOLLOW_T, pose_ok=False)

#             csv_writer.writerow([
#                 frame_idx, timestamp,
#                 f"{stable_s:.3f}", int(follow_active),
#                 f"{dr_norm:.3f}",
#                 f"{prev_cmd[0]:.3f}", f"{prev_cmd[1]:.3f}", f"{prev_cmd[2]:.3f}",
#                 int(pose_ok),
#                 f"{fps_smooth:.2f}"
#             ])

#             cv2.imshow("Stable 3s -> Follow (REALTIME stream)", img)
#             frame_idx += 1

#             key = cv2.waitKey(1) & 0xFF
#             if key == ord("q"):
#                 break
#             elif key == ord("r"):
#                 stable_start_t = None
#                 stable_s = 0.0
#                 r0 = None
#                 bend_ratio0 = None
#                 follow_active = False
#                 prev_r = None
#                 prev_dr = None
#                 filt_target = BASE_TCP_MM.copy()
#                 prev_cmd = BASE_TCP_MM.copy()
#                 last_print_cmd = BASE_TCP_MM.copy()

#                 node.set_follow_active(False)
#                 node.update_cmd(BASE_TCP_MM, FOLLOW_T, pose_ok=True)

#                 print("[RESET] back to STABLE_WAIT")
#             elif key == ord("m"):
#                 move_enabled = not move_enabled
#                 node.set_move_enabled(move_enabled)
#                 print(f"[TOGGLE] move_enabled={move_enabled} (ENABLE_FOLLOW_MOVE initial={ENABLE_FOLLOW_MOVE})")

#     finally:
#         pipeline.stop()
#         csv_file.close()
#         cv2.destroyAllWindows()
#         pose.close()

#         node.destroy_node()
#         rclpy.shutdown()
#         print(f"Saved CSV → {OUTPUT_CSV}")

# if __name__ == "__main__":
#     main()




#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import csv
from datetime import datetime, timezone, timedelta

import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import EnableRobot, ServoP

# =========================
# 0) 안전 스위치
# =========================
# - 시작: 팔 안정화 3초(AUTO_ZERO) 완료 시 move_enabled를 강제로 True로 켬
# - 정지: 손목이 어깨보다 위로 3초 유지 시 move_enabled=False, follow_active=False, 프로그램 종료
ENABLE_BASE_MOVE = True      # 항상 BASE로 이동 (요구사항)

# =========================
# 1) 로봇 기본자세 (사용자 제공)
# =========================
BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)
BASE_RPY_DEG = np.array([178.7294, -1.9186, -96.5768], dtype=np.float64)
BASE_MOVE_T = 3.0

# =========================
# 2) Dobot 서비스 경로
# =========================
SRV_ENABLE = "/dobot_bringup_ros2/srv/EnableRobot"
SRV_SERVO  = "/dobot_bringup_ros2/srv/ServoP"

# =========================
# 3) RealSense/MediaPipe
# =========================
OUTPUT_CSV = "right_arm_keypoints.csv"

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FPS = 30
DEPTH_AVG_KERNEL = 3

SHOW_FPS_OVERLAY = True
FPS_ALPHA = 0.90

mp_pose = mp.solutions.pose
RIGHT_SHOULDER = 12
RIGHT_ELBOW = 14
RIGHT_WRIST = 16

# =========================
# 4) 안정화(3초) 조건 (시작 트리거)
# =========================
STABLE_TOL_MM = 10.0
STABLE_NOISE_BAND_MM = 10.0
STABLE_SECONDS = 3.0
MAX_JUMP_MM = 30.0

# =========================
# 4-2) 종료 트리거 (손목이 어깨보다 위로 3초 유지)
# =========================
STOP_HOLD_SECONDS = 3.0
STOP_Y_MARGIN_PX = 10  # 손목이 어깨보다 "확실히" 위로: wr_y < sh_y - margin

# =========================
# 5) follow 출력/전송 제어
# =========================
PRINT_GATE_MM = 40.0
FOLLOW_DEADBAND_MM = 3.0

# ★ 부드러움(계단) 개선 세트
SEND_HZ = 200.0 # 약 25.0 -> 약 1HZ
FOLLOW_T = 0.10

# =========================
# 6) 팔 -> 로봇 목표 매핑
# =========================
Kx = 0.8
Ky = 0.8
Kz = 0.8
BEND_GAIN = 0.35

# =========================
# 7) 로봇 cmd 부드러움/안전 제한
# =========================
MAX_STEP_MM = 4.0
CMD_DEADBAND_MM = 0.0



# 로봇 작업 공간 330x100x450 mm (좁음)
WORKSPACE_MIN = np.array([-600.0, -500.0, 150.0], dtype=np.float64)
WORKSPACE_MAX = np.array([-270.0,  550.0, 600.0], dtype=np.float64)

# =========================
# 스트리밍 안정성/안전 정책
# =========================
RX_TIMEOUT_SEC = 0.25
SOFTSTOP_ALPHA = 0.15
SOFTSTOP_T_MULT = 3.0

# ★ 최신값 추종 + 노이즈 완화(목표에만 약한 EMA)
TARGET_EMA_ALPHA = 0.30

# =========================
# 유틸
# =========================
def norm3(v) -> float:
    return float(np.linalg.norm(v))

def ema(prev, cur, alpha):
    return (1.0 - alpha) * prev + alpha * cur

def apply_deadband_vec(v_mm: np.ndarray, band_mm: float) -> np.ndarray:
    out = v_mm.copy()
    for i in range(3):
        a = abs(out[i])
        if a <= band_mm:
            out[i] = 0.0
        else:
            out[i] = np.sign(out[i]) * (a - band_mm)
    return out

def clamp_xyz(xyz: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(xyz, lo), hi)

def limit_step(cur: np.ndarray, target: np.ndarray, max_step: float) -> np.ndarray:
    d = target - cur
    dist = norm3(d)
    if dist <= 1e-9:
        return cur.copy()
    if dist <= max_step:
        return target.copy()
    return cur + d * (max_step / dist)

def safe_depth_deproject(depth_frame, intr, x_px, y_px, h, w, ksize=3):
    x_px = int(np.clip(x_px, 0, w - 1))
    y_px = int(np.clip(y_px, 0, h - 1))

    k = ksize // 2
    depths = []
    for yy in range(max(0, y_px - k), min(h, y_px + k + 1)):
        for xx in range(max(0, x_px - k), min(w, x_px + k + 1)):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                depths.append(d)

    depth_m = float(np.mean(depths)) if depths else 0.0
    if depth_m <= 0:
        return None, 0.0

    X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [x_px, y_px], depth_m)
    return (np.array([X, Y, Z], dtype=np.float64) * 1000.0), depth_m

# =========================
# Dobot Bridge (REALTIME STREAM, latest-only)
# =========================
class DobotBridge(Node):
    def __init__(self):
        super().__init__("dobot_mp_pose_follow_stream_latest_triggers")

        self.enable_cli = self.create_client(EnableRobot, SRV_ENABLE)
        self.servo_p_cli = self.create_client(ServoP, SRV_SERVO)

        self.get_logger().info("Dobot 서비스 대기 중...")
        self.enable_cli.wait_for_service()
        self.servo_p_cli.wait_for_service()
        self.get_logger().info("Dobot 서비스 연결 완료")

        req = EnableRobot.Request()
        fut = self.enable_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        self.get_logger().info("로봇 활성화 완료")

        # 시작은 "정지 상태"로 두고, AUTO_ZERO 트리거가 켜면 move_enabled=True로 전환
        self.move_enabled = False
        self.follow_active = False

        self.latest_target = BASE_TCP_MM.copy()
        self.latest_t = float(FOLLOW_T)

        self.cur_cmd = BASE_TCP_MM.copy()
        self.last_pose_ok_t = time.perf_counter()
        self._last_future = None

        self.stat_sent = 0
        self.stat_skipped_busy = 0
        self.stat_softstop = 0
        self.stat_last_send_dt = 0.0
        self._last_send_wall = time.perf_counter()

        self.send_period = 1.0 / max(1e-6, float(SEND_HZ))
        self._timer = self.create_timer(self.send_period, self._on_send_timer)

    def set_move_enabled(self, enabled: bool):
        self.move_enabled = bool(enabled)

    def set_follow_active(self, active: bool):
        self.follow_active = bool(active)

    def update_target(self, target_xyz_mm: np.ndarray, t: float, pose_ok: bool):
        self.latest_target = np.array(target_xyz_mm, dtype=np.float64)
        self.latest_t = float(t)
        if pose_ok:
            self.last_pose_ok_t = time.perf_counter()

    def move_to_blocking(self, x, y, z, rx, ry, rz, t):
        req = ServoP.Request()
        req.a = float(x);  req.b = float(y);  req.c = float(z)
        req.d = float(rx); req.e = float(ry); req.f = float(rz)
        req.param_value = [f"t={float(t)}"]
        fut = self.servo_p_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        self.cur_cmd = np.array([x, y, z], dtype=np.float64)

    def _servo_stream_async(self, x, y, z, rx, ry, rz, t) -> bool:
        if self._last_future is not None and (not self._last_future.done()):
            self.stat_skipped_busy += 1
            return False

        req = ServoP.Request()
        req.a = float(x);  req.b = float(y);  req.c = float(z)
        req.d = float(rx); req.e = float(ry); req.f = float(rz)
        req.param_value = [f"t={float(t)}"]
        self._last_future = self.servo_p_cli.call_async(req)

        self.stat_sent += 1
        return True

    def _on_send_timer(self):
        if not (self.follow_active and self.move_enabled):
            return

        now = time.perf_counter()
        self.stat_last_send_dt = now - self._last_send_wall
        self._last_send_wall = now

        pose_gap = now - self.last_pose_ok_t
        softstop = (pose_gap > RX_TIMEOUT_SEC)

        if softstop:
            self.latest_target = ema(self.latest_target, BASE_TCP_MM, SOFTSTOP_ALPHA)
            t_send = float(self.latest_t * SOFTSTOP_T_MULT)
            self.stat_softstop += 1
        else:
            t_send = float(self.latest_t)

        target = clamp_xyz(self.latest_target, WORKSPACE_MIN, WORKSPACE_MAX)

        self.latest_target = ema(self.latest_target, target, TARGET_EMA_ALPHA)
        target = self.latest_target

        self.cur_cmd = limit_step(self.cur_cmd, target, MAX_STEP_MM)

        if norm3(self.cur_cmd - target) < CMD_DEADBAND_MM:
            self.cur_cmd = target.copy()

        self._servo_stream_async(
            self.cur_cmd[0], self.cur_cmd[1], self.cur_cmd[2],
            BASE_RPY_DEG[0], BASE_RPY_DEG[1], BASE_RPY_DEG[2],
            t_send
        )

# =========================
# main
# =========================
def main():
    rclpy.init()
    node = DobotBridge()

    # 시작은 항상 "정지": AUTO_ZERO가 되면 True로 켜짐
    node.set_move_enabled(False)
    node.set_follow_active(False)

    if ENABLE_BASE_MOVE:
        print("베이스 자세로 이동 중")
        node.move_to_blocking(
            BASE_TCP_MM[0], BASE_TCP_MM[1], BASE_TCP_MM[2],
            BASE_RPY_DEG[0], BASE_RPY_DEG[1], BASE_RPY_DEG[2],
            BASE_MOVE_T
        )
        time.sleep(0.4)
        print("BASE pose reached.")

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
    profile = pipeline.start(cfg)

    align = rs.align(rs.stream.color)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    print("RealSense started.")
    print(f"START trigger: STABLE {STABLE_SECONDS:.1f}s → AUTO_ZERO → FOLLOW + MOVE_ON")
    print(f"STOP  trigger: wrist ABOVE shoulder for {STOP_HOLD_SECONDS:.1f}s → MOVE_OFF + EXIT")
    print(f"STREAM: SEND_HZ={SEND_HZ:.1f}Hz FOLLOW_T={FOLLOW_T:.2f} MAX_STEP_MM={MAX_STEP_MM:.1f}")
    print("Keys: r=RESET, q=quit")

    header = [
        "frame_idx","timestamp_iso",
        "stable_s","follow_active",
        "dr_norm","targetX","targetY","targetZ",
        "cmdX","cmdY","cmdZ",
        "pose_ok","fps",
        "stop_hold_s",
        "move_enabled",
        "sent_total","skipped_busy","softstop_total"
    ]
    csv_file = open(OUTPUT_CSV, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(header)

    KST = timezone(timedelta(hours=9))

    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    prev_time = time.perf_counter()
    fps_smooth = 0.0
    frame_idx = 0

    stable_start_t = None
    stable_s = 0.0

    r0 = None
    bend_ratio0 = None
    follow_active = False

    prev_r = None
    prev_dr = None

    stop_hold_start_t = None
    stop_hold_s = 0.0

    last_print = BASE_TCP_MM.copy()

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.0)

            now = time.perf_counter()
            dt = now - prev_time
            prev_time = now
            if dt > 0:
                fps = 1.0 / dt
                fps_smooth = FPS_ALPHA * fps_smooth + (1 - FPS_ALPHA) * fps

            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            h, w = img.shape[:2]

            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)

            timestamp = datetime.now(KST).isoformat()

            if SHOW_FPS_OVERLAY:
                cv2.putText(img, f"FPS: {fps_smooth:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            dr_norm = 0.0
            pose_ok = False
            target = node.latest_target.copy()

            if not results.pose_landmarks:
                stop_hold_start_t = None
                stop_hold_s = 0.0
                cv2.putText(img, "No pose landmarks (soft-stop)", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            else:
                def lm_px(idx):
                    lm = results.pose_landmarks.landmark[idx]
                    x_px = int(np.clip(lm.x * w, 0, w - 1))
                    y_px = int(np.clip(lm.y * h, 0, h - 1))
                    return x_px, y_px

                sh_px = lm_px(RIGHT_SHOULDER)
                el_px = lm_px(RIGHT_ELBOW)
                wr_px = lm_px(RIGHT_WRIST)

                # STOP 트리거: 손목이 어깨보다 위로 3초 유지 (follow 중일 때만)
                wrist_above = (wr_px[1] < (sh_px[1] - STOP_Y_MARGIN_PX))
                if wrist_above and follow_active:
                    if stop_hold_start_t is None:
                        stop_hold_start_t = time.perf_counter()
                    stop_hold_s = time.perf_counter() - stop_hold_start_t

                    cv2.putText(img, f"STOP_HOLD {stop_hold_s:.1f}/{STOP_HOLD_SECONDS:.1f}s",
                                (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

                    if stop_hold_s >= STOP_HOLD_SECONDS:
                        print("[STOP_TRIGGER] wrist above shoulder held → move_off + exit")
                        node.set_follow_active(False)
                        node.set_move_enabled(False)
                        break
                else:
                    stop_hold_start_t = None
                    stop_hold_s = 0.0

                sh_mm, _ = safe_depth_deproject(depth_frame, intr, sh_px[0], sh_px[1], h, w, DEPTH_AVG_KERNEL)
                el_mm, _ = safe_depth_deproject(depth_frame, intr, el_px[0], el_px[1], h, w, DEPTH_AVG_KERNEL)
                wr_mm, _ = safe_depth_deproject(depth_frame, intr, wr_px[0], wr_px[1], h, w, DEPTH_AVG_KERNEL)

                if sh_mm is None or el_mm is None or wr_mm is None:
                    cv2.putText(img, "Depth invalid (soft-stop)", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                else:
                    pose_ok = True

                    for (name, px) in [("shoulder", sh_px), ("elbow", el_px), ("wrist", wr_px)]:
                        cv2.circle(img, (px[0], px[1]), 6, (0,255,0), -1)
                        cv2.putText(img, name, (px[0]+6, px[1]-6),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    cv2.line(img, sh_px, el_px, (0,255,0), 3)
                    cv2.line(img, el_px, wr_px, (0,255,0), 3)

                    se = norm3(el_mm - sh_mm) + 1e-6
                    ew = norm3(wr_mm - el_mm) + 1e-6
                    bend_ratio = ew / se

                    r = (wr_mm - sh_mm)

                    # START 트리거: 안정화 3초 → AUTO_ZERO → follow_active=1 + move_enabled=1
                    if r0 is None:
                        if prev_r is None:
                            prev_r = r.copy()
                            stable_start_t = None
                            stable_s = 0.0
                        else:
                            r_delta = r - prev_r
                            r_delta = apply_deadband_vec(r_delta, STABLE_NOISE_BAND_MM)

                            jn = norm3(r_delta)
                            if jn > MAX_JUMP_MM:
                                r_delta = r_delta * (MAX_JUMP_MM / jn)

                            if norm3(r_delta) <= STABLE_TOL_MM:
                                if stable_start_t is None:
                                    stable_start_t = time.perf_counter()
                                stable_s = time.perf_counter() - stable_start_t

                                if stable_s >= STABLE_SECONDS:
                                    r0 = r.copy()
                                    bend_ratio0 = bend_ratio
                                    follow_active = True
                                    prev_dr = None

                                    node.set_follow_active(True)
                                    node.set_move_enabled(True)  # 핵심: 시작 트리거가 move_on
                                    node.update_target(BASE_TCP_MM, FOLLOW_T, pose_ok=True)

                                    print(f"[AUTO_ZERO] stable {stable_s:.2f}s → follow_active=1, move_enabled=1")
                            else:
                                stable_start_t = None
                                stable_s = 0.0

                            prev_r = r.copy()

                        cv2.putText(img, f"STABLE_WAIT {stable_s:.1f}/{STABLE_SECONDS:.1f}s",
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                    # FOLLOW
                    else:
                        raw_dr = (r - r0)
                        raw_dr = apply_deadband_vec(raw_dr, FOLLOW_DEADBAND_MM)

                        if prev_dr is None:
                            dr = raw_dr
                        else:
                            jump = raw_dr - prev_dr
                            jn = norm3(jump)
                            if jn > MAX_JUMP_MM:
                                jump = jump * (MAX_JUMP_MM / jn)
                            dr = prev_dr + jump
                        prev_dr = dr.copy()

                        dr_norm = norm3(dr)

                        dx =  Kx * dr[2]
                        dy = -Ky * dr[0]
                        dz = -Kz * dr[1]

                        bend_delta = (bend_ratio - bend_ratio0) if bend_ratio0 is not None else 0.0
                        depth_scale = 1.0 + BEND_GAIN * np.clip(bend_delta, -0.8, 0.8)
                        depth_scale = float(np.clip(depth_scale, 0.5, 1.5))
                        dx *= depth_scale

                        target = BASE_TCP_MM + np.array([dx, dy, dz], dtype=np.float64)
                        target = clamp_xyz(target, WORKSPACE_MIN, WORKSPACE_MAX)

                        if follow_active:
                            node.update_target(target, FOLLOW_T, pose_ok=True)

                        if norm3(target - last_print) >= PRINT_GATE_MM:
                            print(f"[{frame_idx}] target(mm) X:{target[0]:.1f} Y:{target[1]:.1f} Z:{target[2]:.1f} | |dr|={dr_norm:.1f}")
                            last_print = target.copy()

                        cv2.putText(img, f"FOLLOW:1 move={int(node.move_enabled)}", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (0,255,0) if node.move_enabled else (0,255,255), 2)

            if follow_active and (not pose_ok):
                node.update_target(node.latest_target, FOLLOW_T, pose_ok=False)

            cv2.putText(img, f"sent:{node.stat_sent} skip(busy):{node.stat_skipped_busy} soft:{node.stat_softstop}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,0), 2)
            cv2.putText(img, f"timer_dt:{node.stat_last_send_dt*1000:.1f}ms  period:{(1000.0/SEND_HZ):.1f}ms  t:{FOLLOW_T:.2f}",
                        (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 2)
            cv2.putText(img, f"cur_cmd X:{node.cur_cmd[0]:.1f} Y:{node.cur_cmd[1]:.1f} Z:{node.cur_cmd[2]:.1f}",
                        (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 2)

            csv_writer.writerow([
                frame_idx, timestamp,
                f"{stable_s:.3f}", int(follow_active),
                f"{dr_norm:.3f}",
                f"{target[0]:.3f}", f"{target[1]:.3f}", f"{target[2]:.3f}",
                f"{node.cur_cmd[0]:.3f}", f"{node.cur_cmd[1]:.3f}", f"{node.cur_cmd[2]:.3f}",
                int(pose_ok),
                f"{fps_smooth:.2f}",
                f"{stop_hold_s:.3f}",
                int(node.move_enabled),
                node.stat_sent, node.stat_skipped_busy, node.stat_softstop
            ])

            cv2.imshow("Stable 3s -> Follow (AUTO_START, STOP: wrist above 3s)", img)
            frame_idx += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                # RESET: 다시 안정화 대기(정지 상태로)
                stable_start_t = None
                stable_s = 0.0
                r0 = None
                bend_ratio0 = None
                follow_active = False
                prev_r = None
                prev_dr = None

                stop_hold_start_t = None
                stop_hold_s = 0.0

                node.set_follow_active(False)
                node.set_move_enabled(False)  # RESET 시 무조건 OFF로
                node.update_target(BASE_TCP_MM, FOLLOW_T, pose_ok=True)

                print("[RESET] back to STABLE_WAIT (move_enabled=0)")

    finally:
        pipeline.stop()
        csv_file.close()
        cv2.destroyAllWindows()
        pose.close()

        node.destroy_node()
        rclpy.shutdown()
        print(f"Saved CSV → {OUTPUT_CSV}")

if __name__ == "__main__":
    main()

