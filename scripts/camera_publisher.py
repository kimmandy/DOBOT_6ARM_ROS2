# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# """
# cam_only_target_publisher_fixed_timer.py

# 요구사항 반영:
# - 카메라 루프는 'target 계산/갱신'만 담당 (불규칙 OK)
# - publish는 고정 타이머(예: 30Hz)에서 DT마다 '항상' 실행 (규칙적)
# - publish 주기와 PRINT 주기를 DT에 맞춤 (0.08초가 아니라, 원하면 0.08도 별도 가능)
# - 최신 target만 사용 (큐/누적 없음) -> 토픽은 어차피 최신만 보는 쪽에서 사용하면 됨

# 키:
# - r : RESET (AUTO_ZERO 다시)
# - q : 종료
# """

# import time
# import threading
# from datetime import timezone, timedelta

# import cv2
# import numpy as np
# import mediapipe as mp
# import pyrealsense2 as rs

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped

# # =========================
# # 0) publish/출력 주기 (고정)
# # =========================
# SEND_HZ = 30.0                  # ★ 고정 publish 주기 (30Hz 권장)
# DT = 1.0 / SEND_HZ              # 0.0333s
# PRINT_EVERY_SEC = 0.08          # 로그는 0.08초마다(요구 유지). 원하면 DT로 바꾸세요.
# TOPIC_TARGET = "/arm_target_mm"

# # =========================
# # 1) 기준
# # =========================
# BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)

# # =========================
# # 2) RealSense/MediaPipe
# # =========================
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
# # 3) 시작 트리거: 안정화 3초 (AUTO_ZERO)
# # =========================
# STABLE_TOL_MM = 10.0
# STABLE_NOISE_BAND_MM = 10.0
# STABLE_SECONDS = 3.0
# MAX_JUMP_MM = 30.0

# # =========================
# # 4) 종료 트리거: 손목이 어깨보다 위로 3초 유지
# # =========================
# STOP_HOLD_SECONDS = 3.0
# STOP_Y_MARGIN_PX = 10

# # =========================
# # 5) 팔 -> target 매핑
# # =========================
# Kx = 0.8
# Ky = 0.8
# Kz = 0.8
# BEND_GAIN = 0.35

# # (원래 clamp 유지)
# WORKSPACE_MIN = np.array([-600.0, -500.0, 150.0], dtype=np.float64)
# WORKSPACE_MAX = np.array([-270.0,  550.0, 600.0], dtype=np.float64)

# # =========================
# # 6) 노이즈/부드러움 (카메라 계산용)
# # =========================
# FOLLOW_DEADBAND_MM = 3.0

# # (선택) 아주 약한 target EMA (토픽을 downstream에서 그대로 쓰면 흔들릴 수 있어서)
# TARGET_EMA_ALPHA = 0.25  # 0이면 끔

# # =========================
# # 유틸
# # =========================
# def norm3(v) -> float:
#     return float(np.linalg.norm(v))

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

# def ema(prev: np.ndarray, cur: np.ndarray, alpha: float) -> np.ndarray:
#     return (1.0 - alpha) * prev + alpha * cur

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
# # Node: 고정 타이머 publish (카메라 루프와 분리)
# # =========================
# class ArmTargetPublisher(Node):
#     def __init__(self):
#         super().__init__("arm_target_publisher_fixed_timer")

#         self.pub = self.create_publisher(PointStamped, TOPIC_TARGET, 10)

#         # 공유 상태 (카메라 루프에서 갱신)
#         self._lock = threading.Lock()
#         self._follow_active = False
#         self._latest_target = BASE_TCP_MM.copy()
#         self._latest_target_filt = BASE_TCP_MM.copy()

#         # 통계/디버그
#         self._last_print_t = time.perf_counter()
#         self._last_send_t = time.perf_counter()
#         self._send_dt_ms = 0.0
#         self._sent = 0

#         # ★ 고정 주기 publish 타이머
#         self._timer = self.create_timer(DT, self._on_timer)

#     def set_follow_active(self, active: bool):
#         with self._lock:
#             self._follow_active = bool(active)

#     def reset_target(self):
#         with self._lock:
#             self._latest_target = BASE_TCP_MM.copy()
#             self._latest_target_filt = BASE_TCP_MM.copy()

#     def update_target(self, target_mm: np.ndarray):
#         with self._lock:
#             self._latest_target = np.array(target_mm, dtype=np.float64)

#     def _on_timer(self):
#         # 고정 주기 측정
#         now = time.perf_counter()
#         self._send_dt_ms = (now - self._last_send_t) * 1000.0
#         self._last_send_t = now

#         with self._lock:
#             if not self._follow_active:
#                 return

#             target = clamp_xyz(self._latest_target, WORKSPACE_MIN, WORKSPACE_MAX)

#             # (선택) target EMA
#             if TARGET_EMA_ALPHA > 0.0:
#                 self._latest_target_filt = ema(self._latest_target_filt, target, TARGET_EMA_ALPHA)
#                 target_to_pub = self._latest_target_filt.copy()
#             else:
#                 target_to_pub = target.copy()

#         # publish
#         msg = PointStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "camera_only"
#         msg.point.x = float(target_to_pub[0])
#         msg.point.y = float(target_to_pub[1])
#         msg.point.z = float(target_to_pub[2])
#         self.pub.publish(msg)
#         self._sent += 1

#         # 로그 (요구: 0.08초)
#         if now - self._last_print_t >= PRINT_EVERY_SEC:
#             self._last_print_t = now
#             self.get_logger().info(
#                 f"[PUB {TOPIC_TARGET}] x={target_to_pub[0]:.1f} y={target_to_pub[1]:.1f} z={target_to_pub[2]:.1f} "
#                 f"| send_dt={self._send_dt_ms:.1f}ms (target={DT*1000:.1f}ms)"
#             )

# # =========================
# # main (카메라 루프: target 계산만)
# # =========================
# def main():
#     rclpy.init()
#     node = ArmTargetPublisher()

#     # ★ spin을 별도 스레드로 돌려 타이머 publish가 흔들리지 않게
#     spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     spin_thread.start()

#     # RealSense start
#     pipeline = rs.pipeline()
#     cfg = rs.config()
#     cfg.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
#     cfg.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
#     profile = pipeline.start(cfg)

#     align = rs.align(rs.stream.color)
#     intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

#     print("RealSense started.")
#     print(f"START trigger: stable {STABLE_SECONDS:.1f}s → AUTO_ZERO → FOLLOW+publish_on")
#     print(f"STOP  trigger: wrist ABOVE shoulder {STOP_HOLD_SECONDS:.1f}s → publish_off + EXIT")
#     print(f"PUBLISH: SEND_HZ={SEND_HZ:.1f}Hz (DT={DT*1000:.1f}ms), print_every={PRINT_EVERY_SEC:.2f}s")
#     print("Keys: r=RESET, q=quit")

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

#     # 안정화 트리거
#     stable_start_t = None
#     stable_s = 0.0
#     r0 = None
#     bend_ratio0 = None
#     follow_active = False
#     prev_r = None
#     prev_dr = None

#     # stop hold
#     stop_hold_start_t = None
#     stop_hold_s = 0.0

#     # 마지막 계산 target (pose 끊겼을 때 화면 표시용)
#     last_target = BASE_TCP_MM.copy()

#     try:
#         while True:
#             now = time.perf_counter()
#             dt = now - prev_time
#             prev_time = now
#             if dt > 0:
#                 fps = 1.0 / dt
#                 fps_smooth = FPS_ALPHA * fps_smooth + (1.0 - FPS_ALPHA) * fps

#             frames = pipeline.wait_for_frames()
#             frames = align.process(frames)
#             depth_frame = frames.get_depth_frame()
#             color_frame = frames.get_color_frame()
#             if not depth_frame or not color_frame:
#                 continue

#             img = np.asanyarray(color_frame.get_data())
#             h, w = img.shape[:2]

#             results = pose.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

#             if SHOW_FPS_OVERLAY:
#                 cv2.putText(img, f"FPS: {fps_smooth:.1f}", (10, 30),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

#             dr_norm = 0.0
#             pose_ok = False

#             if not results.pose_landmarks:
#                 stop_hold_start_t = None
#                 stop_hold_s = 0.0
#                 cv2.putText(img, "No pose (hold last target)", (10, 60),
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

#                 # STOP 트리거
#                 wrist_above = (wr_px[1] < (sh_px[1] - STOP_Y_MARGIN_PX))
#                 if wrist_above and follow_active:
#                     if stop_hold_start_t is None:
#                         stop_hold_start_t = time.perf_counter()
#                     stop_hold_s = time.perf_counter() - stop_hold_start_t

#                     cv2.putText(img, f"STOP_HOLD {stop_hold_s:.1f}/{STOP_HOLD_SECONDS:.1f}s",
#                                 (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

#                     if stop_hold_s >= STOP_HOLD_SECONDS:
#                         print("[STOP_TRIGGER] wrist above shoulder held → publish_off + exit")
#                         node.set_follow_active(False)
#                         break
#                 else:
#                     stop_hold_start_t = None
#                     stop_hold_s = 0.0

#                 # depth
#                 sh_mm, _ = safe_depth_deproject(depth_frame, intr, sh_px[0], sh_px[1], h, w, DEPTH_AVG_KERNEL)
#                 el_mm, _ = safe_depth_deproject(depth_frame, intr, el_px[0], el_px[1], h, w, DEPTH_AVG_KERNEL)
#                 wr_mm, _ = safe_depth_deproject(depth_frame, intr, wr_px[0], wr_px[1], h, w, DEPTH_AVG_KERNEL)

#                 if sh_mm is None or el_mm is None or wr_mm is None:
#                     cv2.putText(img, "Depth invalid (hold last target)", (10, 60),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
#                 else:
#                     pose_ok = True

#                     # draw
#                     for (name, px) in [("shoulder", sh_px), ("elbow", el_px), ("wrist", wr_px)]:
#                         cv2.circle(img, (px[0], px[1]), 6, (0,255,0), -1)
#                         cv2.putText(img, name, (px[0]+6, px[1]-6),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
#                     cv2.line(img, sh_px, el_px, (0,255,0), 3)
#                     cv2.line(img, el_px, wr_px, (0,255,0), 3)

#                     # bend ratio
#                     se = norm3(el_mm - sh_mm) + 1e-6
#                     ew = norm3(wr_mm - el_mm) + 1e-6
#                     bend_ratio = ew / se

#                     r = (wr_mm - sh_mm)

#                     # START: 안정화 3초 → AUTO_ZERO
#                     if r0 is None:
#                         if prev_r is None:
#                             prev_r = r.copy()
#                             stable_start_t = None
#                             stable_s = 0.0
#                         else:
#                             r_delta = r - prev_r
#                             r_delta = apply_deadband_vec(r_delta, STABLE_NOISE_BAND_MM)

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

#                                     node.reset_target()
#                                     node.set_follow_active(True)
#                                     print(f"[AUTO_ZERO] stable {stable_s:.2f}s → follow_active=1, publish_on=1")
#                             else:
#                                 stable_start_t = None
#                                 stable_s = 0.0

#                             prev_r = r.copy()

#                         cv2.putText(img, f"STABLE_WAIT {stable_s:.1f}/{STABLE_SECONDS:.1f}s",
#                                     (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

#                     else:
#                         # FOLLOW: 매 프레임 target 계산 (publish는 타이머가 담당)
#                         raw_dr = (r - r0)
#                         raw_dr = apply_deadband_vec(raw_dr, FOLLOW_DEADBAND_MM)

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
#                         dy = -Ky * dr[0]
#                         dz = -Kz * dr[1]

#                         bend_delta = (bend_ratio - bend_ratio0) if bend_ratio0 is not None else 0.0
#                         depth_scale = 1.0 + BEND_GAIN * np.clip(bend_delta, -0.8, 0.8)
#                         depth_scale = float(np.clip(depth_scale, 0.5, 1.5))
#                         dx *= depth_scale

#                         target = BASE_TCP_MM + np.array([dx, dy, dz], dtype=np.float64)
#                         target = clamp_xyz(target, WORKSPACE_MIN, WORKSPACE_MAX)

#                         last_target = target.copy()

#                         # ★ publish 대신 latest_target만 갱신
#                         node.update_target(target)

#                         cv2.putText(img, f"FOLLOW:1 publish_hz={SEND_HZ:.0f}", (10, 60),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

#             # overlay debug
#             cv2.putText(img, f"frame:{frame_idx} pose_ok:{int(pose_ok)} |dr|={dr_norm:.1f}",
#                         (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,0), 2)
#             cv2.putText(img, f"target X:{last_target[0]:.1f} Y:{last_target[1]:.1f} Z:{last_target[2]:.1f}",
#                         (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 2)

#             cv2.imshow("Camera only: Pose -> Target (fixed timer publish)", img)
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

#                 stop_hold_start_t = None
#                 stop_hold_s = 0.0

#                 last_target = BASE_TCP_MM.copy()

#                 node.set_follow_active(False)
#                 node.reset_target()
#                 print("[RESET] back to STABLE_WAIT (publish_off)")

#     finally:
#         pipeline.stop()
#         cv2.destroyAllWindows()
#         pose.close()

#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()








# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import time
# from datetime import timezone, timedelta

# import cv2
# import numpy as np
# import mediapipe as mp
# import pyrealsense2 as rs

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped

# # =========================
# # 출력/토픽
# # =========================
# TOPIC_TARGET = "/arm_target_mm"
# PRINT_EVERY_SEC = 0.08

# # =========================
# # "기준" (target 계산 기준점)
# # =========================
# BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)

# # =========================
# # RealSense/MediaPipe
# # =========================
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
# # 시작 트리거: 안정화 3초 (AUTO_ZERO)
# # =========================
# STABLE_TOL_MM = 10.0
# STABLE_NOISE_BAND_MM = 10.0
# STABLE_SECONDS = 3.0
# MAX_JUMP_MM = 30.0

# # =========================
# # 종료 트리거: 손목이 어깨보다 위로 3초 유지
# # =========================
# STOP_HOLD_SECONDS = 3.0
# STOP_Y_MARGIN_PX = 10

# # =========================
# # 팔 -> target 매핑
# # =========================
# Kx = 0.8
# Ky = 0.8
# Kz = 0.8
# BEND_GAIN = 0.35

# # =========================
# # 노이즈/부드러움(카메라 출력용)
# # =========================
# FOLLOW_DEADBAND_MM = 3.0

# # =========================
# # 유틸
# # =========================
# def norm3(v) -> float:
#     return float(np.linalg.norm(v))

# def apply_deadband_vec(v_mm: np.ndarray, band_mm: float) -> np.ndarray:
#     out = v_mm.copy()
#     for i in range(3):
#         a = abs(out[i])
#         if a <= band_mm:
#             out[i] = 0.0
#         else:
#             out[i] = np.sign(out[i]) * (a - band_mm)
#     return out

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
# # 카메라-only 노드: target publish
# # =========================
# class ArmTargetPublisher(Node):
#     def __init__(self):
#         super().__init__("arm_target_publisher_cam_only")
#         self.pub = self.create_publisher(PointStamped, TOPIC_TARGET, 10)
#         self._last_print_t = time.perf_counter()

#     def publish_target(self, target_mm: np.ndarray):
#         msg = PointStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "camera_only"
#         msg.point.x = float(target_mm[0])
#         msg.point.y = float(target_mm[1])
#         msg.point.z = float(target_mm[2])
#         self.pub.publish(msg)

#         now = time.perf_counter()
#         if now - self._last_print_t >= PRINT_EVERY_SEC:
#             self._last_print_t = now
#             self.get_logger().info(
#                 f"[PUB {TOPIC_TARGET}] x={target_mm[0]:.1f} y={target_mm[1]:.1f} z={target_mm[2]:.1f}"
#             )

# def main():
#     rclpy.init()
#     node = ArmTargetPublisher()

#     # RealSense start
#     pipeline = rs.pipeline()
#     cfg = rs.config()
#     cfg.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
#     cfg.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
#     profile = pipeline.start(cfg)

#     align = rs.align(rs.stream.color)
#     intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

#     print("RealSense started.")
#     print(f"START trigger: stable {STABLE_SECONDS:.1f}s → AUTO_ZERO → publish target")
#     print(f"STOP  trigger: wrist ABOVE shoulder {STOP_HOLD_SECONDS:.1f}s → EXIT")
#     print("Keys: r=RESET, q=quit")

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

#     stop_hold_start_t = None
#     stop_hold_s = 0.0

#     last_target = BASE_TCP_MM.copy()

#     try:
#         while True:
#             rclpy.spin_once(node, timeout_sec=0.0)

#             now = time.perf_counter()
#             dt = now - prev_time
#             prev_time = now
#             if dt > 0:
#                 fps = 1.0 / dt
#                 fps_smooth = FPS_ALPHA * fps_smooth + (1.0 - FPS_ALPHA) * fps

#             frames = pipeline.wait_for_frames()
#             frames = align.process(frames)
#             depth_frame = frames.get_depth_frame()
#             color_frame = frames.get_color_frame()
#             if not depth_frame or not color_frame:
#                 continue

#             img = np.asanyarray(color_frame.get_data())
#             h, w = img.shape[:2]

#             results = pose.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

#             if SHOW_FPS_OVERLAY:
#                 cv2.putText(img, f"FPS: {fps_smooth:.1f}", (10, 30),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

#             dr_norm = 0.0
#             pose_ok = False
#             target = last_target.copy()

#             if not results.pose_landmarks:
#                 stop_hold_start_t = None
#                 stop_hold_s = 0.0
#                 cv2.putText(img, "No pose (hold last target)", (10, 60),
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

#                 wrist_above = (wr_px[1] < (sh_px[1] - STOP_Y_MARGIN_PX))
#                 if wrist_above and follow_active:
#                     if stop_hold_start_t is None:
#                         stop_hold_start_t = time.perf_counter()
#                     stop_hold_s = time.perf_counter() - stop_hold_start_t
#                     cv2.putText(img, f"STOP_HOLD {stop_hold_s:.1f}/{STOP_HOLD_SECONDS:.1f}s",
#                                 (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
#                     if stop_hold_s >= STOP_HOLD_SECONDS:
#                         print("[STOP_TRIGGER] wrist above shoulder held → exit")
#                         break
#                 else:
#                     stop_hold_start_t = None
#                     stop_hold_s = 0.0

#                 sh_mm, _ = safe_depth_deproject(depth_frame, intr, sh_px[0], sh_px[1], h, w, DEPTH_AVG_KERNEL)
#                 el_mm, _ = safe_depth_deproject(depth_frame, intr, el_px[0], el_px[1], h, w, DEPTH_AVG_KERNEL)
#                 wr_mm, _ = safe_depth_deproject(depth_frame, intr, wr_px[0], wr_px[1], h, w, DEPTH_AVG_KERNEL)

#                 if sh_mm is None or el_mm is None or wr_mm is None:
#                     cv2.putText(img, "Depth invalid (hold last target)", (10, 60),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
#                 else:
#                     pose_ok = True

#                     se = norm3(el_mm - sh_mm) + 1e-6
#                     ew = norm3(wr_mm - el_mm) + 1e-6
#                     bend_ratio = ew / se

#                     r = (wr_mm - sh_mm)

#                     # AUTO_ZERO
#                     if r0 is None:
#                         if prev_r is None:
#                             prev_r = r.copy()
#                             stable_start_t = None
#                             stable_s = 0.0
#                         else:
#                             r_delta = r - prev_r
#                             r_delta = apply_deadband_vec(r_delta, STABLE_NOISE_BAND_MM)

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
#                                     print(f"[AUTO_ZERO] stable {stable_s:.2f}s → follow_active=1")
#                             else:
#                                 stable_start_t = None
#                                 stable_s = 0.0

#                             prev_r = r.copy()

#                         cv2.putText(img, f"STABLE_WAIT {stable_s:.1f}/{STABLE_SECONDS:.1f}s",
#                                     (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

#                     else:
#                         raw_dr = (r - r0)
#                         raw_dr = apply_deadband_vec(raw_dr, FOLLOW_DEADBAND_MM)

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
#                         dy = -Ky * dr[0]
#                         dz = -Kz * dr[1]

#                         bend_delta = (bend_ratio - bend_ratio0) if bend_ratio0 is not None else 0.0
#                         depth_scale = 1.0 + BEND_GAIN * np.clip(bend_delta, -0.8, 0.8)
#                         depth_scale = float(np.clip(depth_scale, 0.5, 1.5))
#                         dx *= depth_scale

#                         target = BASE_TCP_MM + np.array([dx, dy, dz], dtype=np.float64)
#                         last_target = target.copy()

#                         node.publish_target(target)
#                         cv2.putText(img, "FOLLOW:1 (publishing)", (10, 60),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

#             cv2.putText(img, f"frame:{frame_idx} pose_ok:{int(pose_ok)} |dr|={dr_norm:.1f}",
#                         (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,0), 2)
#             cv2.putText(img, f"target X:{last_target[0]:.1f} Y:{last_target[1]:.1f} Z:{last_target[2]:.1f}",
#                         (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 2)

#             cv2.imshow("Camera only: Pose -> Target publish", img)
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
#                 stop_hold_start_t = None
#                 stop_hold_s = 0.0
#                 last_target = BASE_TCP_MM.copy()
#                 print("[RESET] back to STABLE_WAIT")

#     finally:
#         pipeline.stop()
#         cv2.destroyAllWindows()
#         pose.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()








#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import timezone, timedelta

import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# =========================
# 0) 출력/디버그
# =========================
PRINT_EVERY_SEC = 0.08
TOPIC_TARGET = "/arm_target_mm"

# =========================
# 1) "기준" (로봇 연결 안 해도 target 계산 기준점으로 씀)
# =========================
BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)

# =========================
# 2) RealSense/MediaPipe
# =========================
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
# 3) 시작 트리거: 안정화 3초 (AUTO_ZERO)
# =========================
STABLE_TOL_MM = 10.0
STABLE_NOISE_BAND_MM = 10.0
STABLE_SECONDS = 3.0
MAX_JUMP_MM = 30.0

# =========================
# 4) 종료 트리거: 손목이 어깨보다 위로 3초 유지
# =========================
STOP_HOLD_SECONDS = 3.0
STOP_Y_MARGIN_PX = 10

# =========================
# 5) 팔 -> target 매핑
# =========================
Kx = 0.8
Ky = 0.8
Kz = 0.8
BEND_GAIN = 0.35

# (원하면 clamp 유지)
WORKSPACE_MIN = np.array([-600.0, -500.0, 150.0], dtype=np.float64)
WORKSPACE_MAX = np.array([-270.0,  550.0, 600.0], dtype=np.float64)

# =========================
# 6) 노이즈/부드러움(카메라 출력 확인용)
# =========================
FOLLOW_DEADBAND_MM = 3.0

# =========================
# 유틸
# =========================
def norm3(v) -> float:
    return float(np.linalg.norm(v))

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

def draw_arm_points(img, sh_px, el_px, wr_px):
    """어깨/팔꿈치/손목 3점 + 선 2개 + 라벨 표시"""
    # 선
    cv2.line(img, sh_px, el_px, (0, 255, 0), 4)
    cv2.line(img, el_px, wr_px, (0, 255, 0), 4)

    # 점
    cv2.circle(img, sh_px, 10, (0, 255, 0), -1)
    cv2.circle(img, el_px, 10, (0, 255, 0), -1)
    cv2.circle(img, wr_px, 10, (0, 255, 0), -1)

    # 라벨
    cv2.putText(img, "S", (sh_px[0] + 12, sh_px[1] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(img, "E", (el_px[0] + 12, el_px[1] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(img, "W", (wr_px[0] + 12, wr_px[1] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

# =========================
# 카메라-only 노드: target publish + print
# =========================
class ArmTargetPublisher(Node):
    def __init__(self):
        super().__init__("arm_target_publisher_cam_only")
        self.pub = self.create_publisher(PointStamped, TOPIC_TARGET, 10)
        self._last_print_t = time.perf_counter()

    def publish_target(self, target_mm: np.ndarray):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_only"
        msg.point.x = float(target_mm[0])
        msg.point.y = float(target_mm[1])
        msg.point.z = float(target_mm[2])
        self.pub.publish(msg)

        now = time.perf_counter()
        if now - self._last_print_t >= PRINT_EVERY_SEC:
            self._last_print_t = now
            self.get_logger().info(
                f"[PUB {TOPIC_TARGET}] x={target_mm[0]:.1f} y={target_mm[1]:.1f} z={target_mm[2]:.1f}"
            )

# =========================
# main (카메라 루프)
# =========================
def main():
    rclpy.init()
    node = ArmTargetPublisher()

    # RealSense start
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
    profile = pipeline.start(cfg)

    align = rs.align(rs.stream.color)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    print("RealSense started.")
    print(f"START trigger: stable {STABLE_SECONDS:.1f}s → AUTO_ZERO → publish target")
    print(f"STOP  trigger: wrist ABOVE shoulder {STOP_HOLD_SECONDS:.1f}s → EXIT")
    print("Keys: r=RESET, q=quit")

    KST = timezone(timedelta(hours=9))  # (현재 코드에 필요하진 않지만 기존 구조 유지)

    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    prev_time = time.perf_counter()
    fps_smooth = 0.0
    frame_idx = 0

    # 안정화 트리거
    stable_start_t = None
    stable_s = 0.0
    r0 = None
    bend_ratio0 = None
    follow_active = False
    prev_r = None
    prev_dr = None

    # stop hold
    stop_hold_start_t = None
    stop_hold_s = 0.0

    # 마지막 target
    last_target = BASE_TCP_MM.copy()

    try:
        while True:
            # ROS (publisher timestamp 등)
            rclpy.spin_once(node, timeout_sec=0.0)

            now = time.perf_counter()
            dt = now - prev_time
            prev_time = now
            if dt > 0:
                fps = 1.0 / dt
                fps_smooth = FPS_ALPHA * fps_smooth + (1.0 - FPS_ALPHA) * fps

            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            h, w = img.shape[:2]

            results = pose.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

            if SHOW_FPS_OVERLAY:
                cv2.putText(img, f"FPS: {fps_smooth:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            dr_norm = 0.0
            pose_ok = False

            if not results.pose_landmarks:
                stop_hold_start_t = None
                stop_hold_s = 0.0
                cv2.putText(img, "No pose (hold last target)", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            else:
                def lm_px(idx):
                    lm = results.pose_landmarks.landmark[idx]
                    x_px = int(np.clip(lm.x * w, 0, w - 1))
                    y_px = int(np.clip(lm.y * h, 0, h - 1))
                    return x_px, y_px

                sh_px = lm_px(RIGHT_SHOULDER)
                el_px = lm_px(RIGHT_ELBOW)
                wr_px = lm_px(RIGHT_WRIST)

                # 항상 3점/선 표시 (depth 실패해도 “2D 위치”는 보이도록)
                draw_arm_points(img, sh_px, el_px, wr_px)

                # STOP 트리거
                wrist_above = (wr_px[1] < (sh_px[1] - STOP_Y_MARGIN_PX))
                if wrist_above and follow_active:
                    if stop_hold_start_t is None:
                        stop_hold_start_t = time.perf_counter()
                    stop_hold_s = time.perf_counter() - stop_hold_start_t

                    cv2.putText(img, f"STOP_HOLD {stop_hold_s:.1f}/{STOP_HOLD_SECONDS:.1f}s",
                                (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    if stop_hold_s >= STOP_HOLD_SECONDS:
                        print("[STOP_TRIGGER] wrist above shoulder held → exit")
                        break
                else:
                    stop_hold_start_t = None
                    stop_hold_s = 0.0

                # depth (mm)
                sh_mm, _ = safe_depth_deproject(depth_frame, intr, sh_px[0], sh_px[1], h, w, DEPTH_AVG_KERNEL)
                el_mm, _ = safe_depth_deproject(depth_frame, intr, el_px[0], el_px[1], h, w, DEPTH_AVG_KERNEL)
                wr_mm, _ = safe_depth_deproject(depth_frame, intr, wr_px[0], wr_px[1], h, w, DEPTH_AVG_KERNEL)

                if sh_mm is None or el_mm is None or wr_mm is None:
                    cv2.putText(img, "Depth invalid (hold last target)", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    pose_ok = True

                    # bend ratio
                    se = norm3(el_mm - sh_mm) + 1e-6
                    ew = norm3(wr_mm - el_mm) + 1e-6
                    bend_ratio = ew / se

                    r = (wr_mm - sh_mm)

                    # START: 안정화 3초 → AUTO_ZERO
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
                                    print(f"[AUTO_ZERO] stable {stable_s:.2f}s → follow_active=1")
                            else:
                                stable_start_t = None
                                stable_s = 0.0

                            prev_r = r.copy()

                        cv2.putText(img, f"STABLE_WAIT {stable_s:.1f}/{STABLE_SECONDS:.1f}s",
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    else:
                        # FOLLOW: 매 프레임 target 계산 + publish
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

                        # clamp 유지
                        target = clamp_xyz(target, WORKSPACE_MIN, WORKSPACE_MAX)

                        last_target = target.copy()

                        # publish
                        node.publish_target(target)

                        cv2.putText(img, "FOLLOW:1 (publishing)", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # overlay debug
            cv2.putText(img, f"frame:{frame_idx} pose_ok:{int(pose_ok)} |dr|={dr_norm:.1f}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            cv2.putText(img, f"target X:{last_target[0]:.1f} Y:{last_target[1]:.1f} Z:{last_target[2]:.1f}",
                        (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

            cv2.imshow("Camera only: Pose -> Target publish", img)
            frame_idx += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                stable_start_t = None
                stable_s = 0.0
                r0 = None
                bend_ratio0 = None
                follow_active = False
                prev_r = None
                prev_dr = None

                stop_hold_start_t = None
                stop_hold_s = 0.0

                last_target = BASE_TCP_MM.copy()
                print("[RESET] back to STABLE_WAIT")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        pose.close()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()