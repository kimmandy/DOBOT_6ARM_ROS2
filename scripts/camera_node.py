#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading

import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PointStamped

# =========================
# 0) 출력/디버그 (터미널 출력용)
# =========================
PRINT_EVERY_SEC = 1.0
TOPIC_TARGET    = "/arm_target_mm"

# =========================
# 1) 기준점 (카메라에 감지된 팔의 기준점)
# =========================
BASE_TCP_MM = np.array([-514.8936, -157.7017, 334.1491], dtype=np.float64)

# =========================
# 2) RealSense / MediaPipe
# =========================
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FPS = 30
DEPTH_AVG_KERNEL = 3

SHOW_FPS_OVERLAY = True
FPS_ALPHA  = 0.90

# 3점의 포즈
mp_pose = mp.solutions.pose
RIGHT_SHOULDER = 12
RIGHT_ELBOW = 14
RIGHT_WRIST = 16

# =========================
# 3) 시작 트리거 (AUTO_ZERO)
# =========================
STABLE_TOL_MM = 10.0
STABLE_NOISE_BAND_MM = 10.0
STABLE_SECONDS = 3.0

# 가만히 있는데 튀는 값 잡기
MAX_JUMP_MM = 30.0

# =========================
# 4) 종료 트리거
# =========================
STOP_HOLD_SECONDS = 3.0
STOP_Y_MARGIN_PX = 15

# =========================
# 5) 팔 → target 매핑
# =========================
Kx = 0.8
Ky = 0.8
Kz = 0.8
BEND_GAIN = 0.35


# 로봇의 작업반경
WORKSPACE_MIN = np.array([-600.0, -500.0, 150.0], dtype=np.float64)
WORKSPACE_MAX = np.array([-270.0,  550.0, 600.0], dtype=np.float64)

# =========================
# 6) 노이즈 / 부드러움
# =========================
FOLLOW_DEADBAND_MM = 3.0



def norm3(v) -> float:
    return float(np.linalg.norm(v))


# [5] 벡터 크기 기준 deadband (방향 왜곡 없음)
def apply_deadband_vec(v: np.ndarray, band: float) -> np.ndarray:
    mag = norm3(v)
    if mag <= band:
        return np.zeros(3, dtype=np.float64)
    return v * (mag - band) / mag


def clamp_xyz(xyz, lo, hi):
    return np.clip(xyz, lo, hi)


# [6] 최신 프레임만 사용 (큐 소진)
def get_latest_frames(pipeline: rs.pipeline, align: rs.align, timeout_ms: int = 5000):
    latest = None
    while True:
        f = pipeline.poll_for_frames()
        if not f or not f.get_depth_frame() or not f.get_color_frame():
            break
        latest = f
    if latest is None:
        for attempt in range(3):
            try:
                latest = pipeline.wait_for_frames(timeout_ms=timeout_ms)
                if latest.get_depth_frame() and latest.get_color_frame():
                    break
            except RuntimeError as e:
                print(f"[WARN] wait_for_frames 재시도 {attempt+1}/3: {e}")
                time.sleep(0.05)
        else:
            return None
    return align.process(latest)


def safe_depth_deproject(depth_frame, intr, x_px, y_px, h, w, ksize=3):
    x_px = int(np.clip(x_px, 0, w - 1))
    y_px = int(np.clip(y_px, 0, h - 1))
    k = ksize // 2
    depths = [
        depth_frame.get_distance(xx, yy)
        for yy in range(max(0, y_px - k), min(h, y_px + k + 1))
        for xx in range(max(0, x_px - k), min(w, x_px + k + 1))
        if depth_frame.get_distance(xx, yy) > 0
    ]
    if not depths:
        return None, 0.0
    depth_m = float(np.mean(depths))
    X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [x_px, y_px], depth_m)
    return np.array([X, Y, Z], dtype=np.float64) * 1000.0, depth_m


def draw_arm_points(img, sh_px, el_px, wr_px):
    cv2.line(img, sh_px, el_px, (0, 255, 0), 4)
    cv2.line(img, el_px, wr_px, (0, 255, 0), 4)
    for name, px in [("S", sh_px), ("E", el_px), ("W", wr_px)]:
        cv2.circle(img, px, 10, (0, 255, 0), -1)
        cv2.putText(img, name, (px[0]+12, px[1]-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)


# =========================
# 카메라 노드 (publisher)
# =========================

class ArmTargetPublisher(Node):
    def __init__(self):
        super().__init__("arm_target_publisher_cam_only")

        # [7] QoS: 로봇 노드 subscriber와 반드시 일치
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1, 
        )
        self.pub = self.create_publisher(PointStamped, TOPIC_TARGET, qos)
        self._last_print_t = time.perf_counter()
        self.stat_published = 0

    def publish_target(self, target_mm: np.ndarray):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_only"
        msg.point.x = float(target_mm[0])
        msg.point.y = float(target_mm[1])
        msg.point.z = float(target_mm[2])
        self.pub.publish(msg)
        self.stat_published += 1

        now = time.perf_counter()
        if now - self._last_print_t >= PRINT_EVERY_SEC:
            self._last_print_t = now
            self.get_logger().info(
                f"[PUB] x={target_mm[0]:.1f} y={target_mm[1]:.1f} "
                f"z={target_mm[2]:.1f}  total={self.stat_published}"
            )


# =========================
# main
# =========================

def main():
    rclpy.init()
    node = ArmTargetPublisher()

    # [1] spin 별도 스레드 (카메라 블로킹과 독립)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()


    # RealSense 초기화
    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16,  FPS)
    profile  = pipeline.start(cfg)
    align    = rs.align(rs.stream.color)
    intr     = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()


    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )


    print("RealSense started.")
    print(f"START: 팔을 {STABLE_SECONDS:.1f}초 정지 → AUTO_ZERO → publish 시작")
    print(f"STOP : 손목을 어깨 위로 {STOP_HOLD_SECONDS:.1f}초 유지 → 종료")
    print("Keys : r=RESET  q=QUIT")


    fps_smooth = 0.0
    prev_time  = time.perf_counter()
    frame_idx  = 0


    # 추적 상태
    stable_start_t = None # 팔이 처음으로 멈춘 시각. (시작 트리거 3초를 재기 시작한 기준점)
    stable_s = 0.0  # 팔이 현재까지 멈춰있는 누적 시간
    r0 = None # AUTO_ZERO 완료시 시록한 기준 팔 벡터 (NONE이면 아직 안됨)
    bend_ratio0 = None # AUTO_ZERO 완료시의 팔 굽힘 비율 (팔꿈치-손목 거리 / 어깨-팔꿈치 거리)
    follow_active = False # AUTO_ZERO 완료되어 팔 움직임에 따라 target publish 중인지 여부
    prev_r = None  # 이전 프레임의 팔 벡터. 현재 프레임과 비교해 얼마나 움직였는지 계산
    prev_dr= None  # 직점 프레임의 변위(dr) 현재 프레임과 비교해 점프가 너무 크면 제한
    stop_hold_start_t = None  # 손목이 어깨위로 올라간 시각. (종료 트리거 3초를 재기 시작한 기준점)
    stop_hold_s= 0.0   # 손목이 어깨 위에 있었던 누적 시간.
    last_target = BASE_TCP_MM.copy()


    try:
        while True:
            now = time.perf_counter()
            dt  = now - prev_time
            prev_time = now
            if dt > 0:
                fps_smooth = FPS_ALPHA * fps_smooth + (1.0 - FPS_ALPHA) / dt

            # [6] 최신 프레임만 사용
            frames = get_latest_frames(pipeline, align)
            if frames is None:
                print("[WARN] 프레임 수신 실패, 재시도")
                continue
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            img     = np.asanyarray(color_frame.get_data())
            h, w    = img.shape[:2]
            results = pose.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

            if SHOW_FPS_OVERLAY:
                cv2.putText(img, f"FPS: {fps_smooth:.1f}  pub:{node.stat_published}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            dr_norm  = 0.0
            pose_ok  = False

            if not results.pose_landmarks:
                stop_hold_start_t = None
                stop_hold_s       = 0.0
                cv2.putText(img, "No pose (hold last target)",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # [2] pose 없어도 follow 중이면 last_target 계속 publish
                if follow_active:
                    node.publish_target(last_target)

            else:
                def lm_px(idx):
                    lm = results.pose_landmarks.landmark[idx]
                    return (int(np.clip(lm.x * w, 0, w-1)),
                            int(np.clip(lm.y * h, 0, h-1)))

                sh_px = lm_px(RIGHT_SHOULDER)
                el_px = lm_px(RIGHT_ELBOW)
                wr_px = lm_px(RIGHT_WRIST)

                draw_arm_points(img, sh_px, el_px, wr_px)

                # STOP 트리거
                wrist_above = wr_px[1] < sh_px[1] - STOP_Y_MARGIN_PX
                if wrist_above and follow_active:
                    if stop_hold_start_t is None:
                        stop_hold_start_t = now
                    stop_hold_s = now - stop_hold_start_t
                    cv2.putText(img,
                        f"STOP_HOLD {stop_hold_s:.1f}/{STOP_HOLD_SECONDS:.1f}s",
                        (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    if stop_hold_s >= STOP_HOLD_SECONDS:
                        print("[STOP_TRIGGER] 종료")
                        break
                else:
                    stop_hold_start_t = None
                    stop_hold_s = 0.0

                # Depth 역투영
                sh_mm, _ = safe_depth_deproject(depth_frame, intr, *sh_px, h, w, DEPTH_AVG_KERNEL)
                el_mm, _ = safe_depth_deproject(depth_frame, intr, *el_px, h, w, DEPTH_AVG_KERNEL)
                wr_mm, _ = safe_depth_deproject(depth_frame, intr, *wr_px, h, w, DEPTH_AVG_KERNEL)

                if sh_mm is None or el_mm is None or wr_mm is None:
                    cv2.putText(img, "Depth invalid (hold last target)",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    # [2] depth 실패해도 follow 중이면 last_target publish
                    if follow_active:
                        node.publish_target(last_target)
                else:
                    pose_ok = True
                    bend_ratio = (norm3(wr_mm - el_mm) + 1e-6) / (norm3(el_mm - sh_mm) + 1e-6)
                    r = wr_mm - sh_mm

                    # AUTO_ZERO
                    if r0 is None:
                        if prev_r is None:
                            prev_r = r.copy()
                            stable_start_t = None
                            stable_s = 0.0
                        else:
                            r_delta = r - prev_r
                            r_delta = apply_deadband_vec(r_delta, STABLE_NOISE_BAND_MM)

                            # [3] clamp 전 크기로 안정성 판단
                            jn = norm3(r_delta)
                            is_jump  = jn > MAX_JUMP_MM
                            if is_jump:
                                r_delta = r_delta * (MAX_JUMP_MM / jn)

                            if not is_jump and norm3(r_delta) <= STABLE_TOL_MM:
                                if stable_start_t is None:
                                    stable_start_t = now
                                stable_s = now - stable_start_t
                                if stable_s >= STABLE_SECONDS:
                                    r0 = r.copy()
                                    bend_ratio0 = bend_ratio
                                    follow_active = True
                                    prev_dr = None
                                    print(f"[AUTO_ZERO] {stable_s:.2f}s 안정 → follow 시작")
                            else:
                                stable_start_t = None
                                stable_s = 0.0

                            prev_r = r.copy()

                        cv2.putText(img,
                            f"STABLE_WAIT {stable_s:.1f}/{STABLE_SECONDS:.1f}s",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    else:
                        # FOLLOW
                        raw_dr = apply_deadband_vec(r - r0, FOLLOW_DEADBAND_MM)

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

                        bd = (bend_ratio - bend_ratio0) if bend_ratio0 is not None else 0.0
                        dx *= float(np.clip(1.0 + BEND_GAIN * np.clip(bd, -0.8, 0.8), 0.5, 1.5))

                        target = clamp_xyz(
                            BASE_TCP_MM + np.array([dx, dy, dz], dtype=np.float64),
                            WORKSPACE_MIN, WORKSPACE_MAX,
                        )
                        last_target = target.copy()

                        # [2] 항상 publish (팔 멈춰도 last_target 유지 전송)
                        node.publish_target(last_target)

                        cv2.putText(img, "FOLLOW (publishing)",
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(img,
                f"frame:{frame_idx} pose_ok:{int(pose_ok)} |dr|={dr_norm:.1f}",
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            cv2.putText(img,
                f"target X:{last_target[0]:.1f} Y:{last_target[1]:.1f} Z:{last_target[2]:.1f}",
                (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

            cv2.imshow("Camera Node: Pose -> Target", img)
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
                print("[RESET] STABLE_WAIT으로 복귀")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        pose.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
