import cv2
import numpy as np
import math
import threading
import time
import platform
import os
import json
from pathlib import Path
from collections import deque
from typing import Tuple, Optional

SRC_ROOT = Path(__file__).resolve().parent.parent  # src/
CONFIG_DIR = SRC_ROOT / "config"
CALIB_DIR = CONFIG_DIR / "calibration"

def _resource_path(filename: str):
    """Search config/calibration/root for resources."""
    for base in (CONFIG_DIR, CALIB_DIR, SRC_ROOT):
        candidate = base / filename
        if candidate.exists():
            return str(candidate)
    return filename


# 控制終端印出頻率（秒），限制約 2 次/秒
_PRINT_INTERVAL = 0.5
_last_robot_print_ts = 0.0
_last_ball_print_ts = 0.0
_last_cam_log_ts = {}
# 角度全域校正（度），可微調；正值順時針，負值逆時針
ANGLE_GLOBAL_OFFSET = 0.0
# 角度眾數計算參數
ANGLE_MODE_WINDOW_SEC = 1.0
ANGLE_MODE_BIN_DEG = 15.0

def _norm_angle_deg(deg: float) -> float:
    """Normalize angle to [0, 360)."""
    return (deg % 360.0 + 360.0) % 360.0


def _circular_mean_deg(angles):
    """Average angles on a circle."""
    if not angles:
        return None
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    if sin_sum == 0 and cos_sum == 0:
        return _norm_angle_deg(angles[0])
    return _norm_angle_deg(math.degrees(math.atan2(sin_sum, cos_sum)))


def _prune_samples(samples: deque, now_ts: float, window_sec: float):
    """Remove samples older than window_sec."""
    while samples and now_ts - samples[0][0] > window_sec:
        samples.popleft()


def _mode_angle_from_samples(samples, bin_size_deg: float):
    """Return mode angle from timestamped samples using circular bins."""
    if not samples:
        return None
    num_bins = max(1, int(round(360.0 / bin_size_deg)))
    counts = {}
    bin_members = {}
    half_bin = bin_size_deg / 2.0
    for _, ang in samples:
        ang_norm = _norm_angle_deg(ang)
        bin_idx = int((ang_norm + half_bin) // bin_size_deg) % num_bins
        counts[bin_idx] = counts.get(bin_idx, 0) + 1
        bin_members.setdefault(bin_idx, []).append(ang_norm)
    best_idx = max(counts, key=lambda k: counts[k])
    return _circular_mean_deg(bin_members.get(best_idx, []))


def _push_angle_sample(robot, cam_role: str, angle_deg: float, ts: float):
    """Append angle to robot's per-camera buffer."""
    if cam_role == "Cam1":
        robot.angle_samples_left.append((ts, _norm_angle_deg(angle_deg)))
    else:
        robot.angle_samples_right.append((ts, _norm_angle_deg(angle_deg)))


def _get_stable_angle(robot, ts: float):
    """Compute stable angle from both cameras within time window."""
    _prune_samples(robot.angle_samples_left, ts, ANGLE_MODE_WINDOW_SEC)
    _prune_samples(robot.angle_samples_right, ts, ANGLE_MODE_WINDOW_SEC)
    combined = list(robot.angle_samples_left) + list(robot.angle_samples_right)
    if not combined:
        return None
    return _mode_angle_from_samples(combined, ANGLE_MODE_BIN_DEG)


def open_camera_device(device_id):
    """Try multiple backends for better cross-platform camera support."""
    system_name = platform.system().lower()
    if 'windows' in system_name:
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    else:
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]

    for backend in backends:
        cap = cv2.VideoCapture(device_id, backend)
        if cap.isOpened():
            return cap
        cap.release()

    return cv2.VideoCapture(device_id)


def _select_camera_pair(preferred_ids=(1, 0), max_scan=6):
    """
    Try to open two cameras.
    1) First try preferred IDs in order.
    2) If fails, scan 0..max_scan-1 to find first two that open.
    Return list of (id, cap) already opened.
    """
    caps = []
    tried = []
    for cid in preferred_ids:
        if cid in tried:
            continue
        tried.append(cid)
        cap = open_camera_device(cid)
        if cap.isOpened():
            caps.append((cid, cap))
        else:
            cap.release()
        if len(caps) >= 2:
            return caps

    for cid in range(max_scan):
        if cid in tried:
            continue
        cap = open_camera_device(cid)
        if cap.isOpened():
            caps.append((cid, cap))
        else:
            cap.release()
        if len(caps) >= 2:
            break
    return caps

# ==========================================
# [全域常數] 場地與視窗設定
# ==========================================
REAL_WIDTH = 360.0      # 場地實際寬度 (cm)
REAL_HEIGHT = 260.0     # 場地實際高度 (cm)
WINDOW_WIDTH = 720      # 視窗寬度 (pixel)
WINDOW_HEIGHT = 520     # 視窗高度 (pixel)
ROBOT_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# Runtime log control
SUPPRESS_BALL_MASK_LOGS = False


# ==========================================
# [類別定義] Ball - 球體追蹤系統（含多組 HSV 測試與手動半徑調整）
# ==========================================
class Ball():    
    def __init__(self, diameter_cm=None, size_tolerance=0.2):
        """
        初始化球體物件
        :param diameter_cm: 球體實際直徑 (cm)。None 時不做半徑檢查，只靠 HSV+圓形偵測
        :param size_tolerance: 半徑容許誤差 (0.5 代表 ±50%)；diameter_cm 為 None 時忽略
        """
        self.diameter_cm = diameter_cm
        self.size_tolerance = size_tolerance
        self.hsv_range = None  # 儲存 (lower, upper)
        
        # 目前的狀態
        self.center_px = None  # (x, y) 像素座標
        self.center_real = None # (x, y) 實際 cm 座標
        self.is_tracked = False
        self.radius_px = 0

        # 取樣測試候選相關
        self.candidate_ranges = None  # list of (lower, upper, delta_scale)
        self.candidate_idx = None
        self._candidate_start_ts = None
        self._candidate_global_start_ts = None
        self._candidate_watchdog = None
        self._candidate_positions = None
        self._last_log_ts = 0.0  # 節流列印
        self.manual_radius_px = None  # 手動指定顯示半徑
        self.reference_template = None
        self.reference_point = None
        self.last_center_px = None
        self.last_center_real = None
        self.last_radius_px = 0

        # 低通濾波 (位置) - 5Hz
        self._lp_last_ts = None
        self._lp_center = None

        # 快取常用影像處理元件，避免每幀重建
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self._morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        # 計算期望的像素半徑
        self.expected_radius_px = self._calculate_expected_pixel_radius() if diameter_cm is not None else None
        
        # 顯示初始化資訊
        if self.expected_radius_px is not None:
            print(f"[Ball] 初始化: 直徑={self.diameter_cm}cm, 期望半徑={self.expected_radius_px:.1f}px")
        else:
            print(f"[Ball] 初始化: 未設定直徑，將僅依 HSV+圓形偵測")
        
    def _calculate_expected_pixel_radius(self):
        """內部方法:將實際 cm 換算成 pixel 半徑"""
        scale_x = WINDOW_WIDTH / REAL_WIDTH
        scale_y = WINDOW_HEIGHT / REAL_HEIGHT
        px_per_cm = (scale_x + scale_y) / 2.0
        return (self.diameter_cm / 2.0) * px_per_cm


    def set_hsv_range_from_selection(self, image, x, y, range_h=1):
        """根據滑鼠點擊位置周圍的相似像素計算多組 HSV 候選，逐一測試取最穩定者"""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        x_start = max(x - range_h, 0)
        x_end = min(x + range_h + 1, hsv_image.shape[1])
        y_start = max(y - range_h, 0)
        y_end = min(y + range_h + 1, hsv_image.shape[0])
        
        region = hsv_image[y_start:y_end, x_start:x_end]
        if region.size == 0:
            return

        # 以中心像素為基準，建立多組 delta 供測試
        center_hsv = hsv_image[min(max(y, 0), hsv_image.shape[0]-1),
                               min(max(x, 0), hsv_image.shape[1]-1)]
        base_delta = np.array([15, 60, 60])  # 原始相近容許值
        delta_scales = [0.5, 0.75, 1.0, 1.25, 1.5]  # 基準及 ±50% 之間共五組
        candidates = []
        for scale in delta_scales:
            delta = (base_delta * scale).astype(int)
            similar_mask = np.all(np.abs(region - center_hsv) <= delta, axis=2)
            valid_pixels = region[similar_mask]
            if valid_pixels.size == 0:
                valid_pixels = region.reshape(-1, 3)

            lower = np.maximum(valid_pixels.min(axis=0) - np.array([10, 30, 30]), [0, 0, 0])
            upper = np.minimum(valid_pixels.max(axis=0) + np.array([10, 30, 30]), [179, 255, 255])
            candidates.append((lower.astype(int), upper.astype(int), scale))

        if not candidates:
            return

        self._start_candidate_test(candidates)
        print(f"[Ball] HSV 取樣啟動，共 {len(candidates)} 組 delta_scale: {delta_scales}")

        # 在畫面上短暫顯示取樣框，非阻塞式
        try:
            preview = image.copy()
            cv2.rectangle(preview, (x_start, y_start), (x_end - 1, y_end - 1), (0, 255, 255), 2)
            cv2.imshow("Fused View + Ball Track", preview)
            cv2.waitKey(1)
        except cv2.error:
            pass


    def _start_candidate_test(self, candidates):
        """開始多組 HSV 候選測試，每組停留約 3 秒，選擇最穩定者"""
        self.candidate_ranges = candidates
        self.candidate_idx = 0
        self._candidate_start_ts = time.time()
        self._candidate_global_start_ts = self._candidate_start_ts
        self._candidate_positions = [[] for _ in candidates]
        # 背景監視，避免候選流程卡住
        if self._candidate_watchdog is None or not self._candidate_watchdog.is_alive():
            self._candidate_watchdog = threading.Thread(target=self._candidate_tick, daemon=True)
            self._candidate_watchdog.start()
        # 先套用第一組
        self.hsv_range = (candidates[0][0], candidates[0][1])
        print(f"[Ball] 候選開始 1/{len(candidates)}，delta_scale={candidates[0][2]:.2f}")

    def _candidate_tick(self):
        """背景定時推進候選流程"""
        while self.candidate_ranges is not None and self.candidate_idx is not None:
            self._advance_candidate_if_needed()
            time.sleep(0.5)

    def _record_candidate_position(self):
        """記錄目前候選的偵測位置，供穩定度評分"""
        if (
            self.candidate_ranges is None
            or self.candidate_idx is None
            or self._candidate_positions is None
            or not self.is_tracked
            or self.center_px is None
        ):
            return
        self._candidate_positions[self.candidate_idx].append(np.array(self.center_px, dtype=float))

    def _advance_candidate_if_needed(self):
        """每組測試約 3 秒，結束後切換下一組並最終選出最佳"""
        if self.candidate_ranges is None or self.candidate_idx is None:
            return
        now = time.time()
        if now - self._candidate_start_ts < 3.0:
            return

        # 全流程超時，防止卡在候選
        if self._candidate_global_start_ts is not None:
            total_elapsed = now - self._candidate_global_start_ts
            if total_elapsed > 3.0 * len(self.candidate_ranges) + 2.0:
                self.candidate_idx = len(self.candidate_ranges)

        self.candidate_idx += 1
        if self.candidate_idx >= len(self.candidate_ranges):
            # 評分並選最穩定
            best_idx = None
            best_score = float('inf')
            for idx, pts in enumerate(self._candidate_positions):
                if len(pts) == 0:
                    score = float('inf')
                else:
                    pts_arr = np.vstack(pts)
                    mean = pts_arr.mean(axis=0)
                    score = np.linalg.norm(pts_arr - mean, axis=1).mean()
                if score < best_score:
                    best_score = score
                    best_idx = idx
            if best_idx is not None and best_score < float('inf'):
                chosen = self.candidate_ranges[best_idx]
                self.hsv_range = (chosen[0], chosen[1])
                print(f"\n\n[Ball] 完成測試，最佳候選 {best_idx+1}/{len(self.candidate_ranges)}，delta_scale={chosen[2]:.2f}，平均偏移={best_score:.2f}px\n\n")
            else:
                # 沒有任何有效偵測，保留最後一組
                last = self.candidate_ranges[-1]
                self.hsv_range = (last[0], last[1])
                print("[Ball] 沒有偵測到任何候選，沿用最後一組 HSV")

            # 清空測試狀態
            self.candidate_ranges = None
            self.candidate_idx = None
            self._candidate_positions = None
            self._candidate_start_ts = None
            self._candidate_global_start_ts = None
            return

        # 切到下一組候選
        next_candidate = self.candidate_ranges[self.candidate_idx]
        self.hsv_range = (next_candidate[0], next_candidate[1])
        self._candidate_start_ts = time.time()
        print(f"[Ball] 切換候選 {self.candidate_idx+1}/{len(self.candidate_ranges)}，delta_scale={next_candidate[2]:.2f}")

    def _log_throttled(self, msg, interval=0.3):
        """避免過度洗版的節流列印"""
        if SUPPRESS_BALL_MASK_LOGS and msg.startswith("[Ball][Mask]"):
            return
        now = time.time()
        if now - self._last_log_ts >= interval:
            print(msg)
            self._last_log_ts = now

    def _lowpass_center(self, pt):
        """暫停低通，直接回傳原座標"""
        return pt

    def _use_last_known_position(self, last_px, last_real, last_radius):
        """回退到上一筆成功的球座標"""
        if last_px is None or last_real is None:
            return False
        self.center_px = last_px
        self.center_real = last_real
        if self.radius_px == 0 and last_radius:
            self.radius_px = last_radius
        return True

    # ========= 手動半徑調整 =========
    def adjust_manual_radius(self, delta):
        """手動增減顯示半徑 (px)，若尚未設定則從當前偵測半徑或 10px 起算"""
        if self.manual_radius_px is None:
            base = self.radius_px if self.radius_px > 0 else 10
            self.manual_radius_px = max(1, int(base + delta))
        else:
            self.manual_radius_px = max(1, int(self.manual_radius_px + delta))
        print(f"[Ball] 手動半徑= {self.manual_radius_px}px")

    def reset_manual_radius(self):
        """清除手動半徑，回復使用偵測到的半徑"""
        self.manual_radius_px = None
        print("[Ball] 手動半徑清除，改用偵測值")


    def track(self, image):
        """
        在輸入影像中追蹤球體
        :param image: 輸入影像 (通常是鳥瞰圖)
        :return: 處理後的影像 (畫上追蹤結果), mask, 是否追蹤成功
        """
        if self.hsv_range is None:
            self.is_tracked = False
            empty_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            return image, empty_mask, False

        last_px = self.last_center_px
        last_real = self.last_center_real
        last_radius = self.last_radius_px
        fallback_used = False

        # 影像前處理
        enhanced = self._apply_clahe(image)
        hsv_img = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        
        result_img = image.copy()
        self.is_tracked = False
        self.center_px = None
        self.center_real = None

        # 產生 Mask
        mask = cv2.inRange(hsv_img, self.hsv_range[0], self.hsv_range[1])
        # 針對小目標：縮小 kernel 與次數，避免侵蝕掉 10px 的球
        kernel = self._morph_kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        blurred = cv2.GaussianBlur(mask, (5,5), 1)
        nz = np.count_nonzero(mask)
        self._log_throttled(f"[Ball][Mask] nonzero={nz}/{mask.size} ({nz/mask.size:.4f})")

        # 若遮罩內無有效像素，略過 Hough 以節省運算
        if np.count_nonzero(blurred) == 0:
            self._log_throttled("[Ball][Mask] zero pixels after blur, skip Hough")
            self._record_candidate_position()
            self._advance_candidate_if_needed()
            fallback_used = self._use_last_known_position(last_px, last_real, last_radius)
            return result_img, mask, fallback_used


        # Hough Circle 偵測
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.0, minDist=15,
            param1=40, param2=10,
            minRadius=1, maxRadius=40  # 10px 球優先，縮小上限加速
        )

        if circles is None:
            self._log_throttled("[Ball][Hough] no circles found")
            self._record_candidate_position()
            self._advance_candidate_if_needed()
            fallback_used = self._use_last_known_position(last_px, last_real, last_radius)
            return result_img, mask, fallback_used

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for (x, y, r) in circles[0, :]:
                ok_size = self._is_size_valid(r)
                self._log_throttled(f"[Ball][Hough] cand x={x} y={y} r={r} size_ok={ok_size}")
                # 檢查尺寸是否符合預期
                if self._is_size_valid(r):
                    self.center_px = self._lowpass_center((int(x), int(y)))
                    self.radius_px = int(r)
                    self.is_tracked = True
                    
                    # 計算實際座標
                    self._calculate_real_coordinates()
                    self.last_center_px = self.center_px
                    self.last_center_real = self.center_real
                    self.last_radius_px = self.radius_px
                    
                    # 繪圖
                    self._draw_info(result_img)
                    break # 找到一個符合的就跳出

        # 記錄候選穩定度，並視需要切換到下一組候選
        self._record_candidate_position()
        self._advance_candidate_if_needed()

        if not self.is_tracked:
            fallback_used = self._use_last_known_position(last_px, last_real, last_radius)

        return result_img, mask, self.is_tracked or fallback_used


    def _is_size_valid(self, radius):
        """檢查半徑是否在容許誤差範圍內；未設定直徑時直接通過"""
        if self.expected_radius_px is None:
            return True
        r_min = self.expected_radius_px * (1.0 - self.size_tolerance)
        r_max = self.expected_radius_px * (1.0 + self.size_tolerance)
        # 確保至少大於 1px
        if r_min < 1: r_min = 1
        ok = r_min <= radius <= r_max
        if not ok:
            self._log_throttled(f"[Ball][SizeFilter] radius={radius:.1f}, allow=({r_min:.1f},{r_max:.1f})")
        return ok


    def _calculate_real_coordinates(self):
        """像素座標轉實際座標"""
        if self.center_px:
            px, py = self.center_px
            real_x = px * (REAL_WIDTH / WINDOW_WIDTH)
            real_y = py * (REAL_HEIGHT / WINDOW_HEIGHT)
            self.center_real = (real_x, real_y)


    def _draw_info(self, image):
        """在影像上繪製追蹤資訊"""
        if not self.is_tracked: return
        
        cx, cy = self.center_px
        rx, ry = self.center_real
        display_r = self.manual_radius_px if self.manual_radius_px is not None else self.radius_px
        
        # 畫圓與中心點
        cv2.circle(image, (cx, cy), display_r, (0, 255, 0), 2)
        cv2.circle(image, (cx, cy), 2, (0, 0, 255), 3)
        
        # 顯示資訊文字
        info_text = f"Ball: ({rx:.1f}, {ry:.1f}) cm"
        radius_text = f"r={display_r}px"
        
        # 左上角顯示座標
        cv2.putText(image, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # 球體旁顯示半徑
        cv2.putText(image, radius_text, (cx + 15, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)


    def _apply_clahe(self, image):
        """內部工具:CLAHE 增強"""
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        cl = self._clahe.apply(l)
        merged = cv2.merge((cl, a, b))
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)


# ==========================================
# [類別定義] Robot - 機器人追蹤系統
# ==========================================
class Robot():
    def __init__(self, id, aruco_id_list, aruco_x_list, aruco_y_list, aruco_degree_list, camera_setting="right"):
        # 原有的 Robot 基本屬性
        self.id = id 
        self.aruco_id_list = aruco_id_list
        self.aruco_x_list = aruco_x_list
        self.aruco_y_list = aruco_y_list
        self.aruco_degree_list = aruco_degree_list
        self.x = 0
        self.y = 0
        self.degree = 0
        self.degree_base = 0
        self.x_left, self.x_right, self.y_left, self.y_right = 0, 0, 0, 0
        self.degree_left, self.degree_right = 0, 0
        self.degree_base_left, self.degree_base_right = 0, 0
        
        # 新增: 向量資訊
        self.vec_x = 0.0
        self.vec_y = 0.0
        self.vec_x_left = 0.0
        self.vec_x_right = 0.0
        self.vec_y_left = 0.0
        self.vec_y_right = 0.0
        # 180 度跳變計數
        self.flip_count_left = 0
        self.flip_count_right = 0
        # 角度緩衝區（依相機分開，取眾數避免 180 翻轉）
        self.angle_samples_left = deque(maxlen=80)
        self.angle_samples_right = deque(maxlen=80)
        self.stable_angle = None
        
        self.camera_setting = camera_setting
        self.amount_choose_left = 0
        self.amount_choose_right = 0 
        self.is_enemy = False
        # 以第一個 ID 為頂部參考面
        self.top_id = aruco_id_list[0] if aruco_id_list else None
        self.face_order = ["top", "front", "left", "back", "right"]
        self.id_to_face = {}
        for idx, aid in enumerate(aruco_id_list):
            if idx < len(self.face_order):
                self.id_to_face[aid] = self.face_order[idx]
        # 角度連續性追蹤
        self.last_angle_by_marker = {aid: (None, None) for aid in aruco_id_list}  # id -> (angle, ts)
        self.angle_history = []  # list of (ts, angle) for mode voting
        self.angle_mode_samples = []  # list of (ts, angle_int) for 0.5s mode
        self.top_angle_ref = None
        
        # ==========================================
        # 雙相機追蹤系統 - 全域設定
        # ==========================================
        self.CAM1_ID = 0
        self.CAM2_ID = 0
        self.MARKER_SIZE = 0.16  # 標記大小通常是一樣的
        
        # ==========================================
        # 相機 1 專屬設定與參數
        # ==========================================
        self.CONFIG_CAM1 = {
            'calib_file': 'calib_cam0.npz',
            'camera_height': 2.2,
            'robot_height': 0.5,
            'offset_map': {
                10: {'x_shift': 0, 'y_shift': 0},
                20: {'x_shift': 0, 'y_shift': 0},
                0: {'x_shift': 0, 'y_shift': 0},
            }
        }
        
        # ==========================================
        # 相機 2 專屬設定與參數
        # ==========================================
        self.CONFIG_CAM2 = {
            'calib_file': 'calib_cam2.npz',
            'camera_height': 2.2,
            'robot_height': 0.5,
            'offset_map': {
                10: {'x_shift': 0.0, 'y_shift': 0.0},
                20: {'x_shift': 0.0, 'y_shift': 0.0},
                0: {'x_shift': 0.0, 'y_shift': 0.0},
            }
        }
        
        # ==========================================
        # 內部類別定義 - CameraStream
        # ==========================================
        class CameraStream:
            def __init__(self, src=0, name="Camera"):
                self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
                if not self.stream.isOpened():
                    self.stream = cv2.VideoCapture(src)
                
                self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                self.stream.set(cv2.CAP_PROP_FOURCC, fourcc)
                
                self.name = name
                self.stopped = not self.stream.isOpened()
                self.frame = None
                if not self.stopped:
                    print(f"成功開啟 {name} (ID: {src})")
                else:
                    print(f"錯誤 無法開啟 {name}")
            
            def start(self):
                t = threading.Thread(target=self.update, args=())
                t.daemon = True
                t.start()
                return self
            
            def update(self):
                while True:
                    if self.stopped: return
                    grabbed, frame = self.stream.read()
                    if grabbed:
                        self.frame = frame
                    else:
                        self.stopped = True
            
            def read(self):
                return self.frame
            
            def stop(self):
                self.stopped = True
                self.stream.release()
        
        self.CameraStream = CameraStream
        
        # ==========================================
        # 工具函數定義
        # ==========================================
        def load_calibration(calib_file):
            try:
                data = np.load(_resource_path(calib_file))
                return data['mtx'], data['dist']
            except FileNotFoundError:
                print(f"警告 找不到 {calib_file},使用預設參數")
                return np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32), np.zeros((5,), dtype=np.float32)
        
        def initialize_detector():
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
            p = cv2.aruco.DetectorParameters()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG  # 開啟角點細化
            p.cornerRefinementWinSize = 5
            p.cornerRefinementMaxIterations = 50
            p.cornerRefinementMinAccuracy = 0.01
            p.adaptiveThreshWinSizeMin = 3
            p.adaptiveThreshWinSizeMax = 33
            p.adaptiveThreshWinSizeStep = 8
            p.adaptiveThreshConstant = 5
            p.minMarkerPerimeterRate = 0.01  # 允許更小的標記
            p.maxMarkerPerimeterRate = 6.0
            p.minCornerDistanceRate = 0.01
            p.minOtsuStdDev = 2.0
            p.perspectiveRemoveIgnoredMarginPerCell = 0.12
            p.errorCorrectionRate = 0.7
            return cv2.aruco.ArucoDetector(d, p)
        
        self.load_calibration = load_calibration
        self.initialize_detector = initialize_detector
        
        self.mtx1, self.dist1 = load_calibration(self.CONFIG_CAM1['calib_file'])
        self.mtx2, self.dist2 = load_calibration(self.CONFIG_CAM2['calib_file'])
        self.detector = initialize_detector()


# ==========================================
# [獨立工具函數] - 機器人追蹤處理管線
# ==========================================
def process_robot_pipeline(frame, detector, mtx, dist, target_ids, cam_role, robots_dict, config_cam1, config_cam2, marker_size, pre_undist=None):
    """
    處理機器人追蹤的完整流程
    """
    if frame is None: return None
    
    # 只去畸變,不旋轉
    frame_undist = pre_undist if pre_undist is not None else cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2GRAY)
    def detect(gray_img):
        return detector.detectMarkers(gray_img)

    # 多步影像增強與放大，以提升小標記偵測率
    corners, ids, _ = detect(gray)
    if ids is None:
        gray_eq = ROBOT_CLAHE.apply(gray)
        corners, ids, _ = detect(gray_eq)
    if ids is None:
        gray_smooth = cv2.GaussianBlur(gray, (3, 3), 0)
        corners, ids, _ = detect(gray_smooth)
    if ids is None:
        sharp = cv2.addWeighted(gray, 1.5, cv2.GaussianBlur(gray, (0, 0), 3), -0.5, 0)
        corners, ids, _ = detect(sharp)
    if ids is None:
        gray_med = cv2.medianBlur(gray, 3)
        corners, ids, _ = detect(gray_med)
    if ids is None:
        gray_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 7)
        corners, ids, _ = detect(gray_thresh)
    if ids is None:
        gray_clipped = np.minimum(gray, 200).astype(np.uint8)
        corners, ids, _ = detect(gray_clipped)
    if ids is None:
        gamma = 1.4
        gray_gamma = np.clip(((gray / 255.0) ** gamma) * 255.0, 0, 255).astype(np.uint8)
        corners, ids, _ = detect(gray_gamma)
    if ids is None:
        # 最後手段：放大影像，提高小標記佔比
        scale_factor = 1.6
        gray_up = cv2.resize(gray, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
        corners_up, ids_up, _ = detect(gray_up)
        if ids_up is not None:
            corners = [c / scale_factor for c in corners_up]
            ids = ids_up
    
    # 建立物件點
    h = marker_size / 2.0
    obj_pts = np.array([[-h,h,0], [h,h,0], [h,-h,0], [-h,-h,0]], dtype=np.float32)
    info_lines_left = []  # 收集左上角要顯示的文字
    pad_top = 80
    pad_left = 220
    pad_right = 220
    
    if ids is not None:
        for i, mid in enumerate(ids.flatten()):
            raw_fX = raw_fY = None  # default for logging when no coord
            if mid not in target_ids: continue
            
            success, rvec, tvec = cv2.solvePnP(obj_pts, corners[i], mtx, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            # 固定 z 為正值，避免因高度為負造成姿態 180 度翻轉；同時固定 rvec.z 避免跳動
            if success and tvec is not None and tvec.shape[0] >= 3:
                tvec = tvec.copy()
                tvec[2] = abs(float(tvec[2]))
                if tvec[2] < 0.01:
                    tvec[2] = 0.01
            if success and rvec is not None and rvec.shape[0] >= 3:
                rvec = rvec.copy()
                rvec[2] = abs(float(rvec[2]))
                if rvec[2] < 1e-6:
                    rvec[2] = 1e-6
            
            if success:
                now_ts = time.time()
                R, _ = cv2.Rodrigues(rvec)
                
                # 計算物理座標
                fX, fY = None, None
                mapped_fX = mapped_fY = None  # 供顯示用的映射座標
                raw_fX, raw_fY = _map_tvec_to_field(tvec, cam_role, config_cam1, config_cam2, mid)
                mapped_fX, mapped_fY = raw_fX, raw_fY
                R_top = R
                tvec_top = tvec
                rvec_top = rvec
                cos_theta_top = None
                if mid in robots_dict:
                    robot = robots_dict[mid]
                    R_top, tvec_top, _, _, cos_theta_top = _compute_top_pose(R, tvec, obj_pts, robot, mid)
                    rvec_top, _ = cv2.Rodrigues(R_top)
                    mapped_fX, mapped_fY = _map_tvec_to_field(tvec_top, cam_role, config_cam1, config_cam2, mid)
                fX, fY = mapped_fX, mapped_fY
                
                # 角度：直接用對齊後的 R_top 提取前方向，避免依賴 tvec 造成的視差
                raw_angle = _compute_angle_from_R(R_top)
                if cam_role == "Cam1":
                    raw_angle = _norm_angle_deg(-raw_angle)
                angle = raw_angle
                if mid in robots_dict:
                    robot = robots_dict[mid]
                    # 動態校準：頂部標記且視角夠正時更新參考值（漸進式）
                    if robot.top_id is not None and mid == robot.top_id:
                        if cos_theta_top is not None and cos_theta_top > 0.8:
                            robot.top_angle_ref = _blend_angle(robot.top_angle_ref, raw_angle, alpha=0.1)
                    # 角度補償：沒有參考就用當前 yaw；有參考但偏差過大時放棄參考
                    base_angle = raw_angle
                    if robot.top_angle_ref is not None:
                        diff = abs(_norm_angle_deg(robot.top_angle_ref - raw_angle))
                        diff = min(diff, 360.0 - diff)
                        if diff < 15.0:
                            base_angle = robot.top_angle_ref
                        else:
                            base_angle = raw_angle
                    angle = _mode_angle(robot, base_angle, now_ts)
                else:
                    angle = raw_angle

                # 向量直接由 R_top 取得前方向，簡化處理（不做相機別反轉/跳變修正）
                forward_vec = R_top[:, 1]
                norm_f = math.hypot(forward_vec[0], forward_vec[1]) or 1.0
                vec_x = forward_vec[1] / norm_f
                vec_y = forward_vec[0] / norm_f
                # 相機別微調：Cam2 反轉 x，Cam1 反轉 y
                if cam_role == "Cam2":
                    vec_x = -vec_x
                elif cam_role == "Cam1":
                    vec_y = -vec_y
                # 左側記錄相機 vec（限頻）
                key = (cam_role, mid)
                last_ts = _last_cam_log_ts.get(key, 0.0)
                if now_ts - last_ts >= 0.5:
                    _last_cam_log_ts[key] = now_ts
                    print(f"[{cam_role}] ID:{mid} Ang:{angle:.1f} Vec:({vec_x:.2f},{vec_y:.2f}) X:{mapped_fX if mapped_fX is not None else 'N/A'} Y:{mapped_fY if mapped_fY is not None else 'N/A'}")

                # 更新對應的 Robot 物件 (不管 fX 是否為 None 都更新向量和角度)
                if mid in robots_dict:
                    robot = robots_dict[mid]
                    if cam_role == "Cam1":
                        if fX is not None and fY is not None:
                            robot.x_left = fX
                            robot.y_left = fY
                        robot.degree_left = angle
                        robot.degree_base_left = raw_angle
                        robot.vec_x_left = vec_x
                        robot.vec_y_left = vec_y
                        robot.amount_choose_left += 1
                        _push_angle_sample(robot, cam_role, angle, now_ts)
                    else:  # Cam2
                        if fX is not None and fY is not None:
                            robot.x_right = fX
                            robot.y_right = fY
                        robot.degree_right = angle
                        robot.degree_base_right = raw_angle
                        robot.vec_x_right = vec_x
                        robot.vec_y_right = vec_y
                        robot.amount_choose_right += 1
                        _push_angle_sample(robot, cam_role, angle, now_ts)
                    
                    # 根據機器人位置動態選擇相機 (離哪台相機近就用哪台)
                    # 場地中心 x = REAL_WIDTH / 2 = 180，左半場用 Cam1，右半場用 Cam2
                    field_center_x = REAL_WIDTH / 2.0  # 180.0
                    HYSTERESIS = 5.0  # 中線 ±20cm 的緩衝區，避免邊界頻繁切換
                    
                    # 先取得兩側相機的 x 座標來判斷機器人位置
                    ref_x = None
                    if robot.amount_choose_left > 0 and robot.amount_choose_right > 0:
                        # 兩台相機都有資料，取平均座標來判斷位置
                        ref_x = (robot.x_left + robot.x_right) / 2.0
                    elif robot.amount_choose_left > 0:
                        ref_x = robot.x_left
                    elif robot.amount_choose_right > 0:
                        ref_x = robot.x_right
                    
                    # 根據位置決定使用哪台相機 (含滯後機制)
                    use_left_cam = getattr(robot, '_last_cam_choice', True)  # 預設/維持上次選擇
                    if ref_x is not None:
                        if ref_x < (field_center_x - HYSTERESIS):
                            # 明確在左半場 (x < 160)
                            use_left_cam = True
                        elif ref_x > (field_center_x + HYSTERESIS):
                            # 明確在右半場 (x > 200)
                            use_left_cam = False
                        # else: 在緩衝區 [160, 200] 內，維持上一次的選擇
                    robot._last_cam_choice = use_left_cam  # 記錄本次選擇
                    
                    if use_left_cam:
                        # 機器人在左半場，優先使用左側相機 (Cam1)
                        if robot.amount_choose_left > 0:
                            robot.x = robot.x_left
                            robot.y = robot.y_left
                            robot.degree = robot.degree_left
                            robot.degree_base = robot.degree_base_left
                            robot.vec_x = robot.vec_x_left
                            robot.vec_y = robot.vec_y_left
                        elif robot.amount_choose_right > 0:  # fallback to right
                            robot.x = robot.x_right
                            robot.y = robot.y_right
                            robot.degree = robot.degree_right
                            robot.degree_base = robot.degree_base_right
                            robot.vec_x = robot.vec_x_right
                            robot.vec_y = robot.vec_y_right
                    else:
                        # 機器人在右半場，優先使用右側相機 (Cam2)
                        if robot.amount_choose_right > 0:
                            robot.x = robot.x_right
                            robot.y = robot.y_right
                            robot.degree = robot.degree_right
                            robot.degree_base = robot.degree_base_right
                            robot.vec_x = robot.vec_x_right
                            robot.vec_y = robot.vec_y_right
                        elif robot.amount_choose_left > 0:  # fallback to left
                            robot.x = robot.x_left
                            robot.y = robot.y_left
                            robot.degree = robot.degree_left
                            robot.degree_base = robot.degree_base_left
                            robot.vec_x = robot.vec_x_left
                            robot.vec_y = robot.vec_y_left
                    
                    # 角度眾數：合併兩相機一段時間內的角度，取最常見值
                    stable_angle = _get_stable_angle(robot, now_ts)
                    if stable_angle is not None:
                        robot.stable_angle = stable_angle
                        robot.degree_base = stable_angle
                        robot.degree = stable_angle
                
                # 繪圖顯示
                cv2.aruco.drawDetectedMarkers(frame_undist, [corners[i]])
                c = tuple(corners[i][0][0].astype(int))

                # 在標記前方（藍色 Z 軸方向）顯示 ID：將 marker 座標系中的 (0,0,offset) 投影到影像
                try:
                    label_offset = marker_size * 0.8  # 依標記尺寸往前推
                    label_3d = np.array([[[0.0, 0.0, label_offset]]], dtype=np.float32)
                    proj, _ = cv2.projectPoints(label_3d, rvec, tvec, mtx, dist)
                    lx, ly = proj[0,0,0], proj[0,0,1]
                    # 小抖動避免多個 ID 疊在一起
                    jitter_dx = (i % 4) * 12
                    jitter_dy = (i // 4) * 12
                    cv2.putText(frame_undist, f"ID:{mid}", (int(lx) + jitter_dx, int(ly) - 10 - jitter_dy), 0, 0.6, (0,255,255), 2)
                except cv2.error:
                    pass

                # 收集左上角資訊，包含原始/映射座標（映射不影響 Robot 主座標）
                info_block = [
                    (f"[{cam_role}] ID:{mid}", (0,255,255)),
                    (f"AngBase:{angle:.1f}", (0,255,0)),
                    (f"AngRaw:{raw_angle:.1f}", (0,180,255)),
                    (f"Vec:({vec_x:.2f},{vec_y:.2f})", (200,255,200)),
                    (f"X:{mapped_fX:.1f}" if mapped_fX is not None else "X:N/A", (0,255,255) if mapped_fX is not None else (0,0,255)),
                    (f"Y:{mapped_fY:.1f}" if mapped_fY is not None else "Y:N/A", (0,255,255) if mapped_fY is not None else (0,0,255)),
                ]
                if raw_fX is not None and raw_fY is not None:
                    info_block.append((f"RawX:{raw_fX:.1f}", (255,200,0)))
                    info_block.append((f"RawY:{raw_fY:.1f}", (255,200,0)))
                info_lines_left.append(info_block)
                
                cv2.drawFrameAxes(frame_undist, mtx, dist, rvec, tvec, 0.03)
                
                # Console 輸出（限頻 2 次/秒）
                if mid in robots_dict:
                    now_ts = time.time()
                    if now_ts - _last_robot_print_ts >= _PRINT_INTERVAL:
                        globals()['_last_robot_print_ts'] = now_ts
                        robot_id = robots_dict[mid].id
                        r = robots_dict[mid]
                        print(f"[{cam_role}] Robot{robot_id} (ID:{mid}) | X={r.x:.1f}, Y={r.y:.1f} | Angle={r.degree:.1f}° | Vec:({r.vec_x:.2f},{r.vec_y:.2f})")
    
    # 為文字顯示加上上方與右側空間
    frame_disp = cv2.copyMakeBorder(frame_undist, pad_top, 0, pad_left, pad_right, cv2.BORDER_CONSTANT, value=(0,0,0))

    # 左上角資訊
    y_anchor = 20
    line_h = 18
    for block in info_lines_left:
        for idx, (text, color) in enumerate(block):
            cv2.putText(frame_disp, text, (10, y_anchor + idx*line_h), 0, 0.6 if idx==0 else 0.55, color, 2)
        y_anchor += line_h * len(block)

    # 右上角顯示各機器人最新主狀態 (Robot[id] X Y Angle Vec)
    robots_unique = {}
    for r in robots_dict.values():
        robots_unique[r.id] = r
    x_right = frame_undist.shape[1] + pad_left + 10
    y_right = 20
    for rid in sorted(robots_unique.keys()):
        r = robots_unique[rid]
        lines = [
            f"Robot{r.id}",
            f"X:{r.x:.1f} Y:{r.y:.1f}",
            f"Ang:{getattr(r, 'degree_base', r.degree):.1f}",
            f"Vec:({r.vec_x:.2f},{r.vec_y:.2f})",
        ]
        for idx, text in enumerate(lines):
            cv2.putText(frame_disp, text, (x_right, y_right + idx*line_h), 0, 0.6 if idx==0 else 0.55, (200,200,0), 2)
        y_right += line_h * len(lines) + 8

    return frame_disp


# ==========================================
# [相機管理工具函數]
# ==========================================
def load_calibration_params(filename):
    """載入相機校正參數"""
    try:
        data = np.load(_resource_path(filename))
        return data['mtx'], data['dist']
    except:
        return np.eye(3), np.zeros(5)


def build_undistort_maps(mtx, dist, frame_shape):
    """預先建立去畸變 remap，加速每幀處理。"""
    h, w = frame_shape[:2]
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, new_mtx, (w, h), cv2.CV_16SC2)
    return map1, map2


def undistort_image(frame, mtx, dist, rotate_direction=None, maps=None):
    """去除影像畸變並可選旋轉"""
    if maps is not None:
        map1, map2 = maps
        undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
    else:
        h, w = frame.shape[:2]
        new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)
    if rotate_direction == 'clockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_CLOCKWISE)
    elif rotate_direction == 'counterclockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return undistorted


def sort_points(pts):
    """排序四個角點為標準順序"""
    pts = np.array(pts)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    return np.array([pts[np.argmin(s)], pts[np.argmin(diff)], 
                     pts[np.argmax(s)], pts[np.argmax(diff)]], dtype=np.float32)


def get_perspective_matrix(pts, width, height):
    """依四點座標建立透視轉換矩陣。"""
    pts_sorted = sort_points(pts)
    dst = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype=np.float32)
    return cv2.getPerspectiveTransform(pts_sorted, dst)


def get_perspective_warp(image, pts, width, height):
    """透視變換為鳥瞰圖"""
    M = get_perspective_matrix(pts, width, height)
    return cv2.warpPerspective(image, M, (width, height))


def draw_points(image, points):
    """在影像上標記選定的點"""
    for pt in points:
        cv2.circle(image, pt, 5, (0,0,255), -1)


def _normalize_angle(deg):
    return (deg + 360.0) % 360.0


def _rot_x(deg):
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]], dtype=np.float32)


def _rot_y(deg):
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]], dtype=np.float32)


def _rot_z(deg):
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]], dtype=np.float32)


_FACE_R_ALIGN = {
    "top": np.eye(3, dtype=np.float32),
    "front": _rot_x(90),
    "back": _rot_x(-90),
    "right": _rot_y(-90) @ _rot_x(90),
    "left": _rot_y(90) @ _rot_x(90),
}


def _get_face_rotation(robot, marker_id):
    face = robot.id_to_face.get(marker_id, "top")
    return _FACE_R_ALIGN.get(face, np.eye(3, dtype=np.float32))


def _get_face_yaw_offset(robot, marker_id):
    return robot.face_yaw_offsets.get(marker_id, 0.0)


def _compute_top_pose(R_cam_face, tvec, obj_pts, robot, marker_id):
    """
    將任一面的姿態映射到頂部標記中心與朝向，回傳 (R_top, tvec_top, side_len, n_hat)。
    """
    # 以角點建立局部座標系
    corners_cam = []
    for p in obj_pts:
        pc = (R_cam_face @ p.reshape(3, 1) + tvec).reshape(3)
        corners_cam.append(pc)
    u = corners_cam[1] - corners_cam[0]
    v = corners_cam[3] - corners_cam[0]
    x_local = u / max(np.linalg.norm(u), 1e-6)
    y_local = v / max(np.linalg.norm(v), 1e-6)
    z_local = np.cross(x_local, y_local)
    z_norm = np.linalg.norm(z_local)
    if z_norm < 1e-6:
        z_local = R_cam_face[:, 2].reshape(3)
    else:
        z_local = z_local / z_norm

    cam_z = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    cos_theta = abs(float(np.dot(z_local, cam_z)))
    edges = [
        np.linalg.norm(corners_cam[1] - corners_cam[0]),
        np.linalg.norm(corners_cam[2] - corners_cam[1]),
        np.linalg.norm(corners_cam[3] - corners_cam[2]),
        np.linalg.norm(corners_cam[0] - corners_cam[3]),
    ]
    side_len = float(np.mean(edges)) / max(cos_theta, 0.1)

    p_center = sum(corners_cam) / 4.0
    t_center = p_center + (-z_local) * (side_len / 2.0)

    # 以局部軸構成旋轉，再乘對齊矩陣
    R_face_local = np.column_stack((x_local, y_local, z_local)).astype(np.float32)
    R_align = _get_face_rotation(robot, marker_id)
    R_top = R_face_local @ R_align

    n_top_cam = R_top @ np.array([0.0, 0.0, 1.0], dtype=np.float32)
    t_top = t_center + n_top_cam * (side_len / 2.0)
    return R_top, t_top.reshape(3, 1), side_len, z_local, cos_theta


def _map_tvec_to_field(tvec, cam_role, config_cam1, config_cam2, marker_id):
    z_cam = tvec[2][0]
    x_cam = tvec[1][0]
    y_cam = tvec[0][0]
    if cam_role == "Cam1":
        h = config_cam1['camera_height']
        h_robot = config_cam1['robot_height']
        val = (x_cam**2 + z_cam**2) - (h - h_robot)**2
        if val < 0:
            return None, None
        X_raw = math.sqrt(val)
        Y_raw = y_cam
        Xa = (X_raw - 1) * 100
        Ya = (Y_raw + 1.3) * 100
        Xb = -0.000691*Xa**2 - 0.000094*Ya**2 - 0.000593*Xa*Ya + 1.227159*Xa + 0.125918*Ya - 18.666358
        Yb = +0.000421*Xa**2 + 0.000106*Ya**2 - 0.000190*Xa*Ya - 0.058721*Xa + 0.934087*Ya + 26.139245
        return Xb, Yb
    h = config_cam2['camera_height']
    h_robot = config_cam2['robot_height']
    val = (x_cam**2 + z_cam**2) - (h - h_robot)**2
    if val < 0:
        return None, None
    X_raw = math.sqrt(val)
    Y_raw = y_cam
    Xa = 360 - (X_raw - 1) * 100.0
    Ya = (1.3 - Y_raw) * 100.0
    Xb = +0.000450*Xa**2 + 0.000391*Ya**2 + 0.000595*Xa*Ya + 0.639393*Xa - 0.329119*Ya + 77.619781
    Yb = +0.000108*Xa**2 - 0.000156*Ya**2 + 0.000433*Xa*Ya - 0.064196*Xa + 0.862848*Ya +2.782101
    return Xb, Yb


def _angular_diff(a, b):
    """Return smallest signed diff between angles in degrees."""
    return ((a - b + 180.0) % 360.0) - 180.0


def _smooth_angle(robot, marker_id, angle, now_ts, window=2.0):
    """
    若 1 秒內與前一筆差超過 90 度，改用近期眾數角度做平滑。
    """
    angle = _normalize_angle(angle)
    prev_angle, prev_ts = robot.last_angle_by_marker.get(marker_id, (None, None))
    use_angle = angle
    if prev_angle is not None and prev_ts is not None:
        if (now_ts - prev_ts) <= 1.0:
            diff = abs(_angular_diff(angle, prev_angle))
            if diff > 90.0:
                # 取最近 window 秒內的眾數角度
                recent = [round(a, 1) for (t, a) in robot.angle_history if now_ts - t <= window]
                if prev_angle is not None:
                    recent.append(round(prev_angle, 1))
                recent.append(round(angle, 1))
                if recent:
                    counts = {}
                    for a in recent:
                        counts[a] = counts.get(a, 0) + 1
                    dominant = max(counts.items(), key=lambda kv: kv[1])[0]
                    use_angle = float(dominant)
                else:
                    use_angle = prev_angle
    # 更新歷史
    robot.last_angle_by_marker[marker_id] = (use_angle, now_ts)
    robot.angle_history.append((now_ts, use_angle))
    robot.angle_history = [(t, a) for (t, a) in robot.angle_history if now_ts - t <= window]
    return use_angle


def _compute_angle_from_pose(R, tvec):
    y_axis = R[:, 1]
    x_axis = R[:, 0]
    tvec_norm = tvec.flatten() / np.linalg.norm(tvec)
    dot_y = np.dot(tvec_norm, y_axis)
    dot_x = np.dot(tvec_norm, x_axis)
    angle = math.degrees(math.atan2(dot_x, dot_y))
    if angle < 0:
        angle += 360
    angle = (angle + 180) % 360
    return angle


def _compute_angle_from_R(R):
    """
    以對齊後的旋轉矩陣直接取朝向：
    使用 +Y 作為前方，angle = atan2(forward_x, forward_y)。
    """
    forward = R[:, 1]
    ang = math.degrees(math.atan2(forward[0], forward[1]))
    return _normalize_angle(ang)


def _blend_angle(prev_deg, curr_deg, alpha=0.1):
    """角度加權平均，處理環繞 360 的情況。"""
    if prev_deg is None:
        return _normalize_angle(curr_deg)
    prev_rad = math.radians(prev_deg)
    curr_rad = math.radians(curr_deg)
    # 轉向單位向量加權
    x = (1 - alpha) * math.cos(prev_rad) + alpha * math.cos(curr_rad)
    y = (1 - alpha) * math.sin(prev_rad) + alpha * math.sin(curr_rad)
    return _normalize_angle(math.degrees(math.atan2(y, x)))


def _mode_angle(robot, angle, now_ts, window=0.5):
    """0.5 秒內取最多次出現的角度（整度數）"""
    angle_int = int(round(angle))
    robot.angle_mode_samples.append((now_ts, angle_int))
    robot.angle_mode_samples = [(t, a) for (t, a) in robot.angle_mode_samples if now_ts - t <= window]
    counts = {}
    last_seen = {}
    for t, a in robot.angle_mode_samples:
        counts[a] = counts.get(a, 0) + 1
        last_seen[a] = t
    if not counts:
        return angle
    # 票數最多，其次取最近出現者
    best = max(counts.items(), key=lambda kv: (kv[1], last_seen[kv[0]]))[0]
    return float(best)


# ==========================================
# [全域變數]
# ==========================================
points_cam0 = []
points_cam2 = []
_last_saved_points = {"cam0": None, "cam2": None}
CALIB_POINTS_FILE = _resource_path("calibration_points.json")
HSV_CALIB_FILE = _resource_path("calibration_hsv.json")
my_ball = Ball(diameter_cm=None, size_tolerance=0.2)
# 預覽畫面（給 Tk UI 取用）
fused_preview = None
robot_cam0_preview = None
robot_cam2_preview = None
fused_for_mask = None
# 待處理的 HSV 取樣請求（避免在回調裡做重運算而卡畫面）
_pending_hsv_sample = None
# 顯示模式：full -> 全部視窗; compact -> 只顯示 Robot Track & Fused
DISPLAY_MODE = "full"


# ==========================================
# [滑鼠回調函式]
# ==========================================
def mouse_callback_fused(event, x, y, flags, param):
    """融合視窗中點擊設定球體顏色"""
    if event == cv2.EVENT_LBUTTONDOWN:
        if fused_for_mask is None:
            return
        globals()['_pending_hsv_sample'] = (fused_for_mask.copy(), x, y)


def mouse_callback_cam0(event, x, y, flags, param):
    """相機0選點回調"""
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam0) < 4:
        points_cam0.append((x,y))
        print(f"Camera 0 選點: ({x},{y})")


def mouse_callback_cam2(event, x, y, flags, param):
    """相機2選點回調"""
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam2) < 4:
        points_cam2.append((x,y))
        print(f"Camera 2 選點: ({x},{y})")


# ==========================================
# [校正點儲存/載入]
# ==========================================
def _points_equal(a, b):
    return list(map(list, a)) == list(map(list, b))


def _save_default_points(cam0_points, cam2_points):
    """Persist latest 4-point calibration to json for future 'default' use."""
    global _last_saved_points
    data = {
        "points_cam0": [list(pt) for pt in cam0_points],
        "points_cam2": [list(pt) for pt in cam2_points],
    }
    try:
        with open(CALIB_POINTS_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        _last_saved_points = {"cam0": data["points_cam0"], "cam2": data["points_cam2"]}
        print(f"[calibration] Saved default points to {CALIB_POINTS_FILE}")
    except Exception as err:
        print(f"[calibration] Failed to save default points: {err}")


def _save_default_hsv(hsv_range):
    """Persist latest HSV range (and manual radius if有) to json for future 'default' use."""
    if not hsv_range or len(hsv_range) != 2:
        return
    lower, upper = hsv_range
    try:
        radius_val = None
        if my_ball is not None:
            if my_ball.manual_radius_px is not None:
                radius_val = int(my_ball.manual_radius_px)
            elif my_ball.radius_px > 0:
                radius_val = int(my_ball.radius_px)
        data = {
            "lower": np.array(lower, dtype=int).tolist(),
            "upper": np.array(upper, dtype=int).tolist(),
            "manual_radius_px": radius_val,
        }
        with open(HSV_CALIB_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        print(f"[calibration] Saved HSV mask to {HSV_CALIB_FILE} (radius={radius_val})")
    except Exception as err:
        print(f"[calibration] Failed to save HSV mask: {err}")


def _load_default_hsv():
    """Load HSV range (and manual radius) from disk if available."""
    if not os.path.exists(HSV_CALIB_FILE):
        return False
    try:
        with open(HSV_CALIB_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        lower = np.array(data.get("lower"), dtype=np.uint8)
        upper = np.array(data.get("upper"), dtype=np.uint8)
        if lower.size == 0 or upper.size == 0:
            return False
        my_ball.hsv_range = (lower, upper)
        radius_val = data.get("manual_radius_px")
        if radius_val:
            my_ball.manual_radius_px = int(radius_val)
            print(f"[calibration] Loaded HSV mask and radius={radius_val} from {HSV_CALIB_FILE}")
        else:
            print(f"[calibration] Loaded HSV mask from {HSV_CALIB_FILE}")
        return True
    except Exception as err:
        print(f"[calibration] Failed to load HSV mask: {err}")
        return False


def set_display_mode(mode: str):
    """Adjust display mode: 'full' or 'compact' (only fused + robot tracks)."""
    global DISPLAY_MODE
    mode = mode.lower()
    if mode not in ("full", "compact"):
        return
    DISPLAY_MODE = mode


# ==========================================
# [主程式]
# ==========================================
def main():
    global fused_for_mask
    print("=" * 60)
    print("整合系統啟動: 球體追蹤 + 機器人追蹤")
    print("=" * 60)
    
    # ==========================================
    # 1. 建立機器人實例
    # ==========================================
    # 每台機器人各有五個 ArUco ID（上/前/左/後/右），最後被看到的面會覆寫主狀態
    robot_0 = Robot(0, [0, 2, 6, 8, 4],  [0.0]*5, [0.0]*5, [0.0]*5, "right")
    robot_1 = Robot(1, [10, 16, 12, 18, 14], [0.0]*5, [0.0]*5, [0.0]*5, "right")
    robot_2 = Robot(2, [20, 26, 22, 28, 24], [0.0]*5, [0.0]*5, [0.0]*5, "right")
    
    robots = [robot_0, robot_1, robot_2]
    
    print("機器人追蹤系統:")
    for robot in robots:
        print(f"  機器人 {robot.id}: ArUco ID={robot.aruco_id_list}, camera_setting={robot.camera_setting}")
    
    # 收集所有要追蹤的 ArUco ID
    target_ids = []
    for robot in robots:
        target_ids.extend(robot.aruco_id_list)
    
    # 建立 ArUco ID 到 Robot 的映射字典
    robots_dict = {}
    for robot in robots:
        for aruco_id in robot.aruco_id_list:
            robots_dict[aruco_id] = robot
    
    print(f"追蹤目標 ID: {target_ids}")
    
    # ==========================================
    # 2. 載入相機校正參數 (共用)
    # ==========================================
    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')
    
    # 使用第一個機器人的 ArUco 偵測器
    detector = robot_0.detector
    
    # ==========================================
    # 3. 開啟相機
    # ==========================================
    cap0 = open_camera_device(robot_0.CAM1_ID)
    cap2 = open_camera_device(robot_0.CAM2_ID)
    
    # 設定相機解析度
    for cap in [cap0, cap2]:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


    if not cap0.isOpened() or not cap2.isOpened():
        # 針對索引/權限錯誤做快速掃描，找出可用的兩顆鏡頭
        for c in [cap0, cap2]:
            try:
                c.release()
            except Exception:
                pass
        opened = _select_camera_pair(preferred_ids=(robot_0.CAM1_ID, robot_0.CAM2_ID), max_scan=8)
        if len(opened) < 2:
            print("錯誤: 無法開啟相機")
            return
        (robot_0.CAM1_ID, cap0), (robot_0.CAM2_ID, cap2) = opened[0], opened[1]
        print(f"[camera] fallback to CAM1={robot_0.CAM1_ID}, CAM2={robot_0.CAM2_ID}")


    # 權重初始值
    weight_cam0 = 0.5
    weight_cam2 = 0.5


    # ==========================================
    # 4. 建立視窗並設定回調
    # ==========================================
    cv2.namedWindow("Camera 0 - Perspective")
    cv2.setMouseCallback("Camera 0 - Perspective", mouse_callback_cam0)
    cv2.namedWindow("Camera 2 - Perspective")
    cv2.setMouseCallback("Camera 2 - Perspective", mouse_callback_cam2)
    cv2.namedWindow("Fused View + Ball Track")
    cv2.setMouseCallback("Fused View + Ball Track", mouse_callback_fused)
    cv2.namedWindow("Ball Mask", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Robot Track - Cam0")
    cv2.namedWindow("Robot Track - Cam2")


    print("\n[系統操作說明]")
    print("1. 在 'Camera 0/2 - Perspective' 視窗各點選 4 個場地角點")
    print("2. 完成後在 'Fused View + Ball Track' 點擊球體設定顏色")
    print("3. 系統將同時追蹤球體與機器人")
    print("4. 按 ESC 離開\n")

    map_cam0 = None
    map_cam2 = None
    map_cam0_shape = None
    map_cam2_shape = None
    warp_m0 = None
    warp_m2 = None
    warp_pts0 = None
    warp_pts2 = None


    try:
        while True:
            ret0, frame0 = cap0.read()
            ret2, frame2 = cap2.read()
            if not ret0 or not ret2: break


            # ==========================================
            # 5. 影像前處理 (去畸變)
            # ==========================================
            if map_cam0 is None or map_cam0_shape != frame0.shape[:2]:
                map_cam0 = build_undistort_maps(mtx_cam0, dist_cam0, frame0.shape)
                map_cam0_shape = frame0.shape[:2]
            if map_cam2 is None or map_cam2_shape != frame2.shape[:2]:
                map_cam2 = build_undistort_maps(mtx_cam2, dist_cam2, frame2.shape)
                map_cam2_shape = frame2.shape[:2]

            frame0_undist = undistort_image(frame0, mtx_cam0, dist_cam0, maps=map_cam0)
            frame2_undist = undistort_image(frame2, mtx_cam2, dist_cam2, maps=map_cam2)
            undist0 = cv2.rotate(frame0_undist, cv2.ROTATE_90_CLOCKWISE)
            undist2 = cv2.rotate(frame2_undist, cv2.ROTATE_90_COUNTERCLOCKWISE)


            # ==========================================
            # 6. 顯示選點畫面
            # ==========================================
            disp0, disp2 = undist0.copy(), undist2.copy()
            draw_points(disp0, points_cam0)
            draw_points(disp2, points_cam2)
            cv2.imshow("Camera 0 - Perspective", disp0)
            cv2.imshow("Camera 2 - Perspective", disp2)


            # ==========================================
            # 7. 機器人追蹤 (ArUco) - 使用獨立函數
            # ==========================================
            robot_res0 = process_robot_pipeline(
                frame0, detector, mtx_cam0, dist_cam0, target_ids, "Cam1", 
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE, pre_undist=frame0_undist
            )
            robot_res2 = process_robot_pipeline(
                frame2, detector, mtx_cam2, dist_cam2, target_ids, "Cam2", 
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE, pre_undist=frame2_undist
            )
            
            if robot_res0 is not None:
                cv2.imshow("Robot Track - Cam0", robot_res0)
            if robot_res2 is not None:
                cv2.imshow("Robot Track - Cam2", robot_res2)


            # ==========================================
            # 8. 球體追蹤 (需完成選點)
            # ==========================================
            if len(points_cam0) == 4 and len(points_cam2) == 4:
                if warp_m0 is None or not _points_equal(warp_pts0 or [], points_cam0):
                    warp_m0 = get_perspective_matrix(points_cam0, WINDOW_WIDTH, WINDOW_HEIGHT)
                    warp_pts0 = [list(pt) for pt in points_cam0]
                if warp_m2 is None or not _points_equal(warp_pts2 or [], points_cam2):
                    warp_m2 = get_perspective_matrix(points_cam2, WINDOW_WIDTH, WINDOW_HEIGHT)
                    warp_pts2 = [list(pt) for pt in points_cam2]
                warped0 = cv2.warpPerspective(undist0, warp_m0, (WINDOW_WIDTH, WINDOW_HEIGHT))
                warped2 = cv2.warpPerspective(undist2, warp_m2, (WINDOW_WIDTH, WINDOW_HEIGHT))


                # 融合影像（固定權重，減少重複計算以提升即時性）
                fused = cv2.addWeighted(warped0, 0.5, warped2, 0.5, 0)
                fused_for_mask = fused

                # 若有待處理的 HSV 取樣，移至此處執行以避免回調卡畫面
                if _pending_hsv_sample is not None:
                    pending_img, px, py = _pending_hsv_sample
                    globals()['_pending_hsv_sample'] = None
                    my_ball.set_hsv_range_from_selection(pending_img, px, py)
                    # 點擊後立即更新一次遮罩
                    result_img, mask, _ = my_ball.track(fused.copy())
                    cv2.imshow("Fused View + Ball Track", result_img)
                    mask_to_show = mask if mask is not None else np.zeros(result_img.shape[:2], dtype=np.uint8)
                    cv2.imshow("Ball Mask", mask_to_show)


                # 正式追蹤並繪圖
                result_img, mask, is_tracked = my_ball.track(fused)
                
                # 在融合視窗顯示球體追蹤結果
                cv2.imshow("Fused View + Ball Track", result_img)
                # 讓遮罩持續更新，即便為空也刷新
                mask_to_show = mask if mask is not None else np.zeros(result_img.shape[:2], dtype=np.uint8)
                cv2.imshow("Ball Mask", mask_to_show)
                
                if is_tracked:
                    now_ts = time.time()
                    if now_ts - _last_ball_print_ts >= _PRINT_INTERVAL:
                        globals()['_last_ball_print_ts'] = now_ts
                        print(f"\r[Ball] 座標: {my_ball.center_real}", end="")


            # ==========================================
            # 10. 等待按鍵
            # ==========================================
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key in (ord(']'), ord('='), ord('+')):
                my_ball.adjust_manual_radius(1)
            elif key in (ord('['), ord('-'), ord('_')):
                my_ball.adjust_manual_radius(-1)
            elif key == ord('0'):
                my_ball.reset_manual_radius()


    except KeyboardInterrupt:
        print("\n中斷信號收到")
    finally:
        cap0.release()
        cap2.release()
        cv2.destroyAllWindows()
        print("\n系統已關閉")
        
        # 顯示最終統計
        print("\n=== 追蹤統計 ===")
        for robot in robots:
            print(f"\nRobot {robot.id} (ArUco ID:{robot.aruco_id_list[0]}, Setting:{robot.camera_setting}):")
            print(f"  Cam1 偵測次數: {robot.amount_choose_left}")
            print(f"  Cam2 偵測次數: {robot.amount_choose_right}")
            if robot.amount_choose_left > 0:
                print(f"  Cam1 數據: X={robot.x_left:.1f}, Y={robot.y_left:.1f}, Deg={robot.degree_left:.1f}, Vec=({robot.vec_x_left:.2f},{robot.vec_y_left:.2f})")
            if robot.amount_choose_right > 0:
                print(f"  Cam2 數據: X={robot.x_right:.1f}, Y={robot.y_right:.1f}, Deg={robot.degree_right:.1f}, Vec=({robot.vec_x_right:.2f},{robot.vec_y_right:.2f})")
            if robot.amount_choose_left > 0 or robot.amount_choose_right > 0:
                print(f"  最終位置: ({robot.x:.1f}, {robot.y:.1f})")
                print(f"  最終角度: {robot.degree:.1f}°")
                print(f"  最終向量: ({robot.vec_x:.2f}, {robot.vec_y:.2f})")


if __name__ == "__main__":
    main()


# ============================================================
# Bridge for main_v5_nrfcontrol_mod2025 copy.py
#  - Expose team/ball state in the same shape as image_processing_Androsot_no_skin
#  - Provide start_image_thread() entrypoint compatible with main_v5...
# ============================================================
team_pos = []
team_degree = []
team_vec_x = []
team_vec_y = []
oppo_pos = []
ball_center = []

_state_lock = threading.Lock()
_worker = None
_running = False


def _update_shared_state(robots):
    """Update shared lists that main_v5 reads."""
    with _state_lock:
        team_pos.clear()
        team_degree.clear()
        team_vec_x.clear()
        team_vec_y.clear()
        team_pos.extend([[float(r.x), float(r.y)] for r in robots])
        # 角度採用 degree_base（若缺則用 degree），轉成單位向量供主程式使用
        team_degree.extend([
                [float(r.vec_x), float(r.vec_y)]
            for r in robots
        ])
        team_vec_x.extend([
            math.cos(math.radians(getattr(r, "degree_base", r.degree)))
            for r in robots
        ])
        team_vec_y.extend([
            math.sin(math.radians(getattr(r, "degree_base", r.degree)))
            for r in robots
        ])
        oppo_pos.clear()  # not produced by this pipeline

       


        if my_ball and my_ball.center_real:
            ball_center[:] = [float(my_ball.center_real[0]), float(my_ball.center_real[1])]
        else:
            ball_center.clear()


def _default_corners_from_frame(frame):
    """Return 4-corner warp points when user has not clicked any."""
    h, w = frame.shape[:2]
    return [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]


def _processing_loop(show_windows=False):
    global _running, fused_preview, robot_cam0_preview, robot_cam2_preview, fused_for_mask
    # Robot setup (same IDs as original complete1.py) 上 前 左 後 右
    robot_0 = Robot(0, [0, 6, 2, 8, 4],  [0.0]*5, [0.0]*5, [0.0]*5, "right")
    robot_1 = Robot(1, [10, 16, 12, 18, 14], [0.0]*5, [0.0]*5, [0.0]*5, "right")
    robot_2 = Robot(2, [20, 26, 22, 28, 24], [0.0]*5, [0.0]*5, [0.0]*5, "right")
    robots = [robot_0, robot_1, robot_2]

    target_ids = [rid for r in robots for rid in r.aruco_id_list]
    robots_dict = {rid: r for r in robots for rid in r.aruco_id_list}

    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')
    detector = robot_0.detector

    cap0 = open_camera_device(robot_0.CAM1_ID)
    cap2 = open_camera_device(robot_0.CAM2_ID)
    for cap in [cap0, cap2]:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap0.isOpened() or not cap2.isOpened():
        print("[imageprocess_complete1] Cannot open cameras 0/2")
        _running = False
        return

    # 每次重新啟動時重置校正點與視窗狀態
    # 若外部已預載入 4 個點（預設模式），則保留以便直接套用；不足 4 個才清空重新選點
    if len(points_cam0) != 4:
        points_cam0.clear()
    if len(points_cam2) != 4:
        points_cam2.clear()
    windows_active = show_windows
    windows_closed = False  # 確保校正完成後只清一次 cv2 視窗
    _load_default_hsv()  # 讀取上次的 HSV 遮罩
    if show_windows:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
    if show_windows:
        # 初始依模式建立需要的視窗，模式改變時在迴圈內處理
        if DISPLAY_MODE == "full":
            cv2.namedWindow("Camera 0 - Perspective")
            cv2.setMouseCallback("Camera 0 - Perspective", mouse_callback_cam0)
            cv2.namedWindow("Camera 2 - Perspective")
            cv2.setMouseCallback("Camera 2 - Perspective", mouse_callback_cam2)
            cv2.namedWindow("Ball Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Fused View + Ball Track")
        cv2.setMouseCallback("Fused View + Ball Track", mouse_callback_fused)
        cv2.namedWindow("Robot Track - Cam0")
        cv2.namedWindow("Robot Track - Cam2")
        print("\n[系統操作說明]")
        if DISPLAY_MODE == "full":
            print("1. 在 'Camera 0/2 - Perspective' 視窗各點選 4 個場地角點")
            print("2. 完成後在 'Fused View + Ball Track' 點擊球體設定顏色")
        else:
            print("1. 使用預設或已保存的校正點（compact）")
            print("2. 如需重設角點，切換回 full 模式")
        print("3. 系統將同時追蹤球體與機器人")
        print("4. 按 ESC 關閉視窗（辨識持續運行）\n")

    map_cam0 = None
    map_cam2 = None
    map_cam0_shape = None
    map_cam2_shape = None
    warp_m0 = None
    warp_m2 = None
    warp_pts0 = None
    warp_pts2 = None

    try:
        while _running:
            def _close_windows():
                nonlocal windows_active, windows_closed
                windows_active = False
                windows_closed = True
                try:
                    cv2.destroyAllWindows()
                except cv2.error:
                    pass

            if windows_active and not windows_closed:
                try:
                    # 若使用者關閉視窗，偵測後僅關閉顯示，不停止辨識
                    if cv2.getWindowProperty("Fused View + Ball Track", cv2.WND_PROP_VISIBLE) < 1:
                        _close_windows()
                except cv2.error:
                    _close_windows()

            show_perspective = (DISPLAY_MODE == "full")
            show_ball_mask = (DISPLAY_MODE == "full")
            try:
                ret0, frame0 = cap0.read()
                ret2, frame2 = cap2.read()
            except Exception as err:
                print(f"[imageprocess_complete1] Camera read exception: {err}")
                break
            if not ret0 or not ret2:
                print("[imageprocess_complete1] Camera read failed")
                time.sleep(0.1)
                continue

            try:
                if map_cam0 is None or map_cam0_shape != frame0.shape[:2]:
                    map_cam0 = build_undistort_maps(mtx_cam0, dist_cam0, frame0.shape)
                    map_cam0_shape = frame0.shape[:2]
                if map_cam2 is None or map_cam2_shape != frame2.shape[:2]:
                    map_cam2 = build_undistort_maps(mtx_cam2, dist_cam2, frame2.shape)
                    map_cam2_shape = frame2.shape[:2]

                frame0_undist = undistort_image(frame0, mtx_cam0, dist_cam0, maps=map_cam0)
                frame2_undist = undistort_image(frame2, mtx_cam2, dist_cam2, maps=map_cam2)
                undist0 = cv2.rotate(frame0_undist, cv2.ROTATE_90_CLOCKWISE)
                undist2 = cv2.rotate(frame2_undist, cv2.ROTATE_90_COUNTERCLOCKWISE)
            except Exception as err:
                print(f"[imageprocess_complete1] undistort error: {err}")
                time.sleep(0.01)
                continue

            # Robot tracking (no UI drawing unless show_windows)
            try:
                robot_res0 = process_robot_pipeline(
                    frame0, detector, mtx_cam0, dist_cam0, target_ids, "Cam1",
                    robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE, pre_undist=frame0_undist
                )
                robot_res2 = process_robot_pipeline(
                    frame2, detector, mtx_cam2, dist_cam2, target_ids, "Cam2",
                    robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE, pre_undist=frame2_undist
                )
            except Exception as err:
                print(f"[imageprocess_complete1] robot pipeline error: {err}")
                time.sleep(0.01)
                continue

            if windows_active:
                try:
                    if robot_res0 is not None:
                        cv2.imshow("Robot Track - Cam0", robot_res0)
                    if robot_res2 is not None:
                        cv2.imshow("Robot Track - Cam2", robot_res2)
                except cv2.error:
                    pass

            # Ball tracking (best-effort, manual calibration when show_windows)
            need_manual_points = False
            if windows_active and show_perspective:
                disp0 = undist0.copy()
                disp2 = undist2.copy()
                draw_points(disp0, points_cam0)
                draw_points(disp2, points_cam2)
                try:
                    cv2.imshow("Camera 0 - Perspective", disp0)
                    cv2.imshow("Camera 2 - Perspective", disp2)
                except cv2.error:
                    pass
                if len(points_cam0) < 4 or len(points_cam2) < 4:
                    need_manual_points = True
            else:
                if len(points_cam0) != 4:
                    points_cam0[:] = _default_corners_from_frame(undist0)
                if len(points_cam2) != 4:
                    points_cam2[:] = _default_corners_from_frame(undist2)

                # 僅在使用者未關閉視窗時維持顯示
                if show_windows and not windows_closed:
                    windows_active = True

            if need_manual_points and show_perspective:
                # 更新預覽為校正畫面，讓 Tk UI 同步看到
                robot_cam0_preview = disp0.copy()
                robot_cam2_preview = disp2.copy()
                fused_preview = None
                if windows_active and cv2.waitKey(1) & 0xFF == 27:
                    _close_windows()
                time.sleep(0.01)
                continue

            # 如果目前為 compact 模式且仍未取得 4 點，使用即時畫面自動抓角點，避免 warp 失敗
            if not show_perspective:
                if len(points_cam0) != 4:
                    points_cam0[:] = _default_corners_from_frame(undist0)
                if len(points_cam2) != 4:
                    points_cam2[:] = _default_corners_from_frame(undist2)

            # 若仍不足 4 點，跳過該迴圈避免 warp 崩潰
            if len(points_cam0) != 4 or len(points_cam2) != 4:
                if windows_active and cv2.waitKey(1) & 0xFF == 27:
                    _close_windows()
                time.sleep(0.01)
                continue

            # 如有新的 4 點校正，立即儲存為下次預設
            if len(points_cam0) == 4 and len(points_cam2) == 4:
                if not _points_equal(_last_saved_points.get("cam0") or [], points_cam0) or \
                   not _points_equal(_last_saved_points.get("cam2") or [], points_cam2):
                    _save_default_points(points_cam0, points_cam2)

            if warp_m0 is None or not _points_equal(warp_pts0 or [], points_cam0):
                warp_m0 = get_perspective_matrix(points_cam0, WINDOW_WIDTH, WINDOW_HEIGHT)
                warp_pts0 = [list(pt) for pt in points_cam0]
            if warp_m2 is None or not _points_equal(warp_pts2 or [], points_cam2):
                warp_m2 = get_perspective_matrix(points_cam2, WINDOW_WIDTH, WINDOW_HEIGHT)
                warp_pts2 = [list(pt) for pt in points_cam2]
            warped0 = cv2.warpPerspective(undist0, warp_m0, (WINDOW_WIDTH, WINDOW_HEIGHT))
            warped2 = cv2.warpPerspective(undist2, warp_m2, (WINDOW_WIDTH, WINDOW_HEIGHT))

            # Simple fuse and track（避免多次 track 提升即時性）
            try:
                fused = cv2.addWeighted(warped0, 0.5, warped2, 0.5, 0)
            except cv2.error as err:
                print(f"[fuse] addWeighted error: {err}")
                if windows_active:
                    cv2.waitKey(1)
                time.sleep(0.01)
                continue
            fused_for_mask = fused
            result_img, mask, is_tracked = my_ball.track(fused)

            # 更新預覽影像供 Tk UI 使用
            if result_img is not None:
                fused_preview = result_img.copy()
            if robot_res0 is not None:
                robot_cam0_preview = robot_res0.copy()
            if robot_res2 is not None:
                robot_cam2_preview = robot_res2.copy()

            if windows_active:
                try:
                    # 若有待處理的 HSV 取樣，移至此處執行以避免回調卡畫面
                    if _pending_hsv_sample is not None:
                        pending_img, px, py = _pending_hsv_sample
                        globals()['_pending_hsv_sample'] = None
                        my_ball.set_hsv_range_from_selection(pending_img, px, py)
                        result_img, mask, _ = my_ball.track(fused.copy())
                        _save_default_hsv(my_ball.hsv_range)
                    cv2.imshow("Fused View + Ball Track", result_img)
                    if show_ball_mask:
                        mask_to_show = mask if mask is not None else np.zeros(result_img.shape[:2], dtype=np.uint8)
                        cv2.imshow("Ball Mask", mask_to_show)
                except cv2.error:
                    pass

            _update_shared_state(robots)

            if windows_active:
                try:
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:
                        _close_windows()
                    elif key in (ord(']'), ord('='), ord('+')):
                        my_ball.adjust_manual_radius(1)
                    elif key in (ord('['), ord('-'), ord('_')):
                        my_ball.adjust_manual_radius(-1)
                    elif key == ord('0'):
                        my_ball.reset_manual_radius()
                except cv2.error:
                    pass

            # 小延遲，避免過載
            time.sleep(0.01)

    finally:
        cap0.release()
        cap2.release()
        if show_windows:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        _running = False


def start_image_thread(show_windows=False):
    """Start background vision thread (non-blocking)."""
    global _worker, _running
    if _worker and _worker.is_alive():
        _running = False
        _worker.join(timeout=1.0)
        _worker = None
    _running = True
    _worker = threading.Thread(target=_processing_loop, args=(show_windows,), daemon=True)
    _worker.start()


def stop_image_thread():
    """Stop vision thread if running."""
    global _worker, _running
    _running = False
    if _worker and _worker.is_alive():
        _worker.join(timeout=1.0)
    _worker = None


def select_ball_color_at_canvas(x, y, canvas_w, canvas_h):
    """
    Allow Tk canvas click to set ball HSV from the fused preview.
    x,y: click on canvas; canvas_w/h: canvas size in pixels.
    """
    global fused_for_mask
    if fused_for_mask is None:
        return False
    h, w = fused_for_mask.shape[:2]
    fx = int(x / max(canvas_w, 1) * w)
    fy = int(y / max(canvas_h, 1) * h)
    fx = max(0, min(w - 1, fx))
    fy = max(0, min(h - 1, fy))
    try:
        my_ball.set_hsv_range_from_selection(fused_for_mask, fx, fy)
        _save_default_hsv(my_ball.hsv_range)
        return True
    except Exception as err:
        print(f"[select_ball_color] error: {err}")
        return False


def select_ball_reference_at_canvas(x, y, canvas_w, canvas_h):
    """
    Right-click pick: record current fused frame as reference mask region (color/shape).
    This saves both the HSV range around the point and a cropped template of the mask.
    """
    global fused_for_mask
    if fused_for_mask is None:
        return False
    h, w = fused_for_mask.shape[:2]
    fx = int(x / max(canvas_w, 1) * w)
    fy = int(y / max(canvas_h, 1) * h)
    fx = max(0, min(w - 1, fx))
    fy = max(0, min(h - 1, fy))
    try:
        # 更新 HSV 範圍
        my_ball.set_hsv_range_from_selection(fused_for_mask, fx, fy)
        _save_default_hsv(my_ball.hsv_range)
        # 取一個小區塊當作形狀/顏色模板（可擴充用於模板比對）
        sz = 20
        x0 = max(0, fx - sz)
        y0 = max(0, fy - sz)
        x1 = min(w, fx + sz)
        y1 = min(h, fy + sz)
        template = fused_for_mask[y0:y1, x0:x1].copy()
        # 暫存於 ball 物件以備後續使用（可用於形狀/顏色參考）
        my_ball.reference_template = template
        my_ball.reference_point = (fx, fy)
        return True
    except Exception as err:
        print(f"[select_ball_reference] error: {err}")
        return False


def select_ball_reference_rect(x0, y0, x1, y1, canvas_w, canvas_h):
    """
    Drag-select on fused canvas: use the rectangle as reference (HSV + template).
    """
    global fused_for_mask
    if fused_for_mask is None:
        return False


def select_ball_reference_polygon(points, canvas_w, canvas_h):
    """
    Freehand polygon selection on fused canvas: use the polygon as reference (HSV + template).
    points: list of (x,y) in canvas coords.
    """
    global fused_for_mask
    if fused_for_mask is None or not points or len(points) < 3:
        return False
    h, w = fused_for_mask.shape[:2]
    pts = []
    for (cx, cy) in points:
        fx = int(cx / max(canvas_w, 1) * w)
        fy = int(cy / max(canvas_h, 1) * h)
        fx = max(0, min(w - 1, fx))
        fy = max(0, min(h - 1, fy))
        pts.append([fx, fy])
    pts_np = np.array(pts, dtype=np.int32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts_np], 255)
    if cv2.countNonZero(mask) == 0:
        return False
    region = cv2.bitwise_and(fused_for_mask, fused_for_mask, mask=mask)
    hsv_region = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    hsv_pixels = hsv_region[mask == 255]
    if hsv_pixels.size == 0:
        return False
    avg_hsv = hsv_pixels.mean(axis=0).astype(int)
    lower = np.array([max(avg_hsv[0]-40,0), max(avg_hsv[1]-100,0), max(avg_hsv[2]-100,0)])
    upper = np.array([min(avg_hsv[0]+40,179), min(avg_hsv[1]+100,255), min(avg_hsv[2]+100,255)])
    my_ball.hsv_range = (lower, upper)
    x, y, ww, hh = cv2.boundingRect(pts_np)
    x1 = min(w, x + ww)
    y1 = min(h, y + hh)
    template = region[y:y1, x:x1].copy()
    my_ball.reference_template = template
    my_ball.reference_point = (int(pts_np[:,0].mean()), int(pts_np[:,1].mean()))
    _save_default_hsv(my_ball.hsv_range)
    return True
    h, w = fused_for_mask.shape[:2]
    fx0 = int(x0 / max(canvas_w, 1) * w)
    fy0 = int(y0 / max(canvas_h, 1) * h)
    fx1 = int(x1 / max(canvas_w, 1) * w)
    fy1 = int(y1 / max(canvas_h, 1) * h)
    fx0, fx1 = sorted((max(0, min(w - 1, fx0)), max(0, min(w - 1, fx1))))
    fy0, fy1 = sorted((max(0, min(h - 1, fy0)), max(0, min(h - 1, fy1))))
    if fx1 - fx0 < 2 or fy1 - fy0 < 2:
        return False
    try:
        region = fused_for_mask[fy0:fy1, fx0:fx1]
        hsv_region = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        avg_hsv = hsv_region.reshape(-1, 3).mean(axis=0).astype(int)
        lower = np.array([max(avg_hsv[0]-40,0), max(avg_hsv[1]-100,0), max(avg_hsv[2]-100,0)])
        upper = np.array([min(avg_hsv[0]+40,179), min(avg_hsv[1]+100,255), min(avg_hsv[2]+100,255)])
        my_ball.hsv_range = (lower, upper)
        my_ball.reference_template = region.copy()
        my_ball.reference_point = (int((fx0+fx1)/2), int((fy0+fy1)/2))
        _save_default_hsv(my_ball.hsv_range)
        return True
    except Exception as err:
        print(f"[select_ball_reference_rect] error: {err}")
        return False


# main_v5_nrfcontrol_mod2025 copy.py expects this name
def image_result():
    start_image_thread(show_windows=False)


# UI stubs to match old interface (HSV/Color mask not implemented here)
def HSVdetection(color, camera_selection_setting):
    print("[imageprocess_complete1] HSVdetection stub – not implemented in this pipeline")


def ColorMask(color, color_selection_setting, camera_selection_setting):
    print("[imageprocess_complete1] ColorMask stub – not implemented in this pipeline")


def imagedetection(fieldchoose=0):
    """Start vision pipeline with UI windows (matches old button behavior)."""
    start_image_thread(show_windows=True)
