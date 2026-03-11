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

try:
    from . import geometry_mapping as gm
except Exception:
    import geometry_mapping as gm
try:
    from . import angle_fusion as af
except Exception:
    import angle_fusion as af
try:
    from . import camera_runtime as cr
except Exception:
    import camera_runtime as cr
try:
    from . import aruco_runtime as ar
except Exception:
    import aruco_runtime as ar

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
    return af._norm_angle_deg(deg)


def _circular_mean_deg(angles):
    return af._circular_mean_deg(angles)


def _weighted_mean_angle_deg(left_deg: float, right_deg: float, w_left: float, w_right: float):
    return af._weighted_mean_angle_deg(left_deg, right_deg, w_left, w_right)


def _prune_samples(samples: deque, now_ts: float, window_sec: float):
    af._prune_samples(samples, now_ts, window_sec)


def _mode_angle_from_samples(samples, bin_size_deg: float):
    return af._mode_angle_from_samples(samples, bin_size_deg)


def _push_angle_sample(robot, cam_role: str, angle_deg: float, ts: float):
    af._push_angle_sample(robot, cam_role, angle_deg, ts)


def _get_stable_angle(robot, ts: float):
    return af._get_stable_angle(robot, ts, ANGLE_MODE_WINDOW_SEC, ANGLE_MODE_BIN_DEG)


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


# Runtime camera capture settings
RUNTIME_CAPTURE_PARAMS_FILE = cr.RUNTIME_CAPTURE_PARAMS_FILE
CAMERA_CAPTURE_DEFAULTS = cr.CAMERA_CAPTURE_DEFAULTS.copy()
ARUCO_UPSCALE_DEFAULTS = cr.ARUCO_UPSCALE_DEFAULTS.copy()


def _normalize_camera_capture_config(raw):
    return cr._normalize_camera_capture_config(raw)


def _set_camera_prop(cap, prop_id, value):
    return cr._set_camera_prop(cap, prop_id, value)


def _apply_camera_capture_config_to_cap(cap, cfg=None):
    return cr._apply_camera_capture_config_to_cap(cap, cfg)


def _apply_camera_capture_config_to_caps(caps):
    cr._apply_camera_capture_config_to_caps(caps)


def _set_active_caps(caps):
    cr._set_active_caps(caps)


def _clear_active_caps():
    cr._clear_active_caps()


def get_camera_capture_config():
    return cr.get_camera_capture_config()


def set_camera_capture_config(config, apply_now=True, replace=False, persist=False):
    return cr.set_camera_capture_config(config, apply_now=apply_now, replace=replace, persist=persist)

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

# Runtime ArUco detector params (支持 UI 即時覆蓋)
ARUCO_PARAMS_FILE = ar.ARUCO_PARAMS_FILE
ARUCO_DEFAULT_PARAMS = ar.ARUCO_DEFAULT_PARAMS.copy()


def _normalize_upscale_factors(raw):
    return cr._normalize_upscale_factors(raw)


def get_aruco_upscale_options():
    return cr.get_aruco_upscale_options()


def set_aruco_upscale_options(options, replace=False, persist=False):
    return cr.set_aruco_upscale_options(options, replace=replace, persist=persist)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _to_int(value, default):
    try:
        return int(round(float(value)))
    except Exception:
        return int(default)


def _to_float(value, default):
    try:
        return float(value)
    except Exception:
        return float(default)


def _runtime_capture_default_payload():
    return cr._runtime_capture_default_payload()


def _read_runtime_capture_payload(path=None):
    return cr._read_runtime_capture_payload(path)


def save_runtime_capture_params(path=None):
    return cr.save_runtime_capture_params(path)


def load_runtime_capture_params(path=None, apply_now=True):
    return cr.load_runtime_capture_params(path, apply_now=apply_now)


try:
    cr.init_runtime_capture_params()
except Exception as err:
    print(f"[runtime_capture] init failed: {err}")


def _normalize_aruco_params(raw):
    return ar._normalize_aruco_params(raw)


def _build_aruco_detector_from_params(params):
    return ar._build_aruco_detector_from_params(params)


def _set_aruco_detector_params_internal(params, replace=False):
    return ar._set_aruco_detector_params_internal(params, replace=replace)


def get_aruco_detector():
    return ar.get_aruco_detector()


def get_aruco_detector_state():
    return ar.get_aruco_detector_state()


def get_aruco_detector_params():
    return ar.get_aruco_detector_params()


def set_aruco_detector_params(params, persist=False, replace=False):
    return ar.set_aruco_detector_params(params, persist=persist, replace=replace)


def save_aruco_detector_params(path=None):
    return ar.save_aruco_detector_params(path)


def load_aruco_detector_params(path=None):
    return ar.load_aruco_detector_params(path)


def reset_aruco_detector_params(persist=False):
    return ar.reset_aruco_detector_params(persist=persist)


try:
    ar.init_aruco_detector_params()
except Exception as err:
    print(f"[aruco] init failed: {err}")


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
        self.centers_px = []   # 多球像素座標列表
        self.centers_real = [] # 多球實際座標列表
        self.centers_radius_px = []  # 多球半徑列表
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
        self.last_centers_px = []
        self.last_centers_real = []
        self.last_centers_radius_px = []
        self.last_radius_px = 0
        # 球偵測模式固定為單球
        self.detect_mode = "single"
        # 品質統計快取
        self._debug_samples = deque(maxlen=600)
        self._debug_lock = threading.Lock()

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
        max_keep = 1
        self.center_px = last_px
        self.center_real = last_real
        if self.last_centers_px and self.last_centers_real:
            self.centers_px = [tuple(map(int, pt)) for pt in self.last_centers_px[:max_keep]]
            self.centers_real = [tuple(map(float, pt)) for pt in self.last_centers_real[:max_keep]]
            self.centers_radius_px = [int(r) for r in (self.last_centers_radius_px or [])[:max_keep]]
        else:
            self.centers_px = [self.center_px]
            self.centers_real = [self.center_real]
            self.centers_radius_px = [int(last_radius)] if last_radius else []
        if self.radius_px == 0 and last_radius:
            self.radius_px = last_radius
        return True

    def _push_debug_sample(self, ts, mask_ratio, hough_count, latency_ms, fallback_used):
        sample = {
            "ts": float(ts),
            "tracked": bool(self.is_tracked),
            "fallback_used": bool(fallback_used),
            "count": int(len(self.centers_real)),
            "center_px": None if self.center_px is None else [float(self.center_px[0]), float(self.center_px[1])],
            "center_real": None if self.center_real is None else [float(self.center_real[0]), float(self.center_real[1])],
            "centers_px": [[float(x), float(y)] for x, y in (self.centers_px or [])[:1]],
            "centers_real": [[float(x), float(y)] for x, y in (self.centers_real or [])[:1]],
            "radius_px": float(self.radius_px) if self.radius_px else 0.0,
            "mask_ratio": None if mask_ratio is None else float(mask_ratio),
            "hough_count": int(hough_count),
            "latency_ms": float(latency_ms),
            "mode": "single",
        }
        with self._debug_lock:
            self._debug_samples.append(sample)

    def _compute_window_stats(self, samples, window_sec):
        frames = len(samples)
        if frames == 0:
            return {
                "window_sec": float(window_sec),
                "frames": 0,
                "tracked_frames": 0,
                "tracked_ratio": 0.0,
                "fallback_frames": 0,
                "fallback_ratio": 0.0,
                "fps": 0.0,
                "latency_avg_ms": None,
                "latency_p95_ms": None,
                "mask_ratio_avg": None,
                "hough_count_avg": None,
                "detected_count_avg": None,
                "radius_avg_px": None,
                "radius_std_px": None,
                "mean_center_cm": None,
                "jitter_cm_rms": None,
                "jitter_cm_peak": None,
                "jitter_px_rms": None,
                "jitter_px_peak": None,
                "step_cm_avg": None,
                "step_cm_p95": None,
                "two_ball_ratio": 0.0,
                "inter_ball_dist_avg_cm": None,
                "inter_ball_dist_std_cm": None,
            }

        tracked = [s for s in samples if s.get("tracked") and s.get("center_real") is not None]
        tracked_frames = len(tracked)
        fallback_frames = sum(1 for s in samples if s.get("fallback_used"))
        tracked_ratio = tracked_frames / max(frames, 1)
        fallback_ratio = fallback_frames / max(frames, 1)

        t0 = float(samples[0]["ts"])
        t1 = float(samples[-1]["ts"])
        duration = max(t1 - t0, 1e-6)
        fps = float((frames - 1) / duration) if frames > 1 else 0.0

        lat_vals = [float(s["latency_ms"]) for s in samples if s.get("latency_ms") is not None]
        mask_vals = [float(s["mask_ratio"]) for s in samples if s.get("mask_ratio") is not None]
        hough_vals = [float(s["hough_count"]) for s in samples]
        count_vals = [float(s["count"]) for s in samples]
        rad_vals = [float(s["radius_px"]) for s in tracked if s.get("radius_px") is not None and s.get("radius_px", 0.0) > 0.0]

        centers_cm = np.array([s["center_real"] for s in tracked], dtype=np.float64) if tracked_frames > 0 else None
        centers_px = np.array([s["center_px"] for s in tracked if s.get("center_px") is not None], dtype=np.float64) if tracked_frames > 0 else None

        mean_center_cm = None
        jitter_cm_rms = None
        jitter_cm_peak = None
        if centers_cm is not None and len(centers_cm) > 0:
            mean_cm = centers_cm.mean(axis=0)
            mean_center_cm = [float(mean_cm[0]), float(mean_cm[1])]
            dist_cm = np.linalg.norm(centers_cm - mean_cm, axis=1)
            jitter_cm_rms = float(math.sqrt(np.mean(np.square(dist_cm)))) if len(dist_cm) > 0 else None
            jitter_cm_peak = float(np.max(dist_cm)) if len(dist_cm) > 0 else None

        jitter_px_rms = None
        jitter_px_peak = None
        if centers_px is not None and len(centers_px) > 0:
            mean_px = centers_px.mean(axis=0)
            dist_px = np.linalg.norm(centers_px - mean_px, axis=1)
            jitter_px_rms = float(math.sqrt(np.mean(np.square(dist_px)))) if len(dist_px) > 0 else None
            jitter_px_peak = float(np.max(dist_px)) if len(dist_px) > 0 else None

        step_cm_avg = None
        step_cm_p95 = None
        if centers_cm is not None and len(centers_cm) >= 2:
            steps_cm = np.linalg.norm(np.diff(centers_cm, axis=0), axis=1)
            if len(steps_cm) > 0:
                step_cm_avg = float(np.mean(steps_cm))
                step_cm_p95 = float(np.percentile(steps_cm, 95))

        inter_ball_dists = []
        two_ball_frames = 0
        for s in tracked:
            pts = s.get("centers_real") or []
            if len(pts) >= 2:
                two_ball_frames += 1
                inter_ball_dists.append(float(math.hypot(float(pts[0][0]) - float(pts[1][0]), float(pts[0][1]) - float(pts[1][1]))))

        return {
            "window_sec": float(window_sec),
            "frames": int(frames),
            "tracked_frames": int(tracked_frames),
            "tracked_ratio": float(tracked_ratio),
            "fallback_frames": int(fallback_frames),
            "fallback_ratio": float(fallback_ratio),
            "fps": float(fps),
            "latency_avg_ms": float(np.mean(lat_vals)) if lat_vals else None,
            "latency_p95_ms": float(np.percentile(lat_vals, 95)) if lat_vals else None,
            "mask_ratio_avg": float(np.mean(mask_vals)) if mask_vals else None,
            "hough_count_avg": float(np.mean(hough_vals)) if hough_vals else None,
            "detected_count_avg": float(np.mean(count_vals)) if count_vals else None,
            "radius_avg_px": float(np.mean(rad_vals)) if rad_vals else None,
            "radius_std_px": float(np.std(rad_vals)) if rad_vals else None,
            "mean_center_cm": mean_center_cm,
            "jitter_cm_rms": jitter_cm_rms,
            "jitter_cm_peak": jitter_cm_peak,
            "jitter_px_rms": jitter_px_rms,
            "jitter_px_peak": jitter_px_peak,
            "step_cm_avg": step_cm_avg,
            "step_cm_p95": step_cm_p95,
            "two_ball_ratio": float(two_ball_frames / max(tracked_frames, 1)),
            "inter_ball_dist_avg_cm": float(np.mean(inter_ball_dists)) if inter_ball_dists else None,
            "inter_ball_dist_std_cm": float(np.std(inter_ball_dists)) if inter_ball_dists else None,
        }

    def get_debug_stats(self, short_window_sec=1.5, long_window_sec=6.0):
        with self._debug_lock:
            samples = list(self._debug_samples)
        now = time.time()
        short_samples = [s for s in samples if now - float(s["ts"]) <= float(short_window_sec)]
        long_samples = [s for s in samples if now - float(s["ts"]) <= float(long_window_sec)]
        latest = samples[-1] if samples else None
        return {
            "mode": "single",
            "now_ts": now,
            "latest": latest,
            "short": self._compute_window_stats(short_samples, short_window_sec),
            "long": self._compute_window_stats(long_samples, long_window_sec),
        }

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
        t_track_start = time.perf_counter()
        hough_count = 0
        mask_ratio = None
        if self.hsv_range is None:
            self.is_tracked = False
            self.center_px = None
            self.center_real = None
            self.centers_px = []
            self.centers_real = []
            self.centers_radius_px = []
            empty_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            self._push_debug_sample(
                ts=time.time(),
                mask_ratio=None,
                hough_count=0,
                latency_ms=(time.perf_counter() - t_track_start) * 1000.0,
                fallback_used=False,
            )
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
        self.centers_px = []
        self.centers_real = []
        self.centers_radius_px = []

        # 產生 Mask
        mask = cv2.inRange(hsv_img, self.hsv_range[0], self.hsv_range[1])
        # 針對小目標：縮小 kernel 與次數，避免侵蝕掉 10px 的球
        kernel = self._morph_kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        blurred = cv2.GaussianBlur(mask, (5,5), 1)
        nz = np.count_nonzero(mask)
        mask_ratio = float(nz / max(mask.size, 1))
        self._log_throttled(f"[Ball][Mask] nonzero={nz}/{mask.size} ({nz/mask.size:.4f})")

        # 若遮罩內無有效像素，略過 Hough 以節省運算
        if np.count_nonzero(blurred) == 0:
            self._log_throttled("[Ball][Mask] zero pixels after blur, skip Hough")
            self._record_candidate_position()
            self._advance_candidate_if_needed()
            fallback_used = self._use_last_known_position(last_px, last_real, last_radius)
            self._push_debug_sample(
                ts=time.time(),
                mask_ratio=mask_ratio,
                hough_count=hough_count,
                latency_ms=(time.perf_counter() - t_track_start) * 1000.0,
                fallback_used=fallback_used,
            )
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
            self._push_debug_sample(
                ts=time.time(),
                mask_ratio=mask_ratio,
                hough_count=hough_count,
                latency_ms=(time.perf_counter() - t_track_start) * 1000.0,
                fallback_used=fallback_used,
            )
            return result_img, mask, fallback_used

        if circles is not None:
            circles = np.uint16(np.around(circles))
            hough_count = int(len(circles[0])) if len(circles) > 0 else 0
            valid = []
            for (x, y, r) in circles[0, :]:
                ok_size = self._is_size_valid(r)
                self._log_throttled(f"[Ball][Hough] cand x={x} y={y} r={r} size_ok={ok_size}")
                if ok_size:
                    valid.append((int(x), int(y), int(r)))

            if valid:
                max_keep = 1
                # 先取半徑較大者，再去掉過近的重複圓，單球模式只保留一顆
                valid.sort(key=lambda c: c[2], reverse=True)
                selected = []
                for cx, cy, cr in valid:
                    too_close = False
                    for sx, sy, sr in selected:
                        dist = math.hypot(float(cx - sx), float(cy - sy))
                        if dist < max(4.0, 0.6 * min(cr, sr)):
                            too_close = True
                            break
                    if not too_close:
                        selected.append((cx, cy, cr))
                    if len(selected) >= max_keep:
                        break

                self.centers_px = []
                self.centers_real = []
                self.centers_radius_px = []
                for cx, cy, cr in selected:
                    px = self._lowpass_center((int(cx), int(cy)))
                    real_x = px[0] * (REAL_WIDTH / WINDOW_WIDTH)
                    real_y = px[1] * (REAL_HEIGHT / WINDOW_HEIGHT)
                    self.centers_px.append((int(px[0]), int(px[1])))
                    self.centers_real.append((float(real_x), float(real_y)))
                    self.centers_radius_px.append(int(cr))

                if self.centers_px:
                    self.is_tracked = True
                    if len(self.centers_px) == 1:
                        self.center_px = self.centers_px[0]
                        self.center_real = self.centers_real[0]
                        self.radius_px = int(self.centers_radius_px[0])
                    else:
                        mx = int(round(sum(p[0] for p in self.centers_px) / len(self.centers_px)))
                        my = int(round(sum(p[1] for p in self.centers_px) / len(self.centers_px)))
                        self.center_px = (mx, my)
                        self.center_real = (
                            float(sum(p[0] for p in self.centers_real) / len(self.centers_real)),
                            float(sum(p[1] for p in self.centers_real) / len(self.centers_real)),
                        )
                        self.radius_px = int(round(sum(self.centers_radius_px) / len(self.centers_radius_px)))

                    self.last_center_px = self.center_px
                    self.last_center_real = self.center_real
                    self.last_radius_px = self.radius_px
                    self.last_centers_px = list(self.centers_px)
                    self.last_centers_real = list(self.centers_real)
                    self.last_centers_radius_px = list(self.centers_radius_px)
                    self._draw_info(result_img)

        # 記錄候選穩定度，並視需要切換到下一組候選
        self._record_candidate_position()
        self._advance_candidate_if_needed()

        if not self.is_tracked:
            fallback_used = self._use_last_known_position(last_px, last_real, last_radius)

        self._push_debug_sample(
            ts=time.time(),
            mask_ratio=mask_ratio,
            hough_count=hough_count,
            latency_ms=(time.perf_counter() - t_track_start) * 1000.0,
            fallback_used=fallback_used,
        )
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
        
        # 單球繪製
        for idx, ((cx, cy), (rx, ry)) in enumerate(zip(self.centers_px, self.centers_real)):
            base_r = self.centers_radius_px[idx] if idx < len(self.centers_radius_px) else self.radius_px
            draw_r = self.manual_radius_px if self.manual_radius_px is not None else base_r
            color = (0, 255, 0)
            cv2.circle(image, (int(cx), int(cy)), int(draw_r), color, 2)
            cv2.circle(image, (int(cx), int(cy)), 2, (0, 0, 255), 3)
            cv2.putText(
                image,
                f"Ball:({rx:.1f},{ry:.1f})",
                (int(cx) + 12, int(cy) - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        # 主球座標（單球）
        if self.center_real is not None and self.center_px is not None:
            cx, cy = self.center_px
            rx, ry = self.center_real
            info_text = f"Ball(single): ({rx:.1f}, {ry:.1f}) cm"
            cv2.putText(image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.circle(image, (int(cx), int(cy)), 2, (255, 255, 255), 3)


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
        # 相機座標最後有效更新時間（秒）
        self.last_pos_left_ts = -1e9
        self.last_pos_right_ts = -1e9
        self.last_pos_left_frame = -1
        self.last_pos_right_frame = -1
        self._left_motion_step_cm = 1e9
        self._right_motion_step_cm = 1e9
        self._last_fusion_debug_ts = 0.0
        self._lr_bias_dx = None
        self._lr_bias_dy = None
        self._lr_bias_valid_ts = -1e9
        self._last_cam_choice = "N/A"
        
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
                
                _apply_camera_capture_config_to_cap(self.stream)
                
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
            return get_aruco_detector()
        
        self.load_calibration = load_calibration
        self.initialize_detector = initialize_detector
        
        self.mtx1, self.dist1 = load_calibration(self.CONFIG_CAM1['calib_file'])
        self.mtx2, self.dist2 = load_calibration(self.CONFIG_CAM2['calib_file'])
        self.detector = initialize_detector()


# ==========================================
# [獨立工具函數] - 機器人追蹤處理管線
# ==========================================
def process_robot_pipeline(
    frame,
    detector,
    mtx,
    dist,
    target_ids,
    cam_role,
    robots_dict,
    config_cam1,
    config_cam2,
    marker_size,
    pre_undist=None,
    filter_non_robot_aruco=True,
    cam_extrinsics=None,
    K_rot=None,
    frame_seq=None,
):
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
        # 最後手段：多尺度放大影像，提高小標記佔比
        try:
            scale_factors = get_aruco_upscale_options().get("upscale_factors", [1.6])
        except Exception:
            scale_factors = [1.6]
        for scale_factor in scale_factors:
            gray_up = cv2.resize(gray, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
            corners_up, ids_up, _ = detect(gray_up)
            if ids_up is not None:
                corners = [c / scale_factor for c in corners_up]
                ids = ids_up
                break
    
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
            if filter_non_robot_aruco and mid not in target_ids:
                continue
            
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
                xy_source = "MapTop"
                
                # 射線投影座標計算（如果相機外參可用）
                ray_fX, ray_fY = None, None
                if cam_extrinsics is not None and K_rot is not None:
                    aruco_center = corners[i][0].mean(axis=0)
                    rpx, rpy = _rotate_pixel_to_rotated_frame(
                        aruco_center[0], aruco_center[1], cam_role,
                        frame_h=frame.shape[0], frame_w=frame.shape[1],
                    )
                    R_ext, cam_pos = cam_extrinsics
                    h_robot_cfg = config_cam1['robot_height'] if cam_role == "Cam1" else config_cam2['robot_height']
                    ray_fX, ray_fY = _ray_cast_to_height(
                        rpx, rpy, K_rot, R_ext, cam_pos, h_robot_cfg * 100.0,
                    )
                    if _should_accept_ray_xy(ray_fX, ray_fY, mapped_fX, mapped_fY):
                        fX, fY = ray_fX, ray_fY
                        
                        xy_source = "Ray"
                
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
                    px = f"{fX:.1f}" if _is_finite_xy(fX, fY) else "N/A"
                    py = f"{fY:.1f}" if _is_finite_xy(fX, fY) else "N/A"
                    print(f"[{cam_role}] ID:{mid} src:{xy_source} Ang:{angle:.1f} Vec:({vec_x:.2f},{vec_y:.2f}) X:{px} Y:{py}")

                # 更新對應的 Robot 物件 (不管 fX 是否為 None 都更新向量和角度)
                if mid in robots_dict:
                    robot = robots_dict[mid]
                    if cam_role == "Cam1":
                        if fX is not None and fY is not None:
                            prev_left_ts = float(getattr(robot, "last_pos_left_ts", -1e9))
                            if prev_left_ts > 0.0:
                                prev_left_x = float(getattr(robot, "x_left", fX))
                                prev_left_y = float(getattr(robot, "y_left", fY))
                                robot._left_motion_step_cm = float(math.hypot(float(fX) - prev_left_x, float(fY) - prev_left_y))
                            else:
                                robot._left_motion_step_cm = 0.0
                            robot.x_left = fX
                            robot.y_left = fY
                            robot.last_pos_left_ts = now_ts
                            robot.last_pos_left_frame = int(frame_seq) if frame_seq is not None else -1
                        robot.degree_left = angle
                        robot.degree_base_left = raw_angle
                        robot.vec_x_left = vec_x
                        robot.vec_y_left = vec_y
                        robot.amount_choose_left += 1
                        _push_angle_sample(robot, cam_role, angle, now_ts)
                    else:  # Cam2
                        if fX is not None and fY is not None:
                            prev_right_ts = float(getattr(robot, "last_pos_right_ts", -1e9))
                            if prev_right_ts > 0.0:
                                prev_right_x = float(getattr(robot, "x_right", fX))
                                prev_right_y = float(getattr(robot, "y_right", fY))
                                robot._right_motion_step_cm = float(math.hypot(float(fX) - prev_right_x, float(fY) - prev_right_y))
                            else:
                                robot._right_motion_step_cm = 0.0
                            robot.x_right = fX
                            robot.y_right = fY
                            robot.last_pos_right_ts = now_ts
                            robot.last_pos_right_frame = int(frame_seq) if frame_seq is not None else -1
                        robot.degree_right = angle
                        robot.degree_base_right = raw_angle
                        robot.vec_x_right = vec_x
                        robot.vec_y_right = vec_y
                        robot.amount_choose_right += 1
                        _push_angle_sample(robot, cam_role, angle, now_ts)

                    # 雙相機融合策略：
                    # 1) 以本幀 fresh 偵測為主，避免吃到過期值造成偏差
                    # 2) 一側短暫斷線(<=2s)且另一側位置變化很小時，允許舊值參與融合，降低跳動
                    field_center_x = REAL_WIDTH / 2.0  # 180.0
                    HYSTERESIS = 5.0
                    BLEND_BAND = 20.0
                    # 避免雙相機都 fresh 時 Blend 權重飽和成 0/1（看起來像沒 fuse）
                    BLEND_WEIGHT_FLOOR = 0.15
                    # fresh+fresh 但左右差太大時，視為可疑，不做混合避免被壞值拉走
                    FRESH_BLEND_MAX_DIST_CM = 85.0
                    STALE_FUSE_MAX_SEC = 2.0
                    STABLE_STEP_CM = 12.0
                    STALE_MATCH_MAX_CM = 35.0

                    if frame_seq is not None:
                        # 僅允許本幀有讀到 ArUco 的相機作為 fresh
                        left_fresh = int(getattr(robot, "last_pos_left_frame", -1)) == int(frame_seq)
                        right_fresh = int(getattr(robot, "last_pos_right_frame", -1)) == int(frame_seq)
                    else:
                        # 相容舊介面（未傳 frame_seq 時沿用時間窗）
                        FRESH_SEC = 0.35
                        left_fresh = (now_ts - float(getattr(robot, "last_pos_left_ts", -1e9))) <= FRESH_SEC
                        right_fresh = (now_ts - float(getattr(robot, "last_pos_right_ts", -1e9))) <= FRESH_SEC

                    left_ts = float(getattr(robot, "last_pos_left_ts", -1e9))
                    right_ts = float(getattr(robot, "last_pos_right_ts", -1e9))
                    left_age = now_ts - left_ts
                    right_age = now_ts - right_ts
                    left_recent = left_ts > 0.0 and left_age <= STALE_FUSE_MAX_SEC
                    right_recent = right_ts > 0.0 and right_age <= STALE_FUSE_MAX_SEC

                    left_step = float(getattr(robot, "_left_motion_step_cm", 1e9))
                    right_step = float(getattr(robot, "_right_motion_step_cm", 1e9))
                    left_stable = left_step <= STABLE_STEP_CM
                    right_stable = right_step <= STABLE_STEP_CM

                    # 學習左右相機相對偏差（L-R），供單眼退化時補償
                    if left_fresh and right_fresh:
                        cur_dx = float(robot.x_left) - float(robot.x_right)
                        cur_dy = float(robot.y_left) - float(robot.y_right)
                        old_dx = getattr(robot, "_lr_bias_dx", None)
                        old_dy = getattr(robot, "_lr_bias_dy", None)
                        if old_dx is None or old_dy is None or (not math.isfinite(float(old_dx))) or (not math.isfinite(float(old_dy))):
                            robot._lr_bias_dx = cur_dx
                            robot._lr_bias_dy = cur_dy
                        else:
                            alpha = 0.08
                            robot._lr_bias_dx = (1.0 - alpha) * float(old_dx) + alpha * cur_dx
                            robot._lr_bias_dy = (1.0 - alpha) * float(old_dy) + alpha * cur_dy
                        robot._lr_bias_valid_ts = now_ts

                    prev_fx = float(getattr(robot, "x", 0.0))
                    prev_fy = float(getattr(robot, "y", 0.0))
                    x_left_use = float(robot.x_left)
                    y_left_use = float(robot.y_left)
                    x_right_use = float(robot.x_right)
                    y_right_use = float(robot.y_right)
                    left_syn = False
                    right_syn = False

                    bias_dx = getattr(robot, "_lr_bias_dx", None)
                    bias_dy = getattr(robot, "_lr_bias_dy", None)
                    bias_age = now_ts - float(getattr(robot, "_lr_bias_valid_ts", -1e9))
                    bias_valid = (
                        bias_dx is not None and bias_dy is not None and
                        math.isfinite(float(bias_dx)) and math.isfinite(float(bias_dy)) and
                        bias_age <= 120.0
                    )
                    if (not right_fresh) and left_fresh and left_stable and bias_valid:
                        x_right_use = x_left_use - float(bias_dx)
                        y_right_use = y_left_use - float(bias_dy)
                        right_syn = True
                    if (not left_fresh) and right_fresh and right_stable and bias_valid:
                        x_left_use = x_right_use + float(bias_dx)
                        y_left_use = y_right_use + float(bias_dy)
                        left_syn = True

                    lr_dist = math.hypot(x_left_use - x_right_use, y_left_use - y_right_use)

                    # 短斷線容忍：位移小判斷僅看「健側單一相機的自身 step」，不參考 fuse 距離
                    allow_left_hold = (not left_fresh) and left_recent and right_fresh and right_stable and lr_dist <= STALE_MATCH_MAX_CM
                    allow_right_hold = (not right_fresh) and right_recent and left_fresh and left_stable and lr_dist <= STALE_MATCH_MAX_CM
                    if left_syn:
                        allow_left_hold = True
                    if right_syn:
                        allow_right_hold = True

                    left_for_fuse = left_fresh or allow_left_hold
                    right_for_fuse = right_fresh or allow_right_hold

                    def _apply_blend(w_left, w_right):
                        robot.x = w_left * x_left_use + w_right * x_right_use
                        robot.y = w_left * y_left_use + w_right * y_right_use
                        if right_syn and left_fresh:
                            robot.degree = robot.degree_left
                            robot.degree_base = robot.degree_base_left
                            robot.vec_x = robot.vec_x_left
                            robot.vec_y = robot.vec_y_left
                        elif left_syn and right_fresh:
                            robot.degree = robot.degree_right
                            robot.degree_base = robot.degree_base_right
                            robot.vec_x = robot.vec_x_right
                            robot.vec_y = robot.vec_y_right
                        else:
                            robot.degree = _weighted_mean_angle_deg(robot.degree_left, robot.degree_right, w_left, w_right)
                            robot.degree_base = _weighted_mean_angle_deg(robot.degree_base_left, robot.degree_base_right, w_left, w_right)
                            vx = w_left * robot.vec_x_left + w_right * robot.vec_x_right
                            vy = w_left * robot.vec_y_left + w_right * robot.vec_y_right
                            norm_v = math.hypot(vx, vy)
                            if norm_v > 1e-6:
                                robot.vec_x = vx / norm_v
                                robot.vec_y = vy / norm_v
                            else:
                                robot.vec_x = vx
                                robot.vec_y = vy
                        robot._last_cam_choice = "Blend"

                    def _apply_side(use_left):
                        if use_left:
                            robot.x = robot.x_left
                            robot.y = robot.y_left
                            robot.degree = robot.degree_left
                            robot.degree_base = robot.degree_base_left
                            robot.vec_x = robot.vec_x_left
                            robot.vec_y = robot.vec_y_left
                            robot._last_cam_choice = "Cam1"
                        else:
                            robot.x = robot.x_right
                            robot.y = robot.y_right
                            robot.degree = robot.degree_right
                            robot.degree_base = robot.degree_base_right
                            robot.vec_x = robot.vec_x_right
                            robot.vec_y = robot.vec_y_right
                            robot._last_cam_choice = "Cam2"

                    both_fresh = left_fresh and right_fresh
                    ref_x = None
                    if left_for_fuse and right_for_fuse:
                        # 用上一幀融合結果決定權重中心，避免被單幀偏差推成單邊輸出
                        if math.isfinite(prev_fx):
                            ref_x = prev_fx
                        else:
                            ref_x = 0.5 * (x_left_use + x_right_use)
                    elif left_fresh:
                        ref_x = robot.x_left
                    elif right_fresh:
                        ref_x = robot.x_right

                    def _apply_single_fresh():
                        # 非融合情境：左右優先，但只吃 fresh 偵測
                        prev_choice = getattr(robot, "_last_cam_choice", "Cam1")
                        use_left_cam = prev_choice in ("Cam1", True, "Blend")
                        if ref_x is not None:
                            if ref_x < (field_center_x - HYSTERESIS):
                                use_left_cam = True
                            elif ref_x > (field_center_x + HYSTERESIS):
                                use_left_cam = False

                        if use_left_cam:
                            if left_fresh:
                                _apply_side(True)
                            elif right_fresh:
                                _apply_side(False)
                        else:
                            if right_fresh:
                                _apply_side(False)
                            elif left_fresh:
                                _apply_side(True)

                    if left_for_fuse and right_for_fuse:
                        # fresh+fresh 才做正常混合；且左右差異過大時改用單邊 fresh，避免污染
                        if both_fresh and lr_dist > FRESH_BLEND_MAX_DIST_CM:
                            _apply_single_fresh()
                        elif ref_x is not None:
                            t = (ref_x - (field_center_x - BLEND_BAND)) / (2.0 * BLEND_BAND)
                            t = min(max(float(t), 0.0), 1.0)
                            if both_fresh:
                                t = min(max(t, BLEND_WEIGHT_FLOOR), 1.0 - BLEND_WEIGHT_FLOOR)
                            _apply_blend(1.0 - t, t)
                        else:
                            _apply_blend(0.5, 0.5)
                    else:
                        _apply_single_fresh()

                    # 融合診斷輸出（節流），用於追查跳動原因
                    last_dbg = float(getattr(robot, "_last_fusion_debug_ts", 0.0))
                    if now_ts - last_dbg >= 0.25:
                        robot._last_fusion_debug_ts = now_ts
                        print(
                            "[fuse]"
                            f"[R{robot.id}]"
                            f" fresh(L/R)={int(left_fresh)}/{int(right_fresh)}"
                            f" recent(L/R)={int(left_recent)}/{int(right_recent)}"
                            f" hold(L/R)={int(allow_left_hold)}/{int(allow_right_hold)}"
                            f" syn(L/R)={int(left_syn)}/{int(right_syn)}"
                            f" step(L/R)={left_step:.2f}/{right_step:.2f}"
                            f" lr(cm)={lr_dist:.2f}"
                            f" age(L/R)={left_age:.2f}/{right_age:.2f}s"
                            f" prev=({prev_fx:.1f},{prev_fy:.1f})"
                            f" L=({float(robot.x_left):.1f},{float(robot.y_left):.1f})"
                            f" R=({float(robot.x_right):.1f},{float(robot.y_right):.1f})"
                            f" choice={getattr(robot, '_last_cam_choice', 'N/A')}"
                            f" out=({float(robot.x):.1f},{float(robot.y):.1f})"
                        )
                    
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

                # 收集左上角資訊：X/Y 顯示目前實際採用的主座標，OldX/OldY 保留舊映射供比對
                info_block = [
                    (f"[{cam_role}] ID:{mid}", (0,255,255)),
                    (f"AngBase:{angle:.1f}", (0,255,0)),
                    (f"AngRaw:{raw_angle:.1f}", (0,180,255)),
                    (f"Vec:({vec_x:.2f},{vec_y:.2f})", (200,255,200)),
                    (f"X:{fX:.1f}" if fX is not None else "X:N/A", (0,255,255) if fX is not None else (0,0,255)),
                    (f"Y:{fY:.1f}" if fY is not None else "Y:N/A", (0,255,255) if fY is not None else (0,0,255)),
                ]
                if raw_fX is not None and raw_fY is not None:
                    info_block.append((f"OldX:{raw_fX:.1f}", (255,200,0)))
                    info_block.append((f"OldY:{raw_fY:.1f}", (255,200,0)))
                if ray_fX is not None and ray_fY is not None:
                    info_block.append((f"RayX:{ray_fX:.1f}", (0,255,128)))
                    info_block.append((f"RayY:{ray_fY:.1f}", (0,255,128)))
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
    return gm._normalize_angle(deg)


def _rot_x(deg):
    return gm._rot_x(deg)


def _rot_y(deg):
    return gm._rot_y(deg)


def _rot_z(deg):
    return gm._rot_z(deg)


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
    R_align = _get_face_rotation(robot, marker_id)
    return gm._compute_top_pose_from_align(R_cam_face, tvec, obj_pts, R_align)


def _map_tvec_to_field(tvec, cam_role, config_cam1, config_cam2, marker_id):
    return gm._map_tvec_to_field(tvec, cam_role, config_cam1, config_cam2, marker_id)


# ==========================================
# [射線投影座標校正] — 取代多項式擬合
# ==========================================
FIELD_CORNERS_3D = np.array([
    [0.0,        0.0,         0.0],
    [REAL_WIDTH, 0.0,         0.0],
    [REAL_WIDTH, REAL_HEIGHT, 0.0],
    [0.0,        REAL_HEIGHT, 0.0],
], dtype=np.float64)

# Ray 座標保護：避免外參漂移時直接把結果拉飛
RAY_FIELD_MARGIN_CM = 30.0
RAY_MAX_DEVIATION_CM = 90.0


def _compute_new_camera_matrix(mtx, dist, w, h):
    """Compute optimal new camera matrix for undistorted image."""
    return gm._compute_new_camera_matrix(mtx, dist, w, h)


def _compute_k_rotated(new_mtx, rotation, orig_h, orig_w):
    """Compute camera matrix for a 90-degree rotated image."""
    return gm._compute_k_rotated(new_mtx, rotation, orig_h, orig_w)


def _compute_camera_extrinsics(corner_pixels_sorted, K_rot):
    """Compute camera extrinsics from 4 sorted field corner pixels.

    corner_pixels_sorted: 4 points in rotated undistorted image,
        sorted by sort_points() [TL, TR, BR, BL] corresponding to
        field corners [(0,0), (W,0), (W,H), (0,H)].
    K_rot: camera matrix for rotated undistorted image.

    Returns (R, cam_pos_world) or None if failed.
    """
    out = gm._compute_camera_extrinsics(corner_pixels_sorted, K_rot, FIELD_CORNERS_3D)
    if out is None:
        print("[ray_cast] solvePnP on field corners failed")
        return None
    R, cam_pos = out
    print(f"[ray_cast] Camera pos (cm): X={cam_pos[0,0]:.1f} Y={cam_pos[1,0]:.1f} Z={cam_pos[2,0]:.1f}")
    return R, cam_pos


def _ray_cast_to_height(px, py, K_rot, R, cam_pos, height_cm):
    """Cast a ray from camera through pixel (px, py) in rotated image."""
    return gm._ray_cast_to_height(px, py, K_rot, R, cam_pos, height_cm)


def _is_finite_xy(x, y):
    return gm._is_finite_xy(x, y)


def _should_accept_ray_xy(ray_x, ray_y, mapped_x, mapped_y):
    """Guard ray-cast outliers: keep ray only when it is physically plausible."""
    return gm._should_accept_ray_xy(
        ray_x,
        ray_y,
        mapped_x,
        mapped_y,
        REAL_WIDTH,
        REAL_HEIGHT,
        RAY_FIELD_MARGIN_CM,
        RAY_MAX_DEVIATION_CM,
    )


def _rotate_pixel_to_rotated_frame(x, y, cam_role, frame_h=480, frame_w=640):
    """Transform pixel from undistorted un-rotated frame to rotated frame."""
    return gm._rotate_pixel_to_rotated_frame(x, y, cam_role, frame_h, frame_w)


def _angular_diff(a, b):
    return af._angular_diff(a, b)


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
    """Use aligned rotation matrix to compute heading angle."""
    return gm._compute_angle_from_R(R)


def _blend_angle(prev_deg, curr_deg, alpha=0.1):
    return af._blend_angle(prev_deg, curr_deg, alpha)


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

# Robot ID / ArUco filtering runtime options
DEFAULT_ROBOT_ID_GROUPS = [
    [0, 6, 2, 8, 4],
    [10, 16, 12, 18, 14],
    [20, 26, 22, 28, 24],
]
_robot_id_groups = [ids.copy() for ids in DEFAULT_ROBOT_ID_GROUPS]
FILTER_NON_ROBOT_ARUCO = True
_tracking_cfg_lock = threading.Lock()


def _normalize_robot_id_groups(groups):
    if not isinstance(groups, (list, tuple)) or len(groups) != 3:
        raise ValueError("robot_id_groups 必須是長度 3 的 list/tuple")
    normalized = []
    for idx, group in enumerate(groups):
        if not isinstance(group, (list, tuple)):
            raise ValueError(f"Robot{idx} 的 ID 必須是 list/tuple")
        ids = []
        for raw in group:
            try:
                aid = int(raw)
            except Exception as err:
                raise ValueError(f"Robot{idx} 含有無法轉成整數的 ID: {raw}") from err
            if aid < 0:
                raise ValueError(f"Robot{idx} 含有負值 ID: {aid}")
            ids.append(aid)
        if not ids:
            raise ValueError(f"Robot{idx} 至少要有 1 個 ArUco ID")
        normalized.append(ids)

    flat = [aid for group in normalized for aid in group]
    dup = sorted({aid for aid in flat if flat.count(aid) > 1})
    if dup:
        raise ValueError(f"不同 Robot 的 ID 不可重複: {dup}")
    return normalized


def get_robot_tracking_options():
    """Return current robot-id groups and filtering option for UI settings."""
    with _tracking_cfg_lock:
        return {
            "robot_id_groups": [ids.copy() for ids in _robot_id_groups],
            "filter_non_robot_aruco": bool(FILTER_NON_ROBOT_ARUCO),
        }


def set_robot_tracking_options(robot_id_groups=None, filter_non_robot_aruco=None):
    """
    Update runtime tracking options.
    Note: options are applied on next image thread start/restart.
    """
    global _robot_id_groups, FILTER_NON_ROBOT_ARUCO
    with _tracking_cfg_lock:
        if robot_id_groups is not None:
            _robot_id_groups = _normalize_robot_id_groups(robot_id_groups)
        if filter_non_robot_aruco is not None:
            FILTER_NON_ROBOT_ARUCO = bool(filter_non_robot_aruco)


def _build_robots_from_runtime_options():
    with _tracking_cfg_lock:
        id_groups = [ids.copy() for ids in _robot_id_groups]
    robots = []
    for rid, ids in enumerate(id_groups):
        robots.append(Robot(rid, ids, [0.0] * len(ids), [0.0] * len(ids), [0.0] * len(ids), "right"))
    return robots


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


def set_ball_detection_mode(mode: str):
    """Ball detection is single-only; only accepts 'single'."""
    if my_ball is None:
        return False
    mode_norm = str(mode).strip().lower()
    if mode_norm != "single":
        return False
    my_ball.detect_mode = "single"
    return True


def get_ball_detection_mode():
    """Return current ball detection mode (always single)."""
    if my_ball is not None:
        my_ball.detect_mode = "single"
    return "single"


def get_ball_debug_stats():
    """Return latest ball quality metrics for debug UI."""
    if my_ball is None:
        return {}
    try:
        return my_ball.get_debug_stats()
    except Exception as err:
        return {"error": str(err), "mode": get_ball_detection_mode()}


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
    robots = _build_robots_from_runtime_options()
    robot_0 = robots[0]
    
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
    with _tracking_cfg_lock:
        filter_non_robot_aruco = bool(FILTER_NON_ROBOT_ARUCO)
    print(f"過濾非 Robot ID ArUco: {filter_non_robot_aruco}")
    
    # ==========================================
    # 2. 載入相機校正參數 (共用)
    # ==========================================
    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')
    cap_cfg = get_camera_capture_config()
    cap_w = int(cap_cfg.get("width", 640))
    cap_h = int(cap_cfg.get("height", 480))
    new_mtx_cam0 = _compute_new_camera_matrix(mtx_cam0, dist_cam0, cap_w, cap_h)
    new_mtx_cam2 = _compute_new_camera_matrix(mtx_cam2, dist_cam2, cap_w, cap_h)
    K_rot_cam0 = _compute_k_rotated(new_mtx_cam0, "cw", cap_h, cap_w)
    K_rot_cam2 = _compute_k_rotated(new_mtx_cam2, "ccw", cap_h, cap_w)
    
    # 使用 runtime ArUco 偵測器（可即時覆蓋）
    detector = get_aruco_detector()
    
    # ==========================================
    # 3. 開啟相機
    # ==========================================
    cap0 = open_camera_device(robot_0.CAM1_ID)
    cap2 = open_camera_device(robot_0.CAM2_ID)
    
    _apply_camera_capture_config_to_caps([cap0, cap2])


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
        _apply_camera_capture_config_to_caps([cap0, cap2])
        print(f"[camera] fallback to CAM1={robot_0.CAM1_ID}, CAM2={robot_0.CAM2_ID}")
    _set_active_caps([cap0, cap2])


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
    cam0_extrinsics = None
    cam2_extrinsics = None
    cam0_ext_pts = None
    cam2_ext_pts = None
    detector_rev = None


    try:
        frame_seq = 0
        while True:
            frame_seq += 1
            detector = get_aruco_detector()
            try:
                cur_rev = int(get_aruco_detector_state().get("revision", -1))
            except Exception:
                cur_rev = -1
            if cur_rev != detector_rev:
                detector_rev = cur_rev
                print(f"[aruco] detector switched rev={detector_rev}")
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
            # Update camera extrinsics for ray-cast
            if len(points_cam0) == 4 and (cam0_ext_pts is None or not _points_equal(cam0_ext_pts, points_cam0)):
                cam0_extrinsics = _compute_camera_extrinsics(sort_points(points_cam0), K_rot_cam0)
                cam0_ext_pts = [list(pt) for pt in points_cam0]
            if len(points_cam2) == 4 and (cam2_ext_pts is None or not _points_equal(cam2_ext_pts, points_cam2)):
                cam2_extrinsics = _compute_camera_extrinsics(sort_points(points_cam2), K_rot_cam2)
                cam2_ext_pts = [list(pt) for pt in points_cam2]

            robot_res0 = process_robot_pipeline(
                frame0, detector, mtx_cam0, dist_cam0, target_ids, "Cam1", 
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE,
                pre_undist=frame0_undist,
                filter_non_robot_aruco=filter_non_robot_aruco,
                cam_extrinsics=cam0_extrinsics,
                K_rot=K_rot_cam0,
                frame_seq=frame_seq,
            )
            robot_res2 = process_robot_pipeline(
                frame2, detector, mtx_cam2, dist_cam2, target_ids, "Cam2", 
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE,
                pre_undist=frame2_undist,
                filter_non_robot_aruco=filter_non_robot_aruco,
                cam_extrinsics=cam2_extrinsics,
                K_rot=K_rot_cam2,
                frame_seq=frame_seq,
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
        _clear_active_caps()
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
ball_centers = []

_state_lock = threading.Lock()
_worker = None
_running = False
_last_show_windows = False


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
        ball_centers.clear()
        if my_ball and getattr(my_ball, "centers_real", None):
            ball_centers.extend([[float(x), float(y)] for x, y in my_ball.centers_real[:1]])


def _default_corners_from_frame(frame):
    """Return 4-corner warp points when user has not clicked any."""
    h, w = frame.shape[:2]
    return [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]


def _processing_loop(show_windows=False):
    global _running, fused_preview, robot_cam0_preview, robot_cam2_preview, fused_for_mask
    robots = _build_robots_from_runtime_options()
    robot_0 = robots[0]

    target_ids = [rid for r in robots for rid in r.aruco_id_list]
    robots_dict = {rid: r for r in robots for rid in r.aruco_id_list}
    with _tracking_cfg_lock:
        filter_non_robot_aruco = bool(FILTER_NON_ROBOT_ARUCO)

    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')
    cap_cfg = get_camera_capture_config()
    cap_w = int(cap_cfg.get("width", 640))
    cap_h = int(cap_cfg.get("height", 480))
    # Compute rotated camera matrices for ray-cast coordinate mapping
    new_mtx_cam0 = _compute_new_camera_matrix(mtx_cam0, dist_cam0, cap_w, cap_h)
    new_mtx_cam2 = _compute_new_camera_matrix(mtx_cam2, dist_cam2, cap_w, cap_h)
    K_rot_cam0 = _compute_k_rotated(new_mtx_cam0, "cw", cap_h, cap_w)
    K_rot_cam2 = _compute_k_rotated(new_mtx_cam2, "ccw", cap_h, cap_w)
    detector = get_aruco_detector()

    cap0 = open_camera_device(robot_0.CAM1_ID)
    cap2 = open_camera_device(robot_0.CAM2_ID)
    _apply_camera_capture_config_to_caps([cap0, cap2])

    if not cap0.isOpened() or not cap2.isOpened():
        print("[imageprocess_complete1] Cannot open cameras 0/2")
        _running = False
        return
    _set_active_caps([cap0, cap2])

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
    cam0_extrinsics = None
    cam2_extrinsics = None
    cam0_ext_pts = None
    cam2_ext_pts = None
    detector_rev = None

    try:
        frame_seq = 0
        while _running:
            frame_seq += 1
            detector = get_aruco_detector()
            try:
                cur_rev = int(get_aruco_detector_state().get("revision", -1))
            except Exception:
                cur_rev = -1
            if cur_rev != detector_rev:
                detector_rev = cur_rev
                print(f"[aruco] detector switched rev={detector_rev}")
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

            # Update camera extrinsics for ray-cast (when corner points available)
            if len(points_cam0) == 4 and (cam0_ext_pts is None or not _points_equal(cam0_ext_pts, points_cam0)):
                cam0_extrinsics = _compute_camera_extrinsics(sort_points(points_cam0), K_rot_cam0)
                cam0_ext_pts = [list(pt) for pt in points_cam0]
            if len(points_cam2) == 4 and (cam2_ext_pts is None or not _points_equal(cam2_ext_pts, points_cam2)):
                cam2_extrinsics = _compute_camera_extrinsics(sort_points(points_cam2), K_rot_cam2)
                cam2_ext_pts = [list(pt) for pt in points_cam2]

            # Robot tracking (no UI drawing unless show_windows)
            try:
                robot_res0 = process_robot_pipeline(
                    frame0, detector, mtx_cam0, dist_cam0, target_ids, "Cam1",
                    robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE,
                    pre_undist=frame0_undist,
                    filter_non_robot_aruco=filter_non_robot_aruco,
                    cam_extrinsics=cam0_extrinsics,
                    K_rot=K_rot_cam0,
                    frame_seq=frame_seq,
                )
                robot_res2 = process_robot_pipeline(
                    frame2, detector, mtx_cam2, dist_cam2, target_ids, "Cam2",
                    robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE,
                    pre_undist=frame2_undist,
                    filter_non_robot_aruco=filter_non_robot_aruco,
                    cam_extrinsics=cam2_extrinsics,
                    K_rot=K_rot_cam2,
                    frame_seq=frame_seq,
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
        _clear_active_caps()
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
    global _worker, _running, _last_show_windows
    _last_show_windows = bool(show_windows)
    if _worker and _worker.is_alive():
        _running = False
        _worker.join(timeout=2.5)
        if _worker.is_alive():
            print("[imageprocess] previous worker is still alive; skip restart request")
            return
        _worker = None
    _running = True
    _worker = threading.Thread(target=_processing_loop, args=(show_windows,), daemon=True)
    _worker.start()


def stop_image_thread():
    """Stop vision thread if running."""
    global _worker, _running
    _running = False
    if _worker and _worker.is_alive():
        _worker.join(timeout=2.5)
        if _worker.is_alive():
            print("[imageprocess] worker did not stop cleanly within timeout")
            return
    _worker = None


def get_runtime_state():
    """Return current running state and latest display mode used by start_image_thread."""
    return {
        "running": bool(_running),
        "show_windows": bool(_last_show_windows),
    }


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
