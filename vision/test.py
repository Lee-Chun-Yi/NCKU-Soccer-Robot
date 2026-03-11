import copy
import json
import math
import random
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from tkinter import ttk, messagebox

import imageprocess


DETECT_STEP_NAMES = [
    "gray",
    "clahe",
    "gaussian",
    "unsharp",
    "median",
    "adaptive_threshold",
    "gray_clipped",
    "gamma",
    "upscaled",
]

TEST_UI_PARAMS_FILE = Path(__file__).resolve().parent / "data" / "test_ui_params.json"

_DEFAULT_ARUCO_PARAM_SPECS = [
    {"key": "cornerRefinementMethod", "label": "cornerRefinementMethod", "slider_min": 0, "slider_max": 3, "factor": 1, "kind": "int"},
    {"key": "cornerRefinementWinSize", "label": "cornerRefinementWinSize", "slider_min": 1, "slider_max": 31, "factor": 1, "kind": "int"},
    {"key": "cornerRefinementMaxIterations", "label": "cornerRefinementMaxIterations", "slider_min": 1, "slider_max": 200, "factor": 1, "kind": "int"},
    {"key": "cornerRefinementMinAccuracy", "label": "cornerRefinementMinAccuracy", "slider_min": 1, "slider_max": 1000, "factor": 10000, "kind": "float"},
    {"key": "adaptiveThreshWinSizeMin", "label": "adaptiveThreshWinSizeMin", "slider_min": 3, "slider_max": 151, "factor": 1, "kind": "int"},
    {"key": "adaptiveThreshWinSizeMax", "label": "adaptiveThreshWinSizeMax", "slider_min": 3, "slider_max": 201, "factor": 1, "kind": "int"},
    {"key": "adaptiveThreshWinSizeStep", "label": "adaptiveThreshWinSizeStep", "slider_min": 1, "slider_max": 51, "factor": 1, "kind": "int"},
    {"key": "adaptiveThreshConstant", "label": "adaptiveThreshConstant", "slider_min": -300, "slider_max": 300, "factor": 10, "kind": "float"},
    {"key": "minMarkerPerimeterRate", "label": "minMarkerPerimeterRate", "slider_min": 1, "slider_max": 50000, "factor": 1000000, "kind": "float"},
    {"key": "maxMarkerPerimeterRate", "label": "maxMarkerPerimeterRate", "slider_min": 100, "slider_max": 2000, "factor": 100, "kind": "float"},
    {"key": "minCornerDistanceRate", "label": "minCornerDistanceRate", "slider_min": 1, "slider_max": 50000, "factor": 1000000, "kind": "float"},
    {"key": "minMarkerDistanceRate", "label": "minMarkerDistanceRate", "slider_min": 1, "slider_max": 50000, "factor": 1000000, "kind": "float"},
    {"key": "minDistanceToBorder", "label": "minDistanceToBorder", "slider_min": 0, "slider_max": 100, "factor": 1, "kind": "int"},
    {"key": "minOtsuStdDev", "label": "minOtsuStdDev", "slider_min": 0, "slider_max": 500, "factor": 10, "kind": "float"},
    {"key": "perspectiveRemoveIgnoredMarginPerCell", "label": "perspectiveRemoveIgnoredMarginPerCell", "slider_min": 0, "slider_max": 1000, "factor": 1000, "kind": "float"},
    {"key": "errorCorrectionRate", "label": "errorCorrectionRate", "slider_min": 0, "slider_max": 1000, "factor": 1000, "kind": "float"},
    {"key": "detectInvertedMarker", "label": "detectInvertedMarker", "slider_min": 0, "slider_max": 1, "factor": 1, "kind": "bool"},
]

_TEST_UI_DEFAULTS = {
    "aruco_auto_apply_ms": 120,
    "aruco_param_specs": copy.deepcopy(_DEFAULT_ARUCO_PARAM_SPECS),
    "capture_resolution_options": [[640, 480], [960, 540], [1280, 720], [1600, 900], [1920, 1080]],
    "strong_capture_preset": {
        "width": 1280,
        "height": 720,
        "fps": 30,
        "auto_exposure": False,
        "exposure": -7.0,
        "auto_focus": False,
        "focus": 120.0,
        "gain": 8.0,
        "brightness": -1.0,
        "upscale_factors": [1.4, 1.8, 2.2, 2.6],
    },
}


def _load_test_ui_params():
    cfg = copy.deepcopy(_TEST_UI_DEFAULTS)
    loaded = {}
    try:
        if TEST_UI_PARAMS_FILE.exists():
            with TEST_UI_PARAMS_FILE.open("r", encoding="utf-8") as f:
                loaded = json.load(f)
    except Exception:
        loaded = {}

    if not isinstance(loaded, dict):
        loaded = {}
    if isinstance(loaded.get("aruco_auto_apply_ms"), (int, float)):
        cfg["aruco_auto_apply_ms"] = int(max(30, min(2000, int(loaded["aruco_auto_apply_ms"]))))

    specs = loaded.get("aruco_param_specs")
    if isinstance(specs, list) and specs:
        valid_specs = []
        for item in specs:
            if not isinstance(item, dict):
                continue
            key = str(item.get("key", "")).strip()
            if not key:
                continue
            valid_specs.append(item)
        if valid_specs:
            cfg["aruco_param_specs"] = valid_specs

    options = loaded.get("capture_resolution_options")
    if isinstance(options, list) and options:
        parsed = []
        for entry in options:
            if not isinstance(entry, (list, tuple)) or len(entry) != 2:
                continue
            try:
                w = int(entry[0])
                h = int(entry[1])
            except Exception:
                continue
            if w >= 320 and h >= 240:
                parsed.append([w, h])
        if parsed:
            cfg["capture_resolution_options"] = parsed

    preset = loaded.get("strong_capture_preset")
    if isinstance(preset, dict):
        merged = cfg["strong_capture_preset"].copy()
        merged.update(preset)
        cfg["strong_capture_preset"] = merged

    try:
        TEST_UI_PARAMS_FILE.parent.mkdir(parents=True, exist_ok=True)
        with TEST_UI_PARAMS_FILE.open("w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception:
        pass
    return cfg


_TEST_UI_CFG = _load_test_ui_params()
ARUCO_AUTO_APPLY_MS = int(_TEST_UI_CFG.get("aruco_auto_apply_ms", 120))
ARUCO_PARAM_SPECS = list(_TEST_UI_CFG.get("aruco_param_specs") or [])
CAPTURE_RESOLUTION_OPTIONS = [tuple(v) for v in (_TEST_UI_CFG.get("capture_resolution_options") or [])]
if not CAPTURE_RESOLUTION_OPTIONS:
    CAPTURE_RESOLUTION_OPTIONS = [(640, 480), (960, 540), (1280, 720), (1600, 900), (1920, 1080)]
STRONG_CAPTURE_PRESET = dict(_TEST_UI_CFG.get("strong_capture_preset") or {})
ARUCO_PARAM_IMPACTS = {
    "cornerRefinementMethod": {
        "up": "改用更強的角點細化法，角點較準，但 CPU 成本提高。",
        "down": "改用較簡單方法，速度較快，但姿態角度可能更抖。",
        "tip": "通常 3(APRILTAG) 最穩，0 速度最快。",
    },
    "cornerRefinementWinSize": {
        "up": "細化視窗更大，對模糊邊緣較穩，但容易吃到鄰近雜訊。",
        "down": "細化更局部，對細節乾淨畫面快，但抗噪較弱。",
        "tip": "過大可能讓小標記角點偏移。",
    },
    "cornerRefinementMaxIterations": {
        "up": "角點迭代更久，理論上更精細，但延遲增加。",
        "down": "提早停止，速度提升，但角點收斂可能不足。",
        "tip": "先調到不卡，再微調精度。",
    },
    "cornerRefinementMinAccuracy": {
        "up": "較快達到停止條件，速度增加，但細化精度可能下降。",
        "down": "要求更嚴格，角點較準，但計算時間增加。",
        "tip": "太小會拖慢整體 FPS。",
    },
    "adaptiveThreshWinSizeMin": {
        "up": "最小視窗變大，較不敏感雜訊，但小細節可能被吃掉。",
        "down": "更小視窗，對局部光變敏感，但噪點誤檢風險增。",
        "tip": "需與 Max/Step 一起看。",
    },
    "adaptiveThreshWinSizeMax": {
        "up": "可嘗試更大視窗，強光影變化下較穩，但更慢。",
        "down": "範圍縮小，速度變快，但極端光照適應性下降。",
        "tip": "太大常帶來延遲且收益有限。",
    },
    "adaptiveThreshWinSizeStep": {
        "up": "測試視窗數量減少，速度提升，但最佳視窗可能被跳過。",
        "down": "掃描更密，找到最佳二值化機率高，但更耗時。",
        "tip": "調小常見於難光照場景。",
    },
    "adaptiveThreshConstant": {
        "up": "二值門檻整體變嚴，背景抑制較強，可能漏掉淺色邊緣。",
        "down": "門檻變寬鬆，弱對比標記較易出現，也更易引入雜訊。",
        "tip": "光線穩定時可稍大，低對比時可稍小。",
    },
    "minMarkerPerimeterRate": {
        "up": "忽略更小的候選框，誤檢減少，但遠距小標記容易漏檢。",
        "down": "允許更小標記，遠距檢出率提升，但噪點誤檢增加。",
        "tip": "小標記偵測先往下調。",
    },
    "maxMarkerPerimeterRate": {
        "up": "允許更大候選框，近距大標記更不易漏掉。",
        "down": "限制大候選，誤檢可降，但超近距標記可能漏檢。",
        "tip": "一般維持較大上限較安全。",
    },
    "minCornerDistanceRate": {
        "up": "角點需更分散，可抑制扭曲假標記，但小標記可能被排除。",
        "down": "接受更靠近的角點，小標記較易檢出，誤檢風險升。",
        "tip": "過低常導致假框增加。",
    },
    "minMarkerDistanceRate": {
        "up": "相鄰標記需更分開，可減少重疊誤判。",
        "down": "允許標記更靠近，密集場景較能同時檢出。",
        "tip": "多標記貼近時可略降。",
    },
    "minDistanceToBorder": {
        "up": "更排斥靠邊標記，邊界誤檢減少，但邊緣真標記會漏。",
        "down": "允許靠邊標記，提高邊界檢出率，也提高假檢率。",
        "tip": "鏡頭常裁邊時不宜太大。",
    },
    "minOtsuStdDev": {
        "up": "要求更高對比，低對比雜訊會被忽略。",
        "down": "低對比也嘗試辨識，暗場可提升檢出，但較不穩。",
        "tip": "低光環境常需要降低。",
    },
    "perspectiveRemoveIgnoredMarginPerCell": {
        "up": "單元邊緣忽略範圍變大，邊界污染影響降低。",
        "down": "保留更多邊緣像素，理論資訊更多，但受噪聲影響較大。",
        "tip": "畫面模糊或反光時可稍微提高。",
    },
    "errorCorrectionRate": {
        "up": "容錯更高，破損標記較易被接受，也可能誤識別 ID。",
        "down": "容錯更低，ID 更保守，破損標記較容易被丟棄。",
        "tip": "ID 常跳號時可先降低。",
    },
    "detectInvertedMarker": {
        "up": "可偵測反色標記，覆蓋更多情況，但檢測成本增加。",
        "down": "只找正常色標記，速度略快、誤檢機率略降。",
        "tip": "場上沒有反色標記可關閉。",
    },
}


_state_lock = threading.Lock()
_hook_lock = threading.Lock()
_hooks_installed = False

_debug_state = {
    "cams": {
        "Cam1": {},
        "Cam2": {},
    },
    "robots": {},
    "events": deque(maxlen=120),
}


def _fmt_num(value, digits=2):
    if value is None:
        return "N/A"
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return "N/A"


def _fmt_ms(value, digits=2):
    if value is None:
        return "N/A"
    try:
        return f"{float(value):.{digits}f}ms"
    except Exception:
        return "N/A"


def _fmt_pct(value, digits=1):
    if value is None:
        return "N/A"
    try:
        return f"{float(value) * 100.0:.{digits}f}%"
    except Exception:
        return "N/A"


def _as_vec(arr, max_items=3):
    if arr is None:
        return []
    flat = np.asarray(arr, dtype=np.float64).reshape(-1)
    return [round(float(v), 4) for v in flat[:max_items]]


def _as_points(arr, decimals=1):
    if arr is None:
        return []
    pts = np.asarray(arr, dtype=np.float64).reshape(-1, 2)
    return [[round(float(x), decimals), round(float(y), decimals)] for x, y in pts]


def _image_stats(img):
    if img is None:
        return {}
    arr = np.asarray(img)
    return {
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
        "mean": round(float(arr.mean()), 2),
        "std": round(float(arr.std()), 2),
    }


def _stage_stats(values):
    if not values:
        return {
            "count": 0,
            "avg_ms": None,
            "max_ms": None,
            "sum_ms": 0.0,
        }
    vals = [float(v) for v in values]
    return {
        "count": len(vals),
        "avg_ms": float(sum(vals) / len(vals)),
        "max_ms": float(max(vals)),
        "sum_ms": float(sum(vals)),
    }


def _build_timing_summary(timings):
    return {
        "undistort_ms": timings.get("undistort_ms"),
        "gray_ms": timings.get("gray_ms"),
        "process_core_ms": timings.get("process_core_ms"),
        "pipeline_total_ms": timings.get("pipeline_total_ms"),
        "detect": _stage_stats(timings.get("detect_ms", [])),
        "solvepnp": _stage_stats(timings.get("solvepnp_ms", [])),
        "rodrigues": _stage_stats(timings.get("rodrigues_ms", [])),
        "top_pose": _stage_stats(timings.get("top_pose_ms", [])),
        "map_tvec": _stage_stats(timings.get("map_tvec_ms", [])),
        "ray_cast": _stage_stats(timings.get("ray_cast_ms", [])),
        "angle_from_r": _stage_stats(timings.get("angle_from_r_ms", [])),
    }


def _ensure_marker_slot(ctx, marker_id):
    mid = int(marker_id)
    markers = ctx["markers"]
    if mid not in markers:
        markers[mid] = {}
    return markers[mid]


def _match_marker_id(image_points, ctx):
    corner_map = ctx.get("detected_corner_raw") or {}
    if not corner_map:
        return None
    target = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    best_id = None
    best_err = float("inf")
    for mid, pts in corner_map.items():
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        if p.shape != target.shape:
            continue
        err = float(np.mean(np.abs(p - target)))
        if err < best_err:
            best_err = err
            best_id = int(mid)
    if best_err > 2.0:
        return None
    return best_id


def _snapshot_robots(robots_dict):
    snapshot = {}
    seen = set()
    for r in robots_dict.values():
        rid = int(r.id)
        if rid in seen:
            continue
        seen.add(rid)
        choice = getattr(r, "_last_cam_choice", None)
        if choice in ("Cam1", "Cam2", "Blend"):
            chosen = choice
        elif choice is True:
            chosen = "Cam1"
        elif choice is False:
            chosen = "Cam2"
        else:
            chosen = "N/A"
        snapshot[rid] = {
            "chosen": chosen,
            "final": {
                "x": float(r.x),
                "y": float(r.y),
                "deg": float(r.degree),
                "vec_x": float(r.vec_x),
                "vec_y": float(r.vec_y),
            },
            "left": {
                "x": float(r.x_left),
                "y": float(r.y_left),
                "deg": float(r.degree_left),
                "vec_x": float(r.vec_x_left),
                "vec_y": float(r.vec_y_left),
                "count": int(r.amount_choose_left),
            },
            "right": {
                "x": float(r.x_right),
                "y": float(r.y_right),
                "deg": float(r.degree_right),
                "vec_x": float(r.vec_x_right),
                "vec_y": float(r.vec_y_right),
                "count": int(r.amount_choose_right),
            },
        }
    return snapshot


def _add_event(message):
    with _state_lock:
        _debug_state["events"].append(f"{time.strftime('%H:%M:%S')} | {message}")


def install_hooks():
    global _hooks_installed
    if _hooks_installed:
        return
    original_process = imageprocess.process_robot_pipeline

    def wrapped_process(
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
        detector_revision = None
        if hasattr(imageprocess, "get_aruco_detector_state"):
            try:
                detector_revision = int((imageprocess.get_aruco_detector_state() or {}).get("revision"))
            except Exception:
                detector_revision = None
        t_pipeline_start = time.perf_counter()
        t0 = time.perf_counter()
        frame_undist = pre_undist if pre_undist is not None else cv2.undistort(frame, mtx, dist)
        undistort_ms = (time.perf_counter() - t0) * 1000.0
        t0 = time.perf_counter()
        gray = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2GRAY)
        gray_ms = (time.perf_counter() - t0) * 1000.0

        ctx = {
            "cam_role": cam_role,
            "started_at": time.time(),
            "detector_revision": detector_revision,
            "undistort": _image_stats(frame_undist),
            "gray": _image_stats(gray),
            "detect_attempts": [],
            "detected_ids": [],
            "detected_corner_raw": {},
            "detected_corner_view": {},
            "markers": {},
            "map_call_count": {},
            "active_marker_id": None,
            "timings": {
                "undistort_ms": undistort_ms,
                "gray_ms": gray_ms,
                "process_core_ms": None,
                "pipeline_total_ms": None,
                "detect_ms": [],
                "solvepnp_ms": [],
                "rodrigues_ms": [],
                "top_pose_ms": [],
                "map_tvec_ms": [],
                "ray_cast_ms": [],
                "angle_from_r_ms": [],
            },
        }

        class DetectorProxy:
            def __init__(self, base_detector, context):
                self._base = base_detector
                self._ctx = context
                self._call_idx = 0

            def detectMarkers(self, gray_img):
                # Always fetch latest detector to support true hot-swap from Param tab.
                active = self._base
                if hasattr(imageprocess, "get_aruco_detector"):
                    try:
                        active = imageprocess.get_aruco_detector()
                    except Exception:
                        active = self._base
                t_detect = time.perf_counter()
                corners, ids, rejected = active.detectMarkers(gray_img)
                detect_ms = (time.perf_counter() - t_detect) * 1000.0
                if self._call_idx < len(DETECT_STEP_NAMES):
                    step_name = DETECT_STEP_NAMES[self._call_idx]
                else:
                    extra_idx = self._call_idx - len(DETECT_STEP_NAMES) + 2
                    step_name = f"upscaled_x{extra_idx}"
                self._call_idx += 1
                ids_list = [] if ids is None else [int(v) for v in ids.flatten()]
                self._ctx["timings"]["detect_ms"].append(detect_ms)
                self._ctx["detect_attempts"].append(
                    {
                        "step": step_name,
                        "ids": ids_list,
                        "count": len(ids_list),
                        "img_mean": round(float(np.asarray(gray_img).mean()), 2),
                        "latency_ms": round(float(detect_ms), 3),
                    }
                )
                if ids is not None:
                    self._ctx["detected_ids"] = ids_list
                    self._ctx["detected_corner_raw"] = {}
                    self._ctx["detected_corner_view"] = {}
                    for idx, mid in enumerate(ids_list):
                        pts = np.asarray(corners[idx], dtype=np.float64).reshape(-1, 2)
                        self._ctx["detected_corner_raw"][mid] = pts.copy()
                        self._ctx["detected_corner_view"][mid] = _as_points(pts, decimals=1)
                return corners, ids, rejected

            def __getattr__(self, item):
                return getattr(self._base, item)

        detector_proxy = DetectorProxy(detector, ctx)

        with _hook_lock:
            orig_solve_pnp = imageprocess.cv2.solvePnP
            orig_rodrigues = imageprocess.cv2.Rodrigues
            orig_top_pose = imageprocess._compute_top_pose
            orig_map_tvec = imageprocess._map_tvec_to_field
            orig_ray_cast = imageprocess._ray_cast_to_height
            orig_angle_from_r = imageprocess._compute_angle_from_R

            def solve_pnp_hook(object_points, image_points, camera_matrix, dist_coeffs, *args, **kwargs):
                t_stage = time.perf_counter()
                success, rvec, tvec = orig_solve_pnp(object_points, image_points, camera_matrix, dist_coeffs, *args, **kwargs)
                solve_ms = (time.perf_counter() - t_stage) * 1000.0
                ctx["timings"]["solvepnp_ms"].append(solve_ms)
                marker_id = _match_marker_id(image_points, ctx)
                if marker_id is not None:
                    ctx["active_marker_id"] = marker_id
                    slot = _ensure_marker_slot(ctx, marker_id)
                    slot["solvePnP"] = {
                        "success": bool(success),
                        "rvec": _as_vec(rvec, 3),
                        "tvec": _as_vec(tvec, 3),
                        "latency_ms": round(float(solve_ms), 3),
                    }
                return success, rvec, tvec

            def rodrigues_hook(src, *args, **kwargs):
                t_stage = time.perf_counter()
                dst, jac = orig_rodrigues(src, *args, **kwargs)
                rod_ms = (time.perf_counter() - t_stage) * 1000.0
                arr = np.asarray(src)
                if arr.shape != (3, 3) and arr.size == 3:
                    ctx["timings"]["rodrigues_ms"].append(rod_ms)
                    marker_id = ctx.get("active_marker_id")
                    if marker_id is not None:
                        slot = _ensure_marker_slot(ctx, marker_id)
                        slot["Rodrigues_R"] = np.round(np.asarray(dst, dtype=np.float64), 4).tolist()
                        slot["Rodrigues_latency_ms"] = round(float(rod_ms), 3)
                return dst, jac

            def top_pose_hook(R_cam_face, tvec, obj_pts, robot, marker_id):
                t_stage = time.perf_counter()
                r_top, t_top, side_len, n_hat, cos_theta = orig_top_pose(R_cam_face, tvec, obj_pts, robot, marker_id)
                top_ms = (time.perf_counter() - t_stage) * 1000.0
                ctx["timings"]["top_pose_ms"].append(top_ms)
                ctx["active_marker_id"] = int(marker_id)
                slot = _ensure_marker_slot(ctx, marker_id)
                slot["top_pose"] = {
                    "tvec_top": _as_vec(t_top, 3),
                    "cos_theta": _fmt_num(cos_theta, 3),
                    "side_len": _fmt_num(side_len, 3),
                    "n_hat": _as_vec(n_hat, 3),
                    "latency_ms": round(float(top_ms), 3),
                }
                return r_top, t_top, side_len, n_hat, cos_theta

            def map_tvec_hook(tvec, role, cfg1, cfg2, marker_id):
                t_stage = time.perf_counter()
                fx, fy = orig_map_tvec(tvec, role, cfg1, cfg2, marker_id)
                map_ms = (time.perf_counter() - t_stage) * 1000.0
                ctx["timings"]["map_tvec_ms"].append(map_ms)
                slot = _ensure_marker_slot(ctx, marker_id)
                call_count = ctx["map_call_count"].get(int(marker_id), 0)
                stage_key = "map_raw" if call_count == 0 else "map_top"
                poly = _poly_terms_from_tvec(tvec, role, cfg1, cfg2)
                slot[stage_key] = {
                    "fX": fx,
                    "fY": fy,
                    "tvec": _as_vec(tvec, 3),
                    "latency_ms": round(float(map_ms), 3),
                }
                if poly:
                    slot[stage_key].update(
                        {
                            "X_raw": poly.get("X_raw"),
                            "Y_raw": poly.get("Y_raw"),
                            "Xa": poly.get("Xa"),
                            "Ya": poly.get("Ya"),
                            "Xb": fx,
                            "Yb": fy,
                        }
                    )
                ctx["map_call_count"][int(marker_id)] = call_count + 1
                ctx["active_marker_id"] = int(marker_id)
                return fx, fy

            def ray_cast_hook(px, py, k_mat, rot_mat, cam_pos, height_cm):
                t_stage = time.perf_counter()
                fx, fy = orig_ray_cast(px, py, k_mat, rot_mat, cam_pos, height_cm)
                ray_ms = (time.perf_counter() - t_stage) * 1000.0
                ctx["timings"]["ray_cast_ms"].append(ray_ms)
                marker_id = ctx.get("active_marker_id")
                if marker_id is not None:
                    slot = _ensure_marker_slot(ctx, marker_id)
                    slot["ray_cast"] = {
                        "pixel": [round(float(px), 2), round(float(py), 2)],
                        "height_cm": _fmt_num(height_cm, 2),
                        "fX": fx,
                        "fY": fy,
                        "latency_ms": round(float(ray_ms), 3),
                    }
                return fx, fy

            def angle_from_r_hook(R):
                t_stage = time.perf_counter()
                angle = orig_angle_from_r(R)
                ang_ms = (time.perf_counter() - t_stage) * 1000.0
                ctx["timings"]["angle_from_r_ms"].append(ang_ms)
                marker_id = ctx.get("active_marker_id")
                if marker_id is not None:
                    slot = _ensure_marker_slot(ctx, marker_id)
                    slot["angle_from_R_top"] = angle
                    slot["angle_latency_ms"] = round(float(ang_ms), 3)
                    forward = np.asarray(R, dtype=np.float64)[:, 1]
                    norm_xy = math.hypot(float(forward[0]), float(forward[1])) or 1.0
                    vec_x = float(forward[1]) / norm_xy
                    vec_y = float(forward[0]) / norm_xy
                    if cam_role == "Cam2":
                        vec_x = -vec_x
                    elif cam_role == "Cam1":
                        vec_y = -vec_y
                    slot["direction_vec"] = {
                        "R_top_col1": [round(float(forward[0]), 4), round(float(forward[1]), 4), round(float(forward[2]), 4)],
                        "vec_x": vec_x,
                        "vec_y": vec_y,
                    }
                return angle

            imageprocess.cv2.solvePnP = solve_pnp_hook
            imageprocess.cv2.Rodrigues = rodrigues_hook
            imageprocess._compute_top_pose = top_pose_hook
            imageprocess._map_tvec_to_field = map_tvec_hook
            imageprocess._ray_cast_to_height = ray_cast_hook
            imageprocess._compute_angle_from_R = angle_from_r_hook

            try:
                t_core = time.perf_counter()
                result = original_process(
                    frame,
                    detector_proxy,
                    mtx,
                    dist,
                    target_ids,
                    cam_role,
                    robots_dict,
                    config_cam1,
                    config_cam2,
                    marker_size,
                    pre_undist=pre_undist,
                    filter_non_robot_aruco=filter_non_robot_aruco,
                    cam_extrinsics=cam_extrinsics,
                    K_rot=K_rot,
                    frame_seq=frame_seq,
                )
                ctx["timings"]["process_core_ms"] = (time.perf_counter() - t_core) * 1000.0
            finally:
                imageprocess.cv2.solvePnP = orig_solve_pnp
                imageprocess.cv2.Rodrigues = orig_rodrigues
                imageprocess._compute_top_pose = orig_top_pose
                imageprocess._map_tvec_to_field = orig_map_tvec
                imageprocess._ray_cast_to_height = orig_ray_cast
                imageprocess._compute_angle_from_R = orig_angle_from_r

        ctx["timings"]["pipeline_total_ms"] = (time.perf_counter() - t_pipeline_start) * 1000.0
        timing_summary = _build_timing_summary(ctx["timings"])
        robots_snapshot = _snapshot_robots(robots_dict)
        with _state_lock:
            _debug_state["cams"][cam_role] = {
                "updated_at": time.time(),
                "detector_revision": ctx.get("detector_revision"),
                "undistort": ctx["undistort"],
                "gray": ctx["gray"],
                "detect_attempts": ctx["detect_attempts"],
                "detected_ids": ctx["detected_ids"],
                "detected_corner_view": ctx["detected_corner_view"],
                "markers": ctx["markers"],
                "timings": timing_summary,
            }
            _debug_state["robots"] = robots_snapshot

        _add_event(
            f"[{cam_role}] rev={ctx.get('detector_revision')} IDs={ctx['detected_ids']} markers={list(ctx['markers'].keys())} total={_fmt_ms(timing_summary.get('pipeline_total_ms'))}"
        )
        return result

    imageprocess.process_robot_pipeline = wrapped_process
    _hooks_installed = True


def _valid_points(points):
    return (
        isinstance(points, list)
        and len(points) == 4
        and all(isinstance(p, (list, tuple)) and len(p) == 2 for p in points)
    )


def _normalize_points(points):
    normalized = []
    for p in list(points or []):
        if not isinstance(p, (list, tuple)) or len(p) != 2:
            continue
        try:
            normalized.append((int(round(float(p[0]))), int(round(float(p[1])))))
        except Exception:
            continue
    return normalized


def apply_saved_calibration_points():
    calib_file = Path(str(imageprocess.CALIB_POINTS_FILE))
    if not calib_file.exists():
        return False
    try:
        with calib_file.open("r", encoding="utf-8") as f:
            data = json.load(f)
        cam0 = data.get("points_cam0")
        cam2 = data.get("points_cam2")
        if _valid_points(cam0):
            imageprocess.points_cam0[:] = [tuple(map(int, p)) for p in cam0]
        if _valid_points(cam2):
            imageprocess.points_cam2[:] = [tuple(map(int, p)) for p in cam2]
        return _valid_points(cam0) and _valid_points(cam2)
    except Exception:
        return False


def _deep_state_copy():
    with _state_lock:
        return {
            "cams": {
                "Cam1": copy.deepcopy(_debug_state["cams"]["Cam1"]),
                "Cam2": copy.deepcopy(_debug_state["cams"]["Cam2"]),
            },
            "robots": copy.deepcopy(_debug_state["robots"]),
            "events": list(_debug_state["events"]),
        }


def _get_main_xy_from_marker(marker):
    ray = marker.get("ray_cast")
    m_top = marker.get("map_top")
    if ray is not None and ray.get("fX") is not None and ray.get("fY") is not None:
        ray_x = ray.get("fX")
        ray_y = ray.get("fY")
        map_x = m_top.get("fX") if isinstance(m_top, dict) else None
        map_y = m_top.get("fY") if isinstance(m_top, dict) else None
        use_ray = True
        if hasattr(imageprocess, "_should_accept_ray_xy"):
            try:
                use_ray = bool(imageprocess._should_accept_ray_xy(ray_x, ray_y, map_x, map_y))
            except Exception:
                use_ray = True
        if use_ray:
            return ray_x, ray_y, "Ray"
    if m_top is not None and m_top.get("fX") is not None and m_top.get("fY") is not None:
        return m_top.get("fX"), m_top.get("fY"), "MapTop"
    m_raw = marker.get("map_raw")
    if m_raw is not None and m_raw.get("fX") is not None and m_raw.get("fY") is not None:
        return m_raw.get("fX"), m_raw.get("fY"), "MapRaw"
    return None, None, "N/A"


def _poly_terms_from_tvec(tvec, cam_role, config_cam1, config_cam2):
    try:
        z_cam = float(tvec[2][0])
        x_cam = float(tvec[1][0])
        y_cam = float(tvec[0][0])
    except Exception:
        return {}
    if cam_role == "Cam1":
        try:
            h = float(config_cam1.get("camera_height"))
            h_robot = float(config_cam1.get("robot_height"))
        except Exception:
            return {}
        val = (x_cam ** 2 + z_cam ** 2) - (h - h_robot) ** 2
        if val < 0:
            return {}
        x_raw = math.sqrt(val)
        y_raw = y_cam
        xa = (x_raw - 1.0) * 100.0
        ya = (y_raw + 1.3) * 100.0
    else:
        try:
            h = float(config_cam2.get("camera_height"))
            h_robot = float(config_cam2.get("robot_height"))
        except Exception:
            return {}
        val = (x_cam ** 2 + z_cam ** 2) - (h - h_robot) ** 2
        if val < 0:
            return {}
        x_raw = math.sqrt(val)
        y_raw = y_cam
        xa = 360.0 - (x_raw - 1.0) * 100.0
        ya = (1.3 - y_raw) * 100.0
    return {
        "X_raw": float(x_raw),
        "Y_raw": float(y_raw),
        "Xa": float(xa),
        "Ya": float(ya),
    }


def _fmt_marker_block(mid, marker, verbose=True):
    lines = [f"ID {mid}"]
    stale_age = marker.get("__stale_age_sec")
    if isinstance(stale_age, (int, float)) and stale_age >= 0.0:
        lines.append(f"  [暫存] 最近偵測於 {float(stale_age):.2f}s 前")
    main_x, main_y, main_src = _get_main_xy_from_marker(marker)
    lines.append(
        f"  X/Y(主座標) = {_fmt_num(main_x)} / {_fmt_num(main_y)}  source={main_src}"
    )
    if "ray_cast" in marker:
        ray = marker["ray_cast"]
        lines.append(
            f"  RayX/RayY = {_fmt_num(ray.get('fX'))} / {_fmt_num(ray.get('fY'))} (射線投影)"
        )
    if not verbose:
        return lines
    if "solvePnP" in marker:
        s = marker["solvePnP"]
        lines.append(
            f"  solvePnP rvec={s.get('rvec')} tvec={s.get('tvec')} dt={_fmt_ms(s.get('latency_ms'), 3)}"
        )
    if "Rodrigues_R" in marker:
        r0 = marker["Rodrigues_R"][0]
        lines.append(
            f"  Rodrigues R[0]={r0} dt={_fmt_ms(marker.get('Rodrigues_latency_ms'), 3)}"
        )
    if "top_pose" in marker:
        t = marker["top_pose"]
        lines.append(
            f"  top_pose tvec_top={t.get('tvec_top')} cos={t.get('cos_theta')} dt={_fmt_ms(t.get('latency_ms'), 3)}"
        )
    if "map_raw" in marker:
        p = marker["map_raw"]
        lines.append(
            f"  map(raw) fX={_fmt_num(p.get('fX'))} fY={_fmt_num(p.get('fY'))} dt={_fmt_ms(p.get('latency_ms'), 3)}"
        )
        if any(k in p for k in ("Xa", "Ya", "Xb", "Yb")):
            lines.append(
                f"    Xa/Ya={_fmt_num(p.get('Xa'))}/{_fmt_num(p.get('Ya'))}  "
                f"Xb/Yb={_fmt_num(p.get('Xb'))}/{_fmt_num(p.get('Yb'))}"
            )
    if "map_top" in marker:
        p = marker["map_top"]
        lines.append(
            f"  map(top) fX={_fmt_num(p.get('fX'))} fY={_fmt_num(p.get('fY'))} dt={_fmt_ms(p.get('latency_ms'), 3)}"
        )
        if any(k in p for k in ("Xa", "Ya", "Xb", "Yb")):
            lines.append(
                f"    Xa/Ya={_fmt_num(p.get('Xa'))}/{_fmt_num(p.get('Ya'))}  "
                f"Xb/Yb={_fmt_num(p.get('Xb'))}/{_fmt_num(p.get('Yb'))}"
            )
    if "ray_cast" in marker:
        p = marker["ray_cast"]
        lines.append(
            f"  ray fX={_fmt_num(p.get('fX'))} fY={_fmt_num(p.get('fY'))} dt={_fmt_ms(p.get('latency_ms'), 3)}"
        )
    if "angle_from_R_top" in marker:
        lines.append(
            f"  angle(R_top)={_fmt_num(marker.get('angle_from_R_top'))} dt={_fmt_ms(marker.get('angle_latency_ms'), 3)}"
        )
    if "direction_vec" in marker:
        v = marker["direction_vec"]
        lines.append(
            f"  vec from R_top[:,1] => vec_x={_fmt_num(v.get('vec_x'), 3)} vec_y={_fmt_num(v.get('vec_y'), 3)}"
        )
    return lines


def _marker_ids_sorted(markers):
    mids = []
    for k in dict(markers or {}).keys():
        try:
            mids.append(int(k))
        except Exception:
            continue
    return sorted(mids)


def _pick_detail_marker_id(markers, ids_list, requested):
    marker_ids = _marker_ids_sorted(markers)
    if not marker_ids:
        return None
    wanted = str(requested or "").strip().lower()
    if wanted not in ("", "auto", "*"):
        try:
            rid = int(wanted)
            if rid in marker_ids:
                return rid
        except Exception:
            pass
    for raw in ids_list or []:
        try:
            rid = int(raw)
        except Exception:
            continue
        if rid in marker_ids:
            return rid
    return marker_ids[0]


def _fmt_cam_text(cam_name, cam_data, detail_enabled=False, detail_id="auto"):
    if not isinstance(cam_data, dict):
        cam_data = {}
    lines = [f"{cam_name} Pipeline"]
    lines.append(f"Detector Revision: {cam_data.get('detector_revision', 'N/A')}")
    und = cam_data.get("undistort", {})
    gry = cam_data.get("gray", {})
    lines.append(f"去畸變: shape={und.get('shape')} mean={und.get('mean')} std={und.get('std')}")
    lines.append(f"灰階: shape={gry.get('shape')} mean={gry.get('mean')} std={gry.get('std')}")
    timing = cam_data.get("timings", {})
    detect_t = timing.get("detect", {})
    solve_t = timing.get("solvepnp", {})
    rod_t = timing.get("rodrigues", {})
    top_t = timing.get("top_pose", {})
    map_t = timing.get("map_tvec", {})
    ray_t = timing.get("ray_cast", {})
    ang_t = timing.get("angle_from_r", {})
    lines.append(
        "延遲: "
        f"total={_fmt_ms(timing.get('pipeline_total_ms'))}, "
        f"core={_fmt_ms(timing.get('process_core_ms'))}, "
        f"undist={_fmt_ms(timing.get('undistort_ms'))}, "
        f"gray={_fmt_ms(timing.get('gray_ms'))}"
    )

    attempts = cam_data.get("detect_attempts", [])
    if attempts:
        a0 = attempts[0]
        lines.append(
            f"detectMarkers: {a0.get('step')} ids={a0.get('ids')} dt={_fmt_ms(a0.get('latency_ms'), 3)} "
            f"(其餘階段已折疊)"
        )
    else:
        lines.append("detectMarkers: N/A")

    ids_list = list(cam_data.get("detected_ids") or [])
    lines.append(f"IDs: {ids_list}  count={len(ids_list)}")
    markers = cam_data.get("markers", {})
    marker_ids = _marker_ids_sorted(markers)
    lines.append("Summary(固定模板, Top-3):")
    for idx in range(3):
        if idx >= len(marker_ids):
            lines.append("  -")
            continue
        mid = marker_ids[idx]
        marker = markers.get(mid) or markers.get(str(mid)) or {}
        main_x, main_y, main_src = _get_main_xy_from_marker(marker)
        ang = marker.get("angle_from_R_top")
        lines.append(
            f"  ID {mid}: XY={_fmt_num(main_x)}/{_fmt_num(main_y)} src={main_src} "
            f"ang={_fmt_num(ang)}"
        )

    corner_map = cam_data.get("detected_corner_view", {}) or {}
    if detail_enabled:
        focus_mid = _pick_detail_marker_id(markers, ids_list, detail_id)
        if focus_mid is None:
            lines.append("Detail: 無可展開 ID")
        else:
            lines.append(f"Detail(ID={focus_mid}):")
            if focus_mid in corner_map:
                lines.append(f"  corner={corner_map[focus_mid]}")
            elif str(focus_mid) in corner_map:
                lines.append(f"  corner={corner_map[str(focus_mid)]}")
            marker = markers.get(focus_mid) or markers.get(str(focus_mid)) or {}
            lines.extend(_fmt_marker_block(focus_mid, marker, verbose=True))
    else:
        lines.append("Detail: OFF（勾選「詳細」並填 ID 可展開 solvePnP/Rodrigues/top_pose/map/ray）")
    return "\n".join(lines)


def _fmt_ball_centers(centers):
    if not centers:
        return "[]"
    try:
        pt = centers[0]
        x = float(pt[0])
        y = float(pt[1])
        return f"[({x:.2f},{y:.2f})]"
    except Exception:
        return "[]"


def _fmt_ball_window(tag, w):
    if not w:
        return [f"{tag}: N/A"]
    lines = [f"{tag} ({_fmt_num(w.get('window_sec'), 1)}s)"]
    lines.append(
        f"  frames={w.get('frames', 0)} fps={_fmt_num(w.get('fps'), 2)} "
        f"tracked={w.get('tracked_frames', 0)}({_fmt_pct(w.get('tracked_ratio'))}) "
        f"fallback={w.get('fallback_frames', 0)}({_fmt_pct(w.get('fallback_ratio'))})"
    )
    lines.append(
        f"  latency avg/p95={_fmt_ms(w.get('latency_avg_ms'))}/{_fmt_ms(w.get('latency_p95_ms'))} "
        f"mask(avg)={_fmt_pct(w.get('mask_ratio_avg'))} hough(avg)={_fmt_num(w.get('hough_count_avg'), 2)} "
        f"count(avg)={_fmt_num(w.get('detected_count_avg'), 2)}"
    )
    mean_center = w.get("mean_center_cm")
    if isinstance(mean_center, list) and len(mean_center) >= 2:
        center_txt = f"({_fmt_num(mean_center[0])}, {_fmt_num(mean_center[1])})"
    else:
        center_txt = "N/A"
    lines.append(
        f"  mean_center(cm)={center_txt} radius avg/std(px)="
        f"{_fmt_num(w.get('radius_avg_px'))}/{_fmt_num(w.get('radius_std_px'))}"
    )
    lines.append(
        f"  jitter cm(rms/peak)={_fmt_num(w.get('jitter_cm_rms'))}/{_fmt_num(w.get('jitter_cm_peak'))} "
        f"px(rms/peak)={_fmt_num(w.get('jitter_px_rms'))}/{_fmt_num(w.get('jitter_px_peak'))}"
    )
    lines.append(
        f"  step cm(avg/p95)={_fmt_num(w.get('step_cm_avg'))}/{_fmt_num(w.get('step_cm_p95'))}"
    )
    return lines


def _fmt_ball_text(ball_stats):
    if not ball_stats:
        return "Ball: waiting for data..."
    if ball_stats.get("error"):
        return f"Ball stats error: {ball_stats.get('error')}"
    latest = ball_stats.get("latest") or {}
    lines = ["Ball Quality Monitor"]
    lines.append(f"mode={ball_stats.get('mode', 'N/A')}")
    lines.append(
        f"latest: tracked={latest.get('tracked')} fallback={latest.get('fallback_used')} "
        f"count={latest.get('count', 0)} latency={_fmt_ms(latest.get('latency_ms'))}"
    )
    lines.append(
        f"latest center(cm)={latest.get('center_real') or 'N/A'} "
        f"mask={_fmt_pct(latest.get('mask_ratio'))} hough={latest.get('hough_count', 'N/A')} "
        f"radius(px)={_fmt_num(latest.get('radius_px'), 1)}"
    )
    lines.append(f"latest centers(cm)={_fmt_ball_centers(latest.get('centers_real') or [])}")
    lines.extend(_fmt_ball_window("SHORT", ball_stats.get("short") or {}))
    lines.extend(_fmt_ball_window("LONG", ball_stats.get("long") or {}))
    return "\n".join(lines)


def _clamp01(value):
    try:
        return max(0.0, min(1.0, float(value)))
    except Exception:
        return 0.0


def _mix_red_green(score):
    s = _clamp01(score)
    r0, g0, b0 = 220, 38, 38
    r1, g1, b1 = 22, 163, 74
    r = int(round(r0 + (r1 - r0) * s))
    g = int(round(g0 + (g1 - g0) * s))
    b = int(round(b0 + (b1 - b0) * s))
    return f"#{r:02x}{g:02x}{b:02x}"


def _ball_quality_scores(ball_stats):
    mode = str((ball_stats or {}).get("mode", "single")).lower()
    short = (ball_stats or {}).get("short") or {}
    frames = int(short.get("frames") or 0)
    if frames <= 0:
        return {
            "overall": 0.0,
            "tracking": 0.0,
            "stability": 0.0,
            "latency": 0.0,
            "count": 0.0,
        }
    expected_count = 1.0 if mode == "single" else 2.0

    tracked_ratio = short.get("tracked_ratio")
    fallback_ratio = short.get("fallback_ratio")
    jitter_rms = short.get("jitter_cm_rms")
    latency_avg = short.get("latency_avg_ms")
    count_avg = short.get("detected_count_avg")

    tracking_score = _clamp01((tracked_ratio if tracked_ratio is not None else 0.0) - 0.5 * (fallback_ratio or 0.0))
    stability_score = 0.0 if jitter_rms is None else _clamp01(1.0 - float(jitter_rms) / 18.0)
    latency_score = 0.0 if latency_avg is None else _clamp01(1.0 - float(latency_avg) / 70.0)
    if count_avg is None:
        count_score = 0.0
    else:
        count_score = _clamp01(1.0 - abs(float(count_avg) - expected_count) / max(expected_count, 1.0))

    overall = _clamp01(
        0.45 * tracking_score
        + 0.25 * stability_score
        + 0.20 * latency_score
        + 0.10 * count_score
    )
    return {
        "overall": overall,
        "tracking": tracking_score,
        "stability": stability_score,
        "latency": latency_score,
        "count": count_score,
    }


def _finite_xy(x, y):
    try:
        fx = float(x)
        fy = float(y)
    except Exception:
        return False
    return math.isfinite(fx) and math.isfinite(fy)


class PipelineDebugWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Challenge Style - Vision Pipeline Debug")
        self.root.geometry("1180x840")
        self.root.minsize(1080, 760)
        self.root.configure(bg="#f3f7fb")

        self.status_var = tk.StringVar(value="狀態: 未啟動")
        self.robot_vars = {}
        self.monitor_robot_vars = {}
        self.param_status_var = tk.StringVar(value="Aruco Params: waiting...")
        self.capture_status_var = tk.StringVar(value="Camera Params: waiting...")
        self.param_help_title_var = tk.StringVar(value="參數影響說明")
        self.param_help_up_var = tk.StringVar(value="↑ 拖動參數可查看影響")
        self.param_help_down_var = tk.StringVar(value="↓ 拖動參數可查看影響")
        self.param_help_tip_var = tk.StringVar(value="建議: 先小幅調整，觀察 IDs 與延遲變化。")
        self._last_ball_stats = {}
        self._robot_prev_final = {rid: None for rid in range(3)}
        self._cam_info_hold_sec = 1.2
        self._cam_info_cache = {
            "Cam1": {"seen_at": {}, "markers": {}, "corners": {}},
            "Cam2": {"seen_at": {}, "markers": {}, "corners": {}},
        }
        self._robot_info_hold_sec = 1.2
        self._robot_info_cache = {}
        self._robot_quality_latest = {}
        self._param_rt_robot_widgets = {}
        self._debug_metric_help_expanded = False
        self._debug_metric_help_toggle_var = tk.StringVar(value="展開說明")
        self._debug_metric_help_body = None
        self._debug_updates_paused = False
        self._debug_pause_btn_text = tk.StringVar(value="暫停Debug更新")
        self._ui_update_ms = 240
        self._text_cache = {}
        self._display_filter_cfg = {
            "alpha_xy": 0.35,
            "alpha_deg": 0.25,
            "alpha_vec": 0.30,
            "deadband_xy_cm": 0.9,
            "deadband_deg": 1.5,
            "deadband_vec": 0.02,
        }
        self._robot_display_filter = {}
        self._ui_alert_history = deque(maxlen=6)
        self._ui_alert_last_ts = {}
        self._ui_alert_flags = {}
        self.debug_alerts_var = tk.StringVar(value="事件提醒:\n-")
        self.cam_detail_enable_vars = {
            "Cam1": tk.BooleanVar(value=False),
            "Cam2": tk.BooleanVar(value=False),
        }
        self.cam_detail_id_vars = {
            "Cam1": tk.StringVar(value="auto"),
            "Cam2": tk.StringVar(value="auto"),
        }
        self._param_controls = {}
        self._param_apply_job = None
        self._param_silent = False
        self._aruco_param_canvas = None
        self._aruco_param_canvas_window = None
        self._capture_ui_silent = False
        self.capture_resolution_var = tk.StringVar(value="640x480")
        self.capture_fps_var = tk.IntVar(value=30)
        self.capture_auto_exposure_var = tk.BooleanVar(value=True)
        self.capture_exposure_var = tk.DoubleVar(value=-6.0)
        self.capture_auto_focus_var = tk.BooleanVar(value=True)
        self.capture_focus_var = tk.DoubleVar(value=0.0)
        self.capture_gain_var = tk.DoubleVar(value=0.0)
        self.capture_brightness_var = tk.DoubleVar(value=-1.0)
        self.capture_multiscale_var = tk.StringVar(value="1.6")
        self.bayes_status_var = tk.StringVar(value="Bayes: idle")
        self.bayes_category_var = tk.StringVar(value="")
        self.bayes_iters_var = tk.IntVar(value=12)
        self.bayes_eval_sec_var = tk.DoubleVar(value=1.5)
        self.bayes_candidates_var = tk.IntVar(value=180)
        self.bayes_expected_ids_var = tk.IntVar(value=3)
        self._bayes_category_defs = {}
        self._bayes_category_labels = []
        self._bayes_results = []
        self._bayes_results_dirty = False
        self._bayes_selected_idx = None
        self._bayes_selected_payload = None
        self._bayes_running = False
        self._bayes_status_pending = "Bayes: idle"
        self._bayes_stop_event = threading.Event()
        self._bayes_lock = threading.Lock()
        self._bayes_worker = None
        self._robot_id_groups = [[0], [10], [20]]
        if hasattr(imageprocess, "get_robot_tracking_options"):
            try:
                opts = imageprocess.get_robot_tracking_options() or {}
                groups = opts.get("robot_id_groups")
                if isinstance(groups, list) and len(groups) == 3:
                    self._robot_id_groups = [list(g) if isinstance(g, (list, tuple)) else [] for g in groups]
            except Exception:
                pass
        try:
            flat_ids = []
            for group in self._robot_id_groups:
                if isinstance(group, (list, tuple)):
                    flat_ids.extend(group)
            default_expected = len({int(x) for x in flat_ids})
            self.bayes_expected_ids_var.set(max(1, int(default_expected)))
        except Exception:
            self.bayes_expected_ids_var.set(3)

        top = tk.Frame(self.root, bg="#f3f7fb")
        top.pack(fill="x", padx=14, pady=(12, 8))
        self.status_label = tk.Label(
            top,
            textvariable=self.status_var,
            anchor="w",
            font=("Microsoft JhengHei UI", 12, "bold"),
            padx=10,
            pady=8,
            bg="#e5e7eb",
            fg="#1f2937",
        )
        self.status_label.pack(side="left", fill="x", expand=True)

        btn_group = tk.Frame(top, bg="#f3f7fb")
        btn_group.pack(side="right")
        tk.Button(
            btn_group,
            text="開始",
            width=8,
            command=self.start_detection,
            bg="#0f766e",
            fg="white",
            activebackground="#115e59",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            btn_group,
            text="停止",
            width=8,
            command=self.stop_detection,
            bg="#b45309",
            fg="white",
            activebackground="#92400e",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            btn_group,
            text="重啟",
            width=8,
            command=self.restart_detection,
            bg="#334155",
            fg="white",
            activebackground="#1e293b",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        self.debug_pause_btn = tk.Button(
            btn_group,
            textvariable=self._debug_pause_btn_text,
            width=12,
            command=self.toggle_debug_updates,
            bg="#7c3aed",
            fg="white",
            activebackground="#6d28d9",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        )
        self.debug_pause_btn.pack(side="left", padx=(0, 8))
        tk.Button(
            btn_group,
            text="Debug頁",
            width=8,
            command=self.show_debug_tab,
            bg="#2563eb",
            fg="white",
            activebackground="#1d4ed8",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            btn_group,
            text="Monitor頁",
            width=8,
            command=self.show_monitor_tab,
            bg="#1d4ed8",
            fg="white",
            activebackground="#1e40af",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            btn_group,
            text="Ball頁",
            width=8,
            command=self.show_ball_tab,
            bg="#0d9488",
            fg="white",
            activebackground="#0f766e",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            btn_group,
            text="Param頁",
            width=8,
            command=self.show_param_tab,
            bg="#0284c7",
            fg="white",
            activebackground="#0369a1",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            btn_group,
            text="關閉影像窗",
            width=10,
            command=self.switch_to_background,
            bg="#475569",
            fg="white",
            activebackground="#334155",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left")

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        self.debug_tab = tk.Frame(self.notebook, bg="#f3f7fb")
        self.monitor_tab = tk.Frame(self.notebook, bg="#f3f7fb")
        self.ball_tab = tk.Frame(self.notebook, bg="#f3f7fb")
        self.param_tab = tk.Frame(self.notebook, bg="#f3f7fb")
        self.notebook.add(self.debug_tab, text="Debug")
        self.notebook.add(self.monitor_tab, text="Monitor")
        self.notebook.add(self.ball_tab, text="Ball")
        self.notebook.add(self.param_tab, text="Param")

        center = tk.Frame(self.debug_tab, bg="#f3f7fb")
        center.pack(fill="both", expand=True, padx=14, pady=(0, 8))

        left_panel = tk.Frame(center, bg="#f3f7fb")
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 8))

        cam1_card = tk.Frame(left_panel, bg="white", bd=1, relief="solid")
        cam1_card.pack(fill="both", expand=True, pady=(0, 6))
        cam1_head = tk.Frame(cam1_card, bg="white")
        cam1_head.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(
            cam1_head,
            text="Cam1 Stage Monitor",
            anchor="w",
            bg="white",
            fg="#1e3a8a",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left")
        tk.Checkbutton(
            cam1_head,
            text="詳細",
            variable=self.cam_detail_enable_vars["Cam1"],
            bg="white",
            fg="#0f172a",
            selectcolor="white",
            font=("Microsoft JhengHei UI", 9),
        ).pack(side="right")
        tk.Label(
            cam1_head,
            text="ID",
            bg="white",
            fg="#334155",
            font=("Consolas", 9),
        ).pack(side="right", padx=(8, 4))
        tk.Entry(
            cam1_head,
            textvariable=self.cam_detail_id_vars["Cam1"],
            width=6,
            bg="#f8fafc",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="right")
        self.cam1_text = tk.Text(
            cam1_card,
            height=16,
            wrap="word",
            bg="#f8fafc",
            fg="#0f172a",
            font=("Consolas", 10),
            relief="flat",
        )
        cam1_scroll = tk.Scrollbar(cam1_card, orient="vertical", command=self.cam1_text.yview)
        self.cam1_text.configure(yscrollcommand=cam1_scroll.set)
        self.cam1_text.pack(side="left", fill="both", expand=True, padx=(10, 0), pady=(0, 10))
        cam1_scroll.pack(side="right", fill="y", padx=(6, 10), pady=(0, 10))

        cam2_card = tk.Frame(left_panel, bg="white", bd=1, relief="solid")
        cam2_card.pack(fill="both", expand=True, pady=(6, 0))
        cam2_head = tk.Frame(cam2_card, bg="white")
        cam2_head.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(
            cam2_head,
            text="Cam2 Stage Monitor",
            anchor="w",
            bg="white",
            fg="#1e3a8a",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left")
        tk.Checkbutton(
            cam2_head,
            text="詳細",
            variable=self.cam_detail_enable_vars["Cam2"],
            bg="white",
            fg="#0f172a",
            selectcolor="white",
            font=("Microsoft JhengHei UI", 9),
        ).pack(side="right")
        tk.Label(
            cam2_head,
            text="ID",
            bg="white",
            fg="#334155",
            font=("Consolas", 9),
        ).pack(side="right", padx=(8, 4))
        tk.Entry(
            cam2_head,
            textvariable=self.cam_detail_id_vars["Cam2"],
            width=6,
            bg="#f8fafc",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="right")
        self.cam2_text = tk.Text(
            cam2_card,
            height=16,
            wrap="word",
            bg="#f8fafc",
            fg="#0f172a",
            font=("Consolas", 10),
            relief="flat",
        )
        cam2_scroll = tk.Scrollbar(cam2_card, orient="vertical", command=self.cam2_text.yview)
        self.cam2_text.configure(yscrollcommand=cam2_scroll.set)
        self.cam2_text.pack(side="left", fill="both", expand=True, padx=(10, 0), pady=(0, 10))
        cam2_scroll.pack(side="right", fill="y", padx=(6, 10), pady=(0, 10))

        right_panel = tk.Frame(center, bg="#f3f7fb")
        right_panel.pack(side="right", fill="y")
        map_card = tk.Frame(right_panel, bg="white", bd=1, relief="solid")
        map_card.pack(fill="both", expand=True, pady=(0, 8))
        tk.Label(
            map_card,
            text="即時場地圖 (座標/方向)",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))
        self.map_canvas = tk.Canvas(map_card, bg="#f8fafc", highlightthickness=0, width=360, height=280)
        self.map_canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        alert_card = tk.Frame(right_panel, bg="white", bd=1, relief="solid")
        alert_card.pack(fill="x", pady=(0, 8))
        tk.Label(
            alert_card,
            text="事件提醒",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(
            alert_card,
            textvariable=self.debug_alerts_var,
            anchor="w",
            justify="left",
            bg="white",
            fg="#475569",
            font=("Consolas", 9),
            wraplength=340,
        ).pack(fill="x", padx=10, pady=(0, 8))

        tk.Label(
            right_panel,
            text="Robot 最終融合結果",
            anchor="w",
            bg="#f3f7fb",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", pady=(0, 4))

        for rid in range(3):
            card = tk.Frame(right_panel, bg="#ffffff", bd=1, relief="solid")
            card.pack(fill="x", pady=4)
            tk.Label(
                card,
                text=f"Robot{rid}",
                bg="#ffffff",
                fg="#1e3a8a",
                font=("Consolas", 10, "bold"),
            ).pack(anchor="w", padx=8, pady=(6, 2))
            chosen = tk.StringVar(value="Chosen: N/A")
            final = tk.StringVar(value="Final: X:- Y:- Ang:- Vec:(-, -)")
            left = tk.StringVar(value="Cam1: X:- Y:- Ang:- Vec:(-, -)")
            right = tk.StringVar(value="Cam2: X:- Y:- Ang:- Vec:(-, -)")
            self.robot_vars[rid] = {
                "chosen": chosen,
                "final": final,
                "left": left,
                "right": right,
            }
            tk.Label(card, textvariable=chosen, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8)
            tk.Label(card, textvariable=final, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8)
            tk.Label(card, textvariable=left, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8)
            tk.Label(card, textvariable=right, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8, pady=(0, 6))

        metric_help_card = tk.Frame(right_panel, bg="white", bd=1, relief="solid")
        metric_help_card.pack(fill="x", pady=(6, 0))
        metric_help_head = tk.Frame(metric_help_card, bg="white")
        metric_help_head.pack(fill="x", padx=8, pady=(8, 4))
        tk.Label(
            metric_help_head,
            text="Cam 指標說明",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left")
        tk.Button(
            metric_help_head,
            textvariable=self._debug_metric_help_toggle_var,
            command=self._toggle_debug_metric_help,
            bg="#475569",
            fg="white",
            activebackground="#334155",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
            padx=8,
            pady=2,
        ).pack(side="right")
        self._debug_metric_help_body = tk.Frame(metric_help_card, bg="white")
        tk.Label(
            self._debug_metric_help_body,
            text=(
                "去畸變/灰階: shape 是影像尺寸；mean/std 是亮度平均與對比波動。\n"
                "延遲 total/core/undist/gray: 單幀總耗時/核心流程/去畸變/灰階耗時。\n"
                "Stage avg/max: 各步驟平均與最大耗時；detect 常是主要瓶頸。\n"
                "detectMarkers: 每個前處理階段的偵測結果與耗時。\n"
                "IDs: 此幀偵測到的 ArUco ID 清單；corner(ID): 該 ID 四角像素座標。\n"
                "X/Y(主座標): 最終採用的場地座標；source 說明來源(MapTop/MapRaw/Ray)。\n"
                "OldX/OldY: 舊多項式映射結果；RayX/RayY: 射線投影結果。\n"
                "solvePnP rvec/tvec: 姿態解算旋轉/平移向量；dt 是該步驟耗時。\n"
                "Rodrigues R: rvec 轉旋轉矩陣結果；top_pose: 轉到機器人上表面參考。\n"
                "map(raw)/map(top)/ray: 三種座標換算路徑結果。\n"
                "angle(R_top): 朝向角；vec from R_top[:,1]: 朝向單位向量(vec_x/vec_y)。"
            ),
            anchor="w",
            justify="left",
            bg="white",
            fg="#475569",
            font=("Microsoft JhengHei UI", 9),
            wraplength=340,
        ).pack(fill="x", padx=8, pady=(0, 8))
        self._set_debug_metric_help_expanded(False)

        quality_frame = tk.Frame(self.debug_tab, bg="#f3f7fb")
        quality_frame.pack(fill="both", expand=False, padx=14, pady=(0, 12))
        tk.Label(
            quality_frame,
            text="Robot 品質燈條（雙目/一致性/穩定度）",
            anchor="w",
            bg="#f3f7fb",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x")
        self.robot_quality_canvas = tk.Canvas(
            quality_frame,
            height=150,
            bg="#f8fafc",
            highlightthickness=1,
            highlightbackground="#cbd5e1",
        )
        self.robot_quality_canvas.pack(fill="both", expand=True)

        # Monitor tab (independent page for field view + realtime robot status)
        mon_top = tk.Frame(self.monitor_tab, bg="#f3f7fb")
        mon_top.pack(fill="x", padx=14, pady=(10, 8))
        tk.Label(
            mon_top,
            text="Monitor 視圖（可與 Debug 頁切換）",
            anchor="w",
            bg="#f3f7fb",
            fg="#334155",
            font=("Microsoft JhengHei UI", 11, "bold"),
        ).pack(fill="x")

        mon_mid = tk.Frame(self.monitor_tab, bg="#f3f7fb")
        mon_mid.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        mon_map_card = tk.Frame(mon_mid, bg="white", bd=1, relief="solid")
        mon_map_card.pack(side="left", fill="both", expand=True, padx=(0, 8))
        tk.Label(
            mon_map_card,
            text="即時場地圖",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))
        self.monitor_map_canvas = tk.Canvas(mon_map_card, bg="#f8fafc", highlightthickness=0)
        self.monitor_map_canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        mon_robot_panel = tk.Frame(mon_mid, bg="#f3f7fb")
        mon_robot_panel.pack(side="right", fill="y")
        tk.Label(
            mon_robot_panel,
            text="即時座標與方向",
            anchor="w",
            bg="#f3f7fb",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", pady=(0, 4))
        for rid in range(3):
            card = tk.Frame(mon_robot_panel, bg="#ffffff", bd=1, relief="solid")
            card.pack(fill="x", pady=4)
            tk.Label(
                card,
                text=f"Robot{rid}",
                bg="#ffffff",
                fg="#1e3a8a",
                font=("Consolas", 10, "bold"),
            ).pack(anchor="w", padx=8, pady=(6, 2))
            mon_line1 = tk.StringVar(value="X:- Y:- Ang:-")
            mon_line2 = tk.StringVar(value="Vec:(-, -) Source:N/A")
            self.monitor_robot_vars[rid] = {
                "line1": mon_line1,
                "line2": mon_line2,
            }
            tk.Label(card, textvariable=mon_line1, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8)
            tk.Label(card, textvariable=mon_line2, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8, pady=(0, 6))

        # Ball tab (single-ball quality metrics)
        ball_top = tk.Frame(self.ball_tab, bg="#f3f7fb")
        ball_top.pack(fill="x", padx=14, pady=(10, 8))
        tk.Label(
            ball_top,
            text="Ball 偵測策略與品質監控（固定單球）",
            anchor="w",
            bg="#f3f7fb",
            fg="#334155",
            font=("Microsoft JhengHei UI", 11, "bold"),
        ).pack(fill="x")

        mode_card = tk.Frame(self.ball_tab, bg="white", bd=1, relief="solid")
        mode_card.pack(fill="x", padx=14, pady=(0, 8))
        mode_row = tk.Frame(mode_card, bg="white")
        mode_row.pack(fill="x", padx=10, pady=10)
        tk.Label(
            mode_row,
            text="球偵測模式",
            bg="white",
            fg="#0f172a",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left")
        self.ball_mode_var = tk.StringVar(value="single")
        self._ball_mode_syncing = False
        tk.Label(
            mode_row,
            text="單球（固定）",
            bg="white",
            fg="#0369a1",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(16, 6))

        ball_coord_card = tk.Frame(self.ball_tab, bg="white", bd=1, relief="solid")
        ball_coord_card.pack(fill="x", padx=14, pady=(0, 8))
        tk.Label(
            ball_coord_card,
            text="當前球座標 (cm)",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 2))
        self.ball_coord_var = tk.StringVar(value="X: -    Y: -")
        tk.Label(
            ball_coord_card,
            textvariable=self.ball_coord_var,
            anchor="w",
            bg="white",
            fg="#be123c",
            font=("Consolas", 24, "bold"),
        ).pack(fill="x", padx=10, pady=(0, 2))
        self.ball_centers_var = tk.StringVar(value="Ball: N/A")
        tk.Label(
            ball_coord_card,
            textvariable=self.ball_centers_var,
            anchor="w",
            bg="white",
            fg="#0f172a",
            font=("Consolas", 11),
        ).pack(fill="x", padx=10, pady=(0, 8))

        ball_mid = tk.Frame(self.ball_tab, bg="#f3f7fb")
        ball_mid.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        ball_map_card = tk.Frame(ball_mid, bg="white", bd=1, relief="solid")
        ball_map_card.pack(side="left", fill="both", expand=True, padx=(0, 8))
        tk.Label(
            ball_map_card,
            text="Ball 場地圖",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))
        self.ball_map_canvas = tk.Canvas(ball_map_card, bg="#f8fafc", highlightthickness=0)
        self.ball_map_canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        ball_stats_card = tk.Frame(ball_mid, bg="white", bd=1, relief="solid")
        ball_stats_card.pack(side="right", fill="both", expand=True)
        tk.Label(
            ball_stats_card,
            text="Ball 品質指標",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))
        self.ball_quality_summary_var = tk.StringVar(value="品質燈條: N/A")
        self.ball_quality_summary_label = tk.Label(
            ball_stats_card,
            textvariable=self.ball_quality_summary_var,
            anchor="w",
            bg="white",
            fg="#64748b",
            font=("Consolas", 11, "bold"),
        )
        self.ball_quality_summary_label.pack(fill="x", padx=10, pady=(0, 4))
        self.ball_quality_canvas = tk.Canvas(
            ball_stats_card,
            height=132,
            bg="#f8fafc",
            highlightthickness=1,
            highlightbackground="#e2e8f0",
        )
        self.ball_quality_canvas.pack(fill="x", padx=10, pady=(0, 8))
        self.ball_quality_canvas.bind("<Configure>", lambda _e: self._draw_ball_quality_bars(self._last_ball_stats))
        self.ball_text = tk.Text(
            ball_stats_card,
            wrap="word",
            bg="#f8fafc",
            fg="#0f172a",
            font=("Consolas", 10),
            relief="flat",
        )
        ball_scroll = tk.Scrollbar(ball_stats_card, orient="vertical", command=self.ball_text.yview)
        self.ball_text.configure(yscrollcommand=ball_scroll.set)
        self.ball_text.pack(side="left", fill="both", expand=True, padx=(10, 0), pady=(0, 10))
        ball_scroll.pack(side="right", fill="y", padx=(6, 10), pady=(0, 10))
        self._set_text(self.ball_text, "Ball: waiting for data...")
        self.refresh_ball_mode()
        self._build_param_tab()
        self._load_aruco_params_from_runtime()
        self._load_capture_params_from_runtime()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind_all("<MouseWheel>", self._on_aruco_param_mousewheel, add="+")
        self.root.bind_all("<Button-4>", self._on_aruco_param_mousewheel_up, add="+")
        self.root.bind_all("<Button-5>", self._on_aruco_param_mousewheel_down, add="+")
        self.root.after(200, self.start_detection)
        self.root.after(100, self.tick)

    def _build_param_tab(self):
        top = tk.Frame(self.param_tab, bg="#f3f7fb")
        top.pack(fill="x", padx=14, pady=(10, 8))
        tk.Label(
            top,
            text="Aruco Detector 參數調整（即時覆蓋）",
            anchor="w",
            bg="#f3f7fb",
            fg="#334155",
            font=("Microsoft JhengHei UI", 11, "bold"),
        ).pack(fill="x")

        action = tk.Frame(self.param_tab, bg="white", bd=1, relief="solid")
        action.pack(fill="x", padx=14, pady=(0, 8))
        row = tk.Frame(action, bg="white")
        row.pack(fill="x", padx=10, pady=10)
        tk.Button(
            row,
            text="立即套用",
            command=self.apply_aruco_params_now,
            bg="#0f766e",
            fg="white",
            activebackground="#115e59",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            row,
            text="儲存參數",
            command=self.save_aruco_params,
            bg="#0369a1",
            fg="white",
            activebackground="#075985",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            row,
            text="重載檔案",
            command=self.reload_aruco_params,
            bg="#475569",
            fg="white",
            activebackground="#334155",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            row,
            text="還原預設",
            command=self.reset_aruco_params,
            bg="#b45309",
            fg="white",
            activebackground="#92400e",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(side="left")
        rt_panel = tk.Frame(row, bg="#f8fafc", bd=1, relief="solid")
        rt_panel.pack(side="right", padx=(10, 0))
        tk.Label(
            rt_panel,
            text="當前機器人座標 / 品質燈",
            anchor="w",
            bg="#f8fafc",
            fg="#334155",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(fill="x", padx=8, pady=(6, 2))
        for rid in range(3):
            line = tk.Frame(rt_panel, bg="#f8fafc")
            line.pack(fill="x", padx=8, pady=(0, 2))
            lamp = tk.Label(
                line,
                text="●",
                bg="#f8fafc",
                fg="#94a3b8",
                font=("Consolas", 11, "bold"),
            )
            lamp.pack(side="left")
            info_var = tk.StringVar(value=f"R{rid} X:- Y:- Q:-")
            tk.Label(
                line,
                textvariable=info_var,
                anchor="w",
                bg="#f8fafc",
                fg="#0f172a",
                font=("Consolas", 9, "bold"),
                width=30,
            ).pack(side="left", padx=(6, 0))
            self._param_rt_robot_widgets[rid] = {
                "lamp": lamp,
                "info_var": info_var,
            }
        tk.Label(
            action,
            textvariable=self.param_status_var,
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Consolas", 10),
        ).pack(fill="x", padx=10, pady=(0, 8))

        capture_card = tk.Frame(self.param_tab, bg="white", bd=1, relief="solid")
        capture_card.pack(fill="x", padx=14, pady=(0, 8))
        tk.Label(
            capture_card,
            text="Camera / ArUco 小標記強化",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(
            capture_card,
            text=(
                "調整建議: 先用「解析度/FPS」定速度，再調「曝光/對焦」保清晰，最後微調 Gain/Brightness 與 Upscale。\n"
                "小標記優先策略: 降低曝光時間(關自動曝光後調 Exposure)、增加光源、關自動對焦並把 Focus 鎖在工作距離。\n"
                "按鈕說明: 「立即套用」只影響當前執行(不寫檔)；「儲存變更」才覆寫設定檔；「重載已儲存值」會回到上次儲存。"
            ),
            anchor="w",
            justify="left",
            bg="white",
            fg="#475569",
            font=("Microsoft JhengHei UI", 9),
            wraplength=980,
        ).pack(fill="x", padx=10, pady=(0, 6))
        capture_main = tk.Frame(capture_card, bg="white")
        capture_main.pack(fill="x", padx=10, pady=(0, 8))
        capture_left = tk.Frame(capture_main, bg="white")
        capture_left.pack(side="left", fill="both", expand=True, padx=(0, 10))

        capture_hint = tk.Frame(capture_main, bg="#f8fafc", bd=1, relief="solid", width=360)
        capture_hint.pack(side="right", fill="y")
        capture_hint.pack_propagate(False)
        tk.Label(
            capture_hint,
            text="右側速查: 條大/條小會怎樣",
            anchor="w",
            bg="#f8fafc",
            fg="#334155",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(
            capture_hint,
            text=(
                "解析度↑ 細節多但更慢；解析度↓ 較快但小標記易漏。\n"
                "FPS↑ 更新快但單幀穩定度可能降；FPS↓ 反應較慢。\n"
                "Exposure→0 較亮但拖影風險增；更負值快門快但變暗。\n"
                "Focus 調整焦點距離，請對工作距離實測最佳點。\n"
                "Gain↑ 變亮但噪點增；Gain↓ 較乾淨但偏暗。\n"
                "Brightness↑ 易過曝；Brightness↓ 較保細節但偏暗。\n"
                "Upscale↑ 小 ArUco 較易檢出但更慢；Upscale↓ 較快。"
            ),
            anchor="w",
            justify="left",
            bg="#f8fafc",
            fg="#475569",
            font=("Microsoft JhengHei UI", 9),
            wraplength=336,
        ).pack(fill="x", padx=10, pady=(0, 8))

        cap_row1 = tk.Frame(capture_left, bg="white")
        cap_row1.pack(fill="x", padx=0, pady=(0, 6))
        tk.Label(cap_row1, text="解析度", bg="white", fg="#0f172a", font=("Consolas", 10, "bold")).pack(side="left")
        res_values = [f"{w}x{h}" for w, h in CAPTURE_RESOLUTION_OPTIONS]
        self.capture_resolution_combo = ttk.Combobox(
            cap_row1,
            textvariable=self.capture_resolution_var,
            values=res_values,
            state="readonly",
            width=12,
        )
        self.capture_resolution_combo.pack(side="left", padx=(8, 16))
        self.capture_resolution_combo.bind("<<ComboboxSelected>>", self._on_capture_resolution_change)

        tk.Label(cap_row1, text="FPS", bg="white", fg="#0f172a", font=("Consolas", 10, "bold")).pack(side="left")
        tk.Spinbox(
            cap_row1,
            from_=1,
            to=120,
            textvariable=self.capture_fps_var,
            width=6,
            increment=1,
            bg="white",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="left", padx=(8, 12))
        tk.Button(
            cap_row1,
            text="立即套用 Camera+MultiScale",
            command=self.apply_capture_params_now,
            bg="#0f766e",
            fg="white",
            activebackground="#115e59",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            cap_row1,
            text="儲存變更",
            command=self.save_capture_params,
            bg="#0369a1",
            fg="white",
            activebackground="#075985",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            cap_row1,
            text="強多尺度預設",
            command=self.apply_strong_multiscale_preset,
            bg="#1d4ed8",
            fg="white",
            activebackground="#1e40af",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            cap_row1,
            text="重載已儲存值",
            command=self.reload_capture_params_from_saved,
            bg="#475569",
            fg="white",
            activebackground="#334155",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left")

        cap_row2 = tk.Frame(capture_left, bg="white")
        cap_row2.pack(fill="x", padx=0, pady=(0, 6))
        tk.Checkbutton(
            cap_row2,
            text="自動曝光",
            variable=self.capture_auto_exposure_var,
            bg="white",
            fg="#0f172a",
            selectcolor="white",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left")
        tk.Label(cap_row2, text="Exposure", bg="white", fg="#334155", font=("Consolas", 9)).pack(side="left", padx=(10, 6))
        tk.Scale(
            cap_row2,
            from_=-16.0,
            to=0.0,
            resolution=0.1,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=self.capture_exposure_var,
            length=170,
            bg="white",
            troughcolor="#dbeafe",
            highlightthickness=0,
        ).pack(side="left")
        tk.Label(cap_row2, textvariable=self.capture_exposure_var, bg="white", fg="#1e3a8a", font=("Consolas", 9, "bold"), width=6).pack(side="left", padx=(4, 8))

        tk.Checkbutton(
            cap_row2,
            text="自動對焦",
            variable=self.capture_auto_focus_var,
            bg="white",
            fg="#0f172a",
            selectcolor="white",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left", padx=(4, 0))
        tk.Label(cap_row2, text="Focus", bg="white", fg="#334155", font=("Consolas", 9)).pack(side="left", padx=(10, 6))
        tk.Scale(
            cap_row2,
            from_=0.0,
            to=255.0,
            resolution=1.0,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=self.capture_focus_var,
            length=150,
            bg="white",
            troughcolor="#dbeafe",
            highlightthickness=0,
        ).pack(side="left")
        tk.Label(cap_row2, textvariable=self.capture_focus_var, bg="white", fg="#1e3a8a", font=("Consolas", 9, "bold"), width=5).pack(side="left", padx=(4, 0))

        cap_row3 = tk.Frame(capture_left, bg="white")
        cap_row3.pack(fill="x", padx=0, pady=(0, 8))
        tk.Label(cap_row3, text="Gain", bg="white", fg="#334155", font=("Consolas", 9, "bold")).pack(side="left")
        tk.Scale(
            cap_row3,
            from_=0.0,
            to=255.0,
            resolution=1.0,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=self.capture_gain_var,
            length=180,
            bg="white",
            troughcolor="#dbeafe",
            highlightthickness=0,
        ).pack(side="left", padx=(6, 0))
        tk.Label(cap_row3, textvariable=self.capture_gain_var, bg="white", fg="#1e3a8a", font=("Consolas", 9, "bold"), width=5).pack(side="left", padx=(4, 10))

        tk.Label(cap_row3, text="Brightness", bg="white", fg="#334155", font=("Consolas", 9, "bold")).pack(side="left")
        tk.Scale(
            cap_row3,
            from_=-1.0,
            to=255.0,
            resolution=1.0,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=self.capture_brightness_var,
            length=180,
            bg="white",
            troughcolor="#dbeafe",
            highlightthickness=0,
        ).pack(side="left", padx=(6, 0))
        tk.Label(cap_row3, textvariable=self.capture_brightness_var, bg="white", fg="#1e3a8a", font=("Consolas", 9, "bold"), width=5).pack(side="left", padx=(4, 12))

        tk.Label(cap_row3, text="Upscale", bg="white", fg="#334155", font=("Consolas", 9, "bold")).pack(side="left")
        tk.Entry(
            cap_row3,
            textvariable=self.capture_multiscale_var,
            width=24,
            bg="#f8fafc",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="left", padx=(6, 0))

        tk.Label(
            capture_left,
            textvariable=self.capture_status_var,
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Consolas", 10),
        ).pack(fill="x", padx=0, pady=(0, 0))

        bayes_card = tk.Frame(self.param_tab, bg="white", bd=1, relief="solid")
        bayes_card.pack(fill="x", padx=14, pady=(0, 8))
        tk.Label(
            bayes_card,
            text="Bayes 調參（分門別類，僅調單一類別）",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(8, 4))

        self._bayes_category_defs = self._build_bayes_categories()
        self._bayes_category_labels = [cfg.get("label", k) for k, cfg in self._bayes_category_defs.items()]
        if self._bayes_category_labels:
            self.bayes_category_var.set(self._bayes_category_labels[0])

        bayes_top = tk.Frame(bayes_card, bg="white")
        bayes_top.pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(bayes_top, text="類別", bg="white", fg="#0f172a", font=("Consolas", 10, "bold")).pack(side="left")
        self.bayes_category_combo = ttk.Combobox(
            bayes_top,
            textvariable=self.bayes_category_var,
            values=self._bayes_category_labels,
            state="readonly",
            width=26,
        )
        self.bayes_category_combo.pack(side="left", padx=(8, 12))

        tk.Label(bayes_top, text="迭代", bg="white", fg="#0f172a", font=("Consolas", 9)).pack(side="left")
        tk.Spinbox(
            bayes_top,
            from_=3,
            to=200,
            textvariable=self.bayes_iters_var,
            width=5,
            increment=1,
            bg="white",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="left", padx=(4, 10))
        tk.Label(bayes_top, text="評估秒", bg="white", fg="#0f172a", font=("Consolas", 9)).pack(side="left")
        tk.Spinbox(
            bayes_top,
            from_=0.5,
            to=10.0,
            textvariable=self.bayes_eval_sec_var,
            width=5,
            increment=0.1,
            format="%.1f",
            bg="white",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="left", padx=(4, 10))
        tk.Label(bayes_top, text="候選池", bg="white", fg="#0f172a", font=("Consolas", 9)).pack(side="left")
        tk.Spinbox(
            bayes_top,
            from_=30,
            to=1000,
            textvariable=self.bayes_candidates_var,
            width=6,
            increment=10,
            bg="white",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="left", padx=(4, 0))
        tk.Label(bayes_top, text="目標ID", bg="white", fg="#0f172a", font=("Consolas", 9)).pack(side="left", padx=(10, 0))
        tk.Spinbox(
            bayes_top,
            from_=0,
            to=200,
            textvariable=self.bayes_expected_ids_var,
            width=5,
            increment=1,
            bg="white",
            fg="#0f172a",
            relief="solid",
            borderwidth=1,
        ).pack(side="left", padx=(4, 0))

        bayes_btn = tk.Frame(bayes_card, bg="white")
        bayes_btn.pack(fill="x", padx=10, pady=(0, 6))
        tk.Button(
            bayes_btn,
            text="開始 Bayes",
            command=self.start_bayes_tuning,
            bg="#0f766e",
            fg="white",
            activebackground="#115e59",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            bayes_btn,
            text="停止",
            command=self.stop_bayes_tuning,
            bg="#b45309",
            fg="white",
            activebackground="#92400e",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            bayes_btn,
            text="套用選擇列",
            command=self.apply_selected_bayes_result,
            bg="#1d4ed8",
            fg="white",
            activebackground="#1e40af",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left", padx=(0, 8))
        tk.Button(
            bayes_btn,
            text="覆寫最後選擇",
            command=self.persist_selected_bayes_result,
            bg="#7c3aed",
            fg="white",
            activebackground="#6d28d9",
            activeforeground="white",
            relief="flat",
            font=("Microsoft JhengHei UI", 9, "bold"),
        ).pack(side="left")

        tk.Label(
            bayes_card,
            textvariable=self.bayes_status_var,
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Consolas", 10),
        ).pack(fill="x", padx=10, pady=(0, 4))

        tree_wrap = tk.Frame(bayes_card, bg="white")
        tree_wrap.pack(fill="x", padx=10, pady=(0, 8))
        cols = ("rank", "score", "category", "params")
        self.bayes_tree = ttk.Treeview(tree_wrap, columns=cols, show="headings", height=8)
        self.bayes_tree.heading("rank", text="#")
        self.bayes_tree.heading("score", text="Score")
        self.bayes_tree.heading("category", text="Category")
        self.bayes_tree.heading("params", text="Params")
        self.bayes_tree.column("rank", width=40, anchor="center")
        self.bayes_tree.column("score", width=90, anchor="e")
        self.bayes_tree.column("category", width=180, anchor="w")
        self.bayes_tree.column("params", width=760, anchor="w")
        bayes_scroll = tk.Scrollbar(tree_wrap, orient="vertical", command=self.bayes_tree.yview)
        self.bayes_tree.configure(yscrollcommand=bayes_scroll.set)
        self.bayes_tree.pack(side="left", fill="x", expand=True)
        bayes_scroll.pack(side="right", fill="y")
        self.bayes_tree.bind("<<TreeviewSelect>>", self._on_bayes_tree_select)

        body = tk.Frame(self.param_tab, bg="#f3f7fb")
        body.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        left = tk.Frame(body, bg="white", bd=1, relief="solid")
        left.pack(side="left", fill="both", expand=True, padx=(0, 8))
        tk.Label(
            left,
            text="拖動拉桿後，約 120ms 自動套用到執行中的 detector",
            anchor="w",
            bg="white",
            fg="#475569",
            font=("Microsoft JhengHei UI", 9),
        ).pack(fill="x", padx=10, pady=(8, 4))

        grid_wrap = tk.Frame(left, bg="white")
        grid_wrap.pack(fill="both", expand=True, padx=10, pady=(2, 10))
        self._aruco_param_canvas = tk.Canvas(
            grid_wrap,
            bg="white",
            highlightthickness=0,
            relief="flat",
            bd=0,
        )
        grid_scroll = tk.Scrollbar(grid_wrap, orient="vertical", command=self._aruco_param_canvas.yview)
        self._aruco_param_canvas.configure(yscrollcommand=grid_scroll.set)
        self._aruco_param_canvas.pack(side="left", fill="both", expand=True)
        grid_scroll.pack(side="right", fill="y")

        grid = tk.Frame(self._aruco_param_canvas, bg="white")
        self._aruco_param_canvas_window = self._aruco_param_canvas.create_window((0, 0), window=grid, anchor="nw")

        def _sync_aruco_scroll_region(_event=None):
            canvas = self._aruco_param_canvas
            if canvas is None:
                return
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _sync_aruco_scroll_width(event):
            canvas = self._aruco_param_canvas
            if canvas is None or self._aruco_param_canvas_window is None:
                return
            canvas.itemconfigure(self._aruco_param_canvas_window, width=event.width)

        grid.bind("<Configure>", _sync_aruco_scroll_region)
        self._aruco_param_canvas.bind("<Configure>", _sync_aruco_scroll_width)
        grid.columnconfigure(1, weight=1)
        for idx, spec in enumerate(ARUCO_PARAM_SPECS):
            key = spec["key"]
            value_var = tk.StringVar(value="0")
            slider_var = tk.IntVar(value=0)
            tk.Label(
                grid,
                text=spec["label"],
                anchor="w",
                bg="white",
                fg="#0f172a",
                font=("Consolas", 9, "bold"),
            ).grid(row=idx, column=0, sticky="w", padx=(0, 10), pady=4)
            scale = tk.Scale(
                grid,
                from_=int(spec["slider_min"]),
                to=int(spec["slider_max"]),
                orient=tk.HORIZONTAL,
                resolution=1,
                showvalue=False,
                variable=slider_var,
                command=lambda _v, k=key: self._on_aruco_slider_change(k),
                bg="white",
                troughcolor="#dbeafe",
                highlightthickness=0,
            )
            scale.grid(row=idx, column=1, sticky="ew", pady=2)
            scale.bind("<Button-1>", lambda _e, k=key: self._update_param_help(k))
            scale.bind("<KeyRelease>", lambda _e, k=key: self._update_param_help(k))
            tk.Label(
                grid,
                textvariable=value_var,
                width=10,
                anchor="e",
                bg="white",
                fg="#1e3a8a",
                font=("Consolas", 10, "bold"),
            ).grid(row=idx, column=2, sticky="e", padx=(8, 0))
            self._param_controls[key] = {
                "spec": spec,
                "slider_var": slider_var,
                "value_var": value_var,
            }
        right = tk.Frame(body, bg="white", bd=1, relief="solid", width=360)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)
        tk.Label(
            right,
            text="調整影響註解",
            anchor="w",
            bg="white",
            fg="#334155",
            font=("Microsoft JhengHei UI", 10, "bold"),
        ).pack(fill="x", padx=10, pady=(10, 6))
        tk.Label(
            right,
            textvariable=self.param_help_title_var,
            anchor="w",
            justify="left",
            bg="white",
            fg="#0f172a",
            font=("Consolas", 10, "bold"),
            wraplength=330,
        ).pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(
            right,
            textvariable=self.param_help_up_var,
            anchor="w",
            justify="left",
            bg="white",
            fg="#047857",
            font=("Microsoft JhengHei UI", 9),
            wraplength=330,
        ).pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(
            right,
            textvariable=self.param_help_down_var,
            anchor="w",
            justify="left",
            bg="white",
            fg="#b45309",
            font=("Microsoft JhengHei UI", 9),
            wraplength=330,
        ).pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(
            right,
            textvariable=self.param_help_tip_var,
            anchor="w",
            justify="left",
            bg="white",
            fg="#475569",
            font=("Microsoft JhengHei UI", 9),
            wraplength=330,
        ).pack(fill="x", padx=10, pady=(0, 10))
        if ARUCO_PARAM_SPECS:
            self._update_param_help(ARUCO_PARAM_SPECS[0]["key"])

    def _pointer_inside_widget(self, widget):
        if widget is None:
            return False
        try:
            if not bool(widget.winfo_exists()):
                return False
            px = int(self.root.winfo_pointerx())
            py = int(self.root.winfo_pointery())
            x0 = int(widget.winfo_rootx())
            y0 = int(widget.winfo_rooty())
            w = int(widget.winfo_width())
            h = int(widget.winfo_height())
        except Exception:
            return False
        return x0 <= px < (x0 + max(1, w)) and y0 <= py < (y0 + max(1, h))

    def _scroll_aruco_param_canvas(self, steps):
        canvas = self._aruco_param_canvas
        if canvas is None or steps == 0:
            return None
        if not self._pointer_inside_widget(canvas):
            return None
        try:
            canvas.yview_scroll(int(steps), "units")
            return "break"
        except Exception:
            return None

    def _on_aruco_param_mousewheel(self, event):
        delta = int(getattr(event, "delta", 0))
        if delta == 0:
            return None
        if abs(delta) >= 120:
            steps = -int(delta / 120)
        else:
            steps = -1 if delta > 0 else 1
        return self._scroll_aruco_param_canvas(steps)

    def _on_aruco_param_mousewheel_up(self, _event):
        return self._scroll_aruco_param_canvas(-1)

    def _on_aruco_param_mousewheel_down(self, _event):
        return self._scroll_aruco_param_canvas(1)

    def _build_bayes_categories(self):
        spec_map = {str(s.get("key")): s for s in ARUCO_PARAM_SPECS if isinstance(s, dict) and s.get("key")}

        def _aruco_vars(keys):
            out = []
            for key in keys:
                spec = spec_map.get(key)
                if not spec:
                    continue
                kind = str(spec.get("kind", "float")).lower()
                if kind == "bool":
                    out.append({"name": key, "type": "bool"})
                    continue
                factor = float(spec.get("factor", 1) or 1)
                lo = float(spec.get("slider_min", 0)) / factor
                hi = float(spec.get("slider_max", 1)) / factor
                if kind == "int":
                    out.append({"name": key, "type": "int", "min": int(round(lo)), "max": int(round(hi)), "step": 1})
                else:
                    out.append({"name": key, "type": "float", "min": float(lo), "max": float(hi), "step": float(1.0 / factor)})
            return out

        defs = {}
        aruco_groups = [
            ("aruco_refine", "ArUco/角點細化", ["cornerRefinementMethod", "cornerRefinementWinSize", "cornerRefinementMaxIterations", "cornerRefinementMinAccuracy"]),
            ("aruco_threshold", "ArUco/二值化閾值", ["adaptiveThreshWinSizeMin", "adaptiveThreshWinSizeMax", "adaptiveThreshWinSizeStep", "adaptiveThreshConstant"]),
            ("aruco_geometry", "ArUco/幾何過濾", ["minMarkerPerimeterRate", "maxMarkerPerimeterRate", "minCornerDistanceRate", "minMarkerDistanceRate", "minDistanceToBorder"]),
            ("aruco_robust", "ArUco/容錯與對比", ["minOtsuStdDev", "perspectiveRemoveIgnoredMarginPerCell", "errorCorrectionRate", "detectInvertedMarker"]),
        ]
        for cid, label, keys in aruco_groups:
            vars_def = _aruco_vars(keys)
            if vars_def:
                defs[cid] = {"id": cid, "label": label, "target": "aruco", "vars": vars_def}

        res_options = [f"{w}x{h}" for w, h in CAPTURE_RESOLUTION_OPTIONS]
        defs["camera_resolution"] = {
            "id": "camera_resolution",
            "label": "Camera/解析度與FPS",
            "target": "camera",
            "vars": [
                {"name": "resolution", "type": "choice", "options": res_options},
                {"name": "fps", "type": "int", "min": 5, "max": 60, "step": 1},
            ],
        }
        defs["camera_exposure"] = {
            "id": "camera_exposure",
            "label": "Camera/曝光與亮度",
            "target": "camera",
            "vars": [
                {"name": "auto_exposure", "type": "bool"},
                {"name": "exposure", "type": "float", "min": -16.0, "max": 0.0, "step": 0.1},
                {"name": "gain", "type": "float", "min": 0.0, "max": 80.0, "step": 1.0},
                {"name": "brightness", "type": "float", "min": -1.0, "max": 80.0, "step": 1.0},
            ],
        }
        defs["camera_focus"] = {
            "id": "camera_focus",
            "label": "Camera/對焦",
            "target": "camera",
            "vars": [
                {"name": "auto_focus", "type": "bool"},
                {"name": "focus", "type": "float", "min": 0.0, "max": 255.0, "step": 1.0},
            ],
        }
        defs["aruco_multiscale"] = {
            "id": "aruco_multiscale",
            "label": "ArUco/多尺度",
            "target": "upscale",
            "vars": [
                {"name": "scale_min", "type": "float", "min": 1.1, "max": 2.4, "step": 0.05},
                {"name": "scale_step", "type": "float", "min": 0.15, "max": 0.9, "step": 0.05},
                {"name": "scale_count", "type": "int", "min": 1, "max": 4, "step": 1},
            ],
        }
        return defs

    def _bayes_category_by_label(self, label):
        wanted = str(label or "").strip()
        for cid, cfg in self._bayes_category_defs.items():
            if str(cfg.get("label")) == wanted:
                return cid, cfg
        for cid, cfg in self._bayes_category_defs.items():
            return cid, cfg
        return None, None

    def _set_bayes_status(self, text, immediate=False):
        msg = str(text)
        with self._bayes_lock:
            self._bayes_status_pending = msg
        if immediate:
            self.bayes_status_var.set(msg)

    def _poll_bayes_updates(self):
        with self._bayes_lock:
            status = str(self._bayes_status_pending)
            dirty = bool(self._bayes_results_dirty)
        if self.bayes_status_var.get() != status:
            self.bayes_status_var.set(status)
        if dirty:
            self._refresh_bayes_tree()

    def _quantize(self, value, step, lo=None, hi=None):
        try:
            v = float(value)
            s = float(step)
        except Exception:
            return value
        if s > 0:
            v = round(v / s) * s
        if lo is not None:
            v = max(float(lo), v)
        if hi is not None:
            v = min(float(hi), v)
        return v

    def _sample_random_candidate(self, cat_cfg):
        out = {}
        for spec in cat_cfg.get("vars", []):
            name = spec.get("name")
            typ = spec.get("type")
            if typ == "bool":
                out[name] = bool(random.getrandbits(1))
            elif typ == "choice":
                options = list(spec.get("options") or [])
                out[name] = random.choice(options) if options else ""
            elif typ == "int":
                lo = int(spec.get("min", 0))
                hi = int(spec.get("max", lo + 1))
                step = int(spec.get("step", 1))
                if step <= 0:
                    step = 1
                count = max(1, ((hi - lo) // step) + 1)
                out[name] = int(lo + random.randrange(count) * step)
            else:
                lo = float(spec.get("min", 0.0))
                hi = float(spec.get("max", lo + 1.0))
                step = float(spec.get("step", 0.01))
                raw = random.uniform(lo, hi)
                out[name] = float(self._quantize(raw, step, lo, hi))
        return out

    def _candidate_key(self, candidate):
        normalized = {}
        for k, v in dict(candidate or {}).items():
            if isinstance(v, float):
                normalized[k] = round(float(v), 6)
            else:
                normalized[k] = v
        return json.dumps(normalized, sort_keys=True, ensure_ascii=False)

    def _encode_candidate(self, cat_cfg, candidate):
        vec = []
        for spec in cat_cfg.get("vars", []):
            name = spec.get("name")
            typ = spec.get("type")
            val = candidate.get(name)
            if typ == "bool":
                vec.append(1.0 if bool(val) else 0.0)
            elif typ == "choice":
                opts = list(spec.get("options") or [])
                if not opts:
                    vec.append(0.0)
                else:
                    try:
                        idx = opts.index(val)
                    except Exception:
                        idx = 0
                    denom = max(1, len(opts) - 1)
                    vec.append(float(idx) / float(denom))
            else:
                lo = float(spec.get("min", 0.0))
                hi = float(spec.get("max", lo + 1.0))
                span = max(1e-9, hi - lo)
                try:
                    fv = float(val)
                except Exception:
                    fv = lo
                vec.append(max(0.0, min(1.0, (fv - lo) / span)))
        return np.asarray(vec, dtype=np.float64)

    def _surrogate_predict(self, x_hist, y_hist, x):
        if not x_hist or not y_hist:
            return 0.0, 1.0
        X = np.asarray(x_hist, dtype=np.float64)
        y = np.asarray(y_hist, dtype=np.float64)
        xx = np.asarray(x, dtype=np.float64)
        dist2 = np.sum((X - xx) ** 2, axis=1)
        sigma = 0.22
        w = np.exp(-dist2 / max(1e-9, 2.0 * sigma * sigma)) + 1e-6
        mu = float(np.sum(w * y) / np.sum(w))
        local_var = float(np.sum(w * (y - mu) ** 2) / np.sum(w))
        global_var = float(np.var(y)) if len(y) >= 2 else local_var
        std = math.sqrt(max(1e-6, 0.7 * local_var + 0.3 * global_var))
        return mu, std

    def _suggest_bayes_candidate(self, cat_cfg, history, pool_size):
        if len(history) < max(4, len(cat_cfg.get("vars", [])) + 1):
            return self._sample_random_candidate(cat_cfg)

        x_hist = [item["x"] for item in history]
        y_hist = [item["score"] for item in history]
        seen = {item["key"] for item in history}
        best = None
        best_acq = -1e18
        pool_n = max(30, int(pool_size))
        for _ in range(pool_n):
            cand = self._sample_random_candidate(cat_cfg)
            key = self._candidate_key(cand)
            if key in seen:
                continue
            xx = self._encode_candidate(cat_cfg, cand)
            mu, std = self._surrogate_predict(x_hist, y_hist, xx)
            acq = mu + 1.25 * std
            if acq > best_acq:
                best_acq = acq
                best = cand
        if best is None:
            return self._sample_random_candidate(cat_cfg)
        return best

    def _candidate_to_payload(self, cat_cfg, candidate):
        cid = str(cat_cfg.get("id"))
        payload = {"aruco": None, "camera": None, "upscale": None}
        if cid.startswith("aruco_") and cid != "aruco_multiscale":
            payload["aruco"] = dict(candidate)
            return payload
        if cid == "camera_resolution":
            res_txt = str(candidate.get("resolution", "640x480"))
            w, h = 640, 480
            if "x" in res_txt:
                try:
                    sw, sh = res_txt.split("x", 1)
                    w = int(sw.strip())
                    h = int(sh.strip())
                except Exception:
                    pass
            payload["camera"] = {
                "width": int(max(320, min(3840, w))),
                "height": int(max(240, min(2160, h))),
                "fps": int(max(1, min(120, int(candidate.get("fps", 30))))),
            }
            return payload
        if cid == "camera_exposure":
            payload["camera"] = {
                "auto_exposure": bool(candidate.get("auto_exposure", True)),
                "exposure": float(candidate.get("exposure", -6.0)),
                "gain": float(candidate.get("gain", 0.0)),
                "brightness": float(candidate.get("brightness", -1.0)),
            }
            return payload
        if cid == "camera_focus":
            payload["camera"] = {
                "auto_focus": bool(candidate.get("auto_focus", True)),
                "focus": float(candidate.get("focus", 0.0)),
            }
            return payload
        if cid == "aruco_multiscale":
            s_min = float(candidate.get("scale_min", 1.3))
            s_step = float(candidate.get("scale_step", 0.4))
            s_count = int(candidate.get("scale_count", 3))
            factors = []
            for i in range(max(1, s_count)):
                factors.append(max(1.05, min(4.0, s_min + i * s_step)))
            factors = [round(float(v), 2) for v in factors]
            payload["upscale"] = {"upscale_factors": factors}
            return payload
        return payload

    def _payload_to_text(self, payload):
        chunks = []
        aruco = payload.get("aruco") if isinstance(payload, dict) else None
        camera = payload.get("camera") if isinstance(payload, dict) else None
        upscale = payload.get("upscale") if isinstance(payload, dict) else None
        if isinstance(aruco, dict):
            items = []
            for k, v in aruco.items():
                if isinstance(v, float):
                    items.append(f"{k}={v:.4f}".rstrip("0").rstrip("."))
                else:
                    items.append(f"{k}={v}")
            chunks.append("Aruco(" + ", ".join(items) + ")")
        if isinstance(camera, dict):
            items = []
            for k, v in camera.items():
                if isinstance(v, float):
                    items.append(f"{k}={v:.3f}".rstrip("0").rstrip("."))
                else:
                    items.append(f"{k}={v}")
            chunks.append("Camera(" + ", ".join(items) + ")")
        if isinstance(upscale, dict):
            chunks.append(f"Upscale({upscale.get('upscale_factors')})")
        return " | ".join(chunks) if chunks else "N/A"

    def _apply_payload_merge(self, payload, persist=False):
        if not isinstance(payload, dict):
            return
        aruco = payload.get("aruco")
        camera = payload.get("camera")
        upscale = payload.get("upscale")

        if isinstance(aruco, dict) and aruco and hasattr(imageprocess, "set_aruco_detector_params"):
            try:
                imageprocess.set_aruco_detector_params(aruco, persist=bool(persist), replace=False)
            except Exception:
                try:
                    imageprocess.set_aruco_detector_params(aruco, persist=False, replace=False)
                    if persist and hasattr(imageprocess, "save_aruco_detector_params"):
                        imageprocess.save_aruco_detector_params()
                except Exception:
                    pass

        if isinstance(camera, dict) and camera and hasattr(imageprocess, "set_camera_capture_config"):
            try:
                imageprocess.set_camera_capture_config(camera, apply_now=True, replace=False, persist=bool(persist))
            except TypeError:
                imageprocess.set_camera_capture_config(camera, apply_now=True, replace=False)
                if persist and hasattr(imageprocess, "save_runtime_capture_params"):
                    imageprocess.save_runtime_capture_params()
            except Exception:
                pass

        if isinstance(upscale, dict) and upscale and hasattr(imageprocess, "set_aruco_upscale_options"):
            try:
                imageprocess.set_aruco_upscale_options(upscale, replace=False, persist=bool(persist))
            except TypeError:
                imageprocess.set_aruco_upscale_options(upscale, replace=False)
                if persist and hasattr(imageprocess, "save_runtime_capture_params"):
                    imageprocess.save_runtime_capture_params()
            except Exception:
                pass

    def _compute_live_metrics(self, state, expected_ids_target=0):
        cams = state.get("cams", {}) if isinstance(state, dict) else {}
        c1 = cams.get("Cam1", {}) or {}
        c2 = cams.get("Cam2", {}) or {}
        ids_cam1 = list(c1.get("detected_ids") or [])
        ids_cam2 = list(c2.get("detected_ids") or [])
        ids_total = len(ids_cam1) + len(ids_cam2)
        unique_ids_seen = 0.0
        try:
            unique_ids_seen = float(len({int(v) for v in (ids_cam1 + ids_cam2)}))
        except Exception:
            unique_ids_seen = float(len({str(v) for v in (ids_cam1 + ids_cam2)}))
        markers_total = len(c1.get("markers") or {}) + len(c2.get("markers") or {})
        t1 = ((c1.get("timings") or {}).get("pipeline_total_ms")) or 0.0
        t2 = ((c2.get("timings") or {}).get("pipeline_total_ms")) or 0.0
        latency_total = float(t1) + float(t2)

        def _dup_stats(ids):
            counts = {}
            for raw_id in ids:
                try:
                    key = int(raw_id)
                except Exception:
                    key = str(raw_id)
                counts[key] = counts.get(key, 0) + 1
            dup_excess = sum(max(0, int(v) - 1) for v in counts.values())
            dup_groups = sum(1 for v in counts.values() if int(v) > 1)
            return float(dup_excess), float(dup_groups)

        dup_excess_1, dup_groups_1 = _dup_stats(ids_cam1)
        dup_excess_2, dup_groups_2 = _dup_stats(ids_cam2)
        duplicate_id_excess = float(dup_excess_1 + dup_excess_2)
        duplicate_id_groups = float(dup_groups_1 + dup_groups_2)
        duplicate_penalty = 12.0 * duplicate_id_excess
        try:
            id_target = max(0.0, float(expected_ids_target))
        except Exception:
            id_target = 0.0
        if id_target > 0.0:
            id_target_gap = abs(float(unique_ids_seen) - id_target)
            id_target_fit = _clamp01(1.0 - id_target_gap / max(id_target, 1.0))
            id_target_bonus = 14.0 * id_target_fit
        else:
            id_target_gap = 0.0
            id_target_fit = 0.0
            id_target_bonus = 0.0

        robots = state.get("robots") or {}
        robots_seen = 0
        for _, r in robots.items():
            final = (r or {}).get("final") or {}
            if _finite_xy(final.get("x"), final.get("y")):
                robots_seen += 1
        score = (
            8.0 * float(ids_total)
            + 3.2 * float(markers_total)
            + 10.0 * float(robots_seen)
            - 0.08 * float(latency_total)
            - float(duplicate_penalty)
            + float(id_target_bonus)
        )
        return {
            "ids_total": float(ids_total),
            "unique_ids_seen": float(unique_ids_seen),
            "markers_total": float(markers_total),
            "robots_seen": float(robots_seen),
            "latency_total_ms": float(latency_total),
            "duplicate_id_excess": float(duplicate_id_excess),
            "duplicate_id_groups": float(duplicate_id_groups),
            "duplicate_penalty": float(duplicate_penalty),
            "id_target": float(id_target),
            "id_target_gap": float(id_target_gap),
            "id_target_fit": float(id_target_fit),
            "id_target_bonus": float(id_target_bonus),
            "score": float(score),
        }

    def _evaluate_live_window(self, seconds, expected_ids_target=0):
        dur = max(0.3, float(seconds))
        end_t = time.time() + dur
        samples = []
        while time.time() < end_t:
            if self._bayes_stop_event.is_set():
                break
            samples.append(self._compute_live_metrics(_deep_state_copy(), expected_ids_target=expected_ids_target))
            time.sleep(0.12)
        if not samples:
            samples.append(self._compute_live_metrics(_deep_state_copy(), expected_ids_target=expected_ids_target))
        keys = [
            "ids_total",
            "unique_ids_seen",
            "markers_total",
            "robots_seen",
            "latency_total_ms",
            "duplicate_id_excess",
            "duplicate_id_groups",
            "duplicate_penalty",
            "id_target",
            "id_target_gap",
            "id_target_fit",
            "id_target_bonus",
            "score",
        ]
        avg = {}
        for k in keys:
            avg[k] = float(sum(float(s.get(k, 0.0)) for s in samples) / max(1, len(samples)))
        return avg

    def _refresh_bayes_tree(self):
        with self._bayes_lock:
            ranked = sorted(self._bayes_results, key=lambda x: float(x.get("score", -1e18)), reverse=True)
            self._bayes_results = ranked
            self._bayes_results_dirty = False
        self.bayes_tree.delete(*self.bayes_tree.get_children())
        for idx, row in enumerate(ranked, start=1):
            vals = (
                idx,
                f"{float(row.get('score', 0.0)):.3f}",
                str(row.get("category_label", "")),
                self._payload_to_text(row.get("payload") or {}),
            )
            self.bayes_tree.insert("", "end", iid=str(idx - 1), values=vals)

    def _on_bayes_tree_select(self, _event=None):
        sel = self.bayes_tree.selection()
        if not sel:
            return
        try:
            idx = int(str(sel[0]))
        except Exception:
            return
        with self._bayes_lock:
            if idx < 0 or idx >= len(self._bayes_results):
                return
            row = self._bayes_results[idx]
        self._bayes_selected_idx = idx
        self._bayes_selected_payload = row.get("payload")
        payload = self._bayes_selected_payload
        if isinstance(payload, dict):
            self._apply_payload_merge(payload, persist=False)
            self._load_aruco_params_from_runtime()
            self._load_capture_params_from_runtime()
            self._set_bayes_status(
                f"Bayes: selected rank#{idx + 1}, score={float(row.get('score', 0.0)):.3f} (applied non-persist)",
                immediate=True,
            )
            _add_event(f"bayes auto apply rank#{idx + 1}")
        else:
            self._set_bayes_status(f"Bayes: selected rank#{idx + 1}, score={float(row.get('score', 0.0)):.3f}", immediate=True)

    def start_bayes_tuning(self):
        if self._bayes_running:
            self._set_bayes_status("Bayes: running (請先停止)", immediate=True)
            return
        cid, cat = self._bayes_category_by_label(self.bayes_category_var.get())
        if not cid or not cat:
            self._set_bayes_status("Bayes: category unavailable", immediate=True)
            return
        iters = int(max(3, min(300, int(self.bayes_iters_var.get()))))
        eval_sec = float(max(0.3, min(12.0, float(self.bayes_eval_sec_var.get()))))
        pool = int(max(30, min(2000, int(self.bayes_candidates_var.get()))))
        target_ids = int(max(0, min(200, int(self.bayes_expected_ids_var.get()))))

        with self._bayes_lock:
            self._bayes_results = []
            self._bayes_results_dirty = True
        self._bayes_selected_idx = None
        self._bayes_selected_payload = None
        self._bayes_stop_event.clear()
        self._bayes_running = True
        self._set_bayes_status(
            f"Bayes: starting {cat.get('label')} iters={iters}, eval={eval_sec:.1f}s, targetID={target_ids}",
            immediate=True,
        )
        _add_event(
            f"bayes start category={cat.get('label')} iters={iters} eval={eval_sec:.1f}s pool={pool} targetID={target_ids}"
        )

        self._bayes_worker = threading.Thread(
            target=self._bayes_worker_loop,
            args=(cid, cat, iters, eval_sec, pool, target_ids),
            daemon=True,
        )
        self._bayes_worker.start()

    def stop_bayes_tuning(self):
        if not self._bayes_running:
            return
        self._bayes_stop_event.set()
        self._set_bayes_status("Bayes: stopping...", immediate=True)
        _add_event("bayes stop requested")

    def _bayes_worker_loop(self, cid, cat, iters, eval_sec, pool, target_ids):
        history = []
        started = time.time()
        worker_error = None
        try:
            for step in range(iters):
                if self._bayes_stop_event.is_set():
                    break
                cand = self._suggest_bayes_candidate(cat, history, pool)
                payload = self._candidate_to_payload(cat, cand)
                self._apply_payload_merge(payload, persist=False)
                metrics = self._evaluate_live_window(eval_sec, expected_ids_target=target_ids)
                score = float(metrics.get("score", -1e18))
                key = self._candidate_key(cand)
                x = self._encode_candidate(cat, cand)
                row = {
                    "iter": int(step + 1),
                    "score": score,
                    "candidate": cand,
                    "payload": payload,
                    "metrics": metrics,
                    "key": key,
                    "x": x,
                    "category_id": cid,
                    "category_label": cat.get("label", cid),
                    "elapsed_sec": float(time.time() - started),
                }
                history.append({"score": score, "key": key, "x": x})
                with self._bayes_lock:
                    self._bayes_results.append(row)
                    self._bayes_results_dirty = True
                self._set_bayes_status(
                    f"Bayes: {cat.get('label')} iter {step + 1}/{iters} score={score:.3f} "
                    f"ids={metrics.get('ids_total', 0):.1f} markers={metrics.get('markers_total', 0):.1f} "
                    f"dup={metrics.get('duplicate_id_excess', 0):.1f} "
                    f"uniq={metrics.get('unique_ids_seen', 0):.1f}/{metrics.get('id_target', 0):.0f} "
                    f"fit={100.0 * float(metrics.get('id_target_fit', 0.0)):.0f}%"
                )
        except Exception as err:
            worker_error = err
            self._set_bayes_status(f"Bayes: worker error ({err})")
            _add_event(f"bayes worker error: {err}")
        finally:
            self._bayes_running = False
            if self._bayes_stop_event.is_set():
                self._set_bayes_status("Bayes: stopped")
                _add_event("bayes stopped")
            elif worker_error is not None:
                self._set_bayes_status(f"Bayes: failed ({worker_error})")
                _add_event("bayes failed")
            else:
                self._set_bayes_status("Bayes: completed (可點選結果套用)")
                _add_event("bayes completed")

    def apply_selected_bayes_result(self):
        payload = self._bayes_selected_payload
        if payload is None:
            self._set_bayes_status("Bayes: 請先在結果表選一列", immediate=True)
            return
        self._apply_payload_merge(payload, persist=False)
        self._load_aruco_params_from_runtime()
        self._load_capture_params_from_runtime()
        self._set_bayes_status("Bayes: selected result applied (non-persist)", immediate=True)
        _add_event("bayes apply selected result")

    def persist_selected_bayes_result(self):
        payload = self._bayes_selected_payload
        if payload is None:
            self._set_bayes_status("Bayes: 請先在結果表選一列", immediate=True)
            return
        self._apply_payload_merge(payload, persist=True)
        self._load_aruco_params_from_runtime()
        self._load_capture_params_from_runtime()
        self._set_bayes_status("Bayes: selected result persisted", immediate=True)
        _add_event("bayes persist selected result")

    def _slider_to_param_value(self, spec, slider_value):
        raw = int(round(float(slider_value)))
        factor = float(spec.get("factor", 1))
        value = raw / factor
        kind = spec.get("kind")
        if kind == "bool":
            return bool(raw)
        if kind == "int":
            return int(round(value))
        return float(value)

    def _param_value_to_slider(self, spec, value):
        factor = float(spec.get("factor", 1))
        kind = spec.get("kind")
        if kind == "bool":
            raw = 1 if bool(value) else 0
        else:
            raw = int(round(float(value) * factor))
        return max(int(spec["slider_min"]), min(int(spec["slider_max"]), raw))

    def _fmt_param_value(self, spec, value):
        kind = spec.get("kind")
        if kind == "bool":
            return "True" if bool(value) else "False"
        if kind == "int":
            return str(int(value))
        return f"{float(value):.6f}".rstrip("0").rstrip(".")

    def _collect_aruco_params_from_ui(self):
        params = {}
        for key, control in self._param_controls.items():
            spec = control["spec"]
            raw_value = control["slider_var"].get()
            params[key] = self._slider_to_param_value(spec, raw_value)
        return params

    def _set_aruco_ui_values(self, params):
        self._param_silent = True
        try:
            for key, control in self._param_controls.items():
                if key not in params:
                    continue
                spec = control["spec"]
                slider = self._param_value_to_slider(spec, params.get(key))
                control["slider_var"].set(slider)
                actual = self._slider_to_param_value(spec, slider)
                control["value_var"].set(self._fmt_param_value(spec, actual))
        finally:
            self._param_silent = False

    def _on_aruco_slider_change(self, key):
        control = self._param_controls.get(key)
        if control is None:
            return
        spec = control["spec"]
        raw_value = control["slider_var"].get()
        actual = self._slider_to_param_value(spec, raw_value)
        control["value_var"].set(self._fmt_param_value(spec, actual))
        self._update_param_help(key)
        if self._param_silent:
            return
        if self._param_apply_job is not None:
            self.root.after_cancel(self._param_apply_job)
        self._param_apply_job = self.root.after(ARUCO_AUTO_APPLY_MS, self.apply_aruco_params_now)

    def _set_param_status(self, text):
        self.param_status_var.set(text)

    def _set_capture_status(self, text):
        self.capture_status_var.set(text)

    def _on_capture_resolution_change(self, _event=None):
        if self._capture_ui_silent:
            return
        text = str(self.capture_resolution_var.get() or "").strip().lower()
        if "x" not in text:
            return
        parts = text.split("x", 1)
        try:
            w = int(parts[0].strip())
            h = int(parts[1].strip())
        except Exception:
            return
        self.capture_resolution_var.set(f"{w}x{h}")

    def _parse_upscale_factors(self, text):
        tokens = str(text or "").replace(";", ",").replace("|", ",").split(",")
        factors = []
        for tok in tokens:
            tok = tok.strip()
            if not tok:
                continue
            try:
                v = float(tok)
            except Exception:
                continue
            if v <= 1.0:
                continue
            factors.append(v)
        if not factors:
            factors = [1.6]
        uniq = []
        for v in sorted(factors):
            if not uniq or abs(v - uniq[-1]) >= 0.05:
                uniq.append(round(float(v), 2))
        return uniq

    def _collect_capture_params_from_ui(self):
        res_txt = str(self.capture_resolution_var.get() or "640x480").strip().lower()
        w, h = 640, 480
        if "x" in res_txt:
            try:
                sw, sh = res_txt.split("x", 1)
                w = int(sw.strip())
                h = int(sh.strip())
            except Exception:
                pass
        cam_cfg = {
            "width": int(max(320, min(3840, w))),
            "height": int(max(240, min(2160, h))),
            "fps": int(max(1, min(120, int(self.capture_fps_var.get())))),
            "auto_exposure": bool(self.capture_auto_exposure_var.get()),
            "exposure": float(self.capture_exposure_var.get()),
            "auto_focus": bool(self.capture_auto_focus_var.get()),
            "focus": float(self.capture_focus_var.get()),
            "gain": float(self.capture_gain_var.get()),
            "brightness": float(self.capture_brightness_var.get()),
            "use_mjpg": True,
        }
        upscale_factors = self._parse_upscale_factors(self.capture_multiscale_var.get())
        return cam_cfg, upscale_factors

    def _set_capture_ui_values(self, cam_cfg, upscale_factors):
        self._capture_ui_silent = True
        try:
            if isinstance(cam_cfg, dict):
                w = int(cam_cfg.get("width", 640))
                h = int(cam_cfg.get("height", 480))
                self.capture_resolution_var.set(f"{w}x{h}")
                self.capture_fps_var.set(int(cam_cfg.get("fps", 30)))
                self.capture_auto_exposure_var.set(bool(cam_cfg.get("auto_exposure", True)))
                self.capture_exposure_var.set(float(cam_cfg.get("exposure", -6.0)))
                self.capture_auto_focus_var.set(bool(cam_cfg.get("auto_focus", True)))
                self.capture_focus_var.set(float(cam_cfg.get("focus", 0.0)))
                self.capture_gain_var.set(float(cam_cfg.get("gain", 0.0)))
                self.capture_brightness_var.set(float(cam_cfg.get("brightness", -1.0)))
            if isinstance(upscale_factors, (list, tuple)) and upscale_factors:
                txt = ",".join(f"{float(v):.2f}".rstrip("0").rstrip(".") for v in upscale_factors)
                self.capture_multiscale_var.set(txt)
        finally:
            self._capture_ui_silent = False

    def _load_capture_params_from_runtime(self):
        cam_cfg = {}
        up_cfg = {}
        try:
            if hasattr(imageprocess, "get_camera_capture_config"):
                cam_cfg = imageprocess.get_camera_capture_config() or {}
            if hasattr(imageprocess, "get_aruco_upscale_options"):
                up_cfg = imageprocess.get_aruco_upscale_options() or {}
            self._set_capture_ui_values(cam_cfg, up_cfg.get("upscale_factors"))
            self._set_capture_status(
                f"Camera={cam_cfg.get('width', 'N/A')}x{cam_cfg.get('height', 'N/A')}@{cam_cfg.get('fps', 'N/A')}  "
                f"Upscale={up_cfg.get('upscale_factors', [])}"
            )
        except Exception as err:
            self._set_capture_status(f"load capture params failed: {err}")

    def reload_capture_params_from_saved(self):
        try:
            if not hasattr(imageprocess, "load_runtime_capture_params"):
                self._load_capture_params_from_runtime()
                return

            state = imageprocess.load_runtime_capture_params(apply_now=True) or {}
            cam_cfg = state.get("camera_capture") if isinstance(state, dict) else {}
            up_cfg = state.get("aruco_upscale") if isinstance(state, dict) else {}
            if not isinstance(cam_cfg, dict):
                cam_cfg = {}
            if not isinstance(up_cfg, dict):
                up_cfg = {}

            # Fallback to current runtime getters when return payload is partial.
            if not cam_cfg and hasattr(imageprocess, "get_camera_capture_config"):
                cam_cfg = imageprocess.get_camera_capture_config() or {}
            if "upscale_factors" not in up_cfg and hasattr(imageprocess, "get_aruco_upscale_options"):
                up_cfg = imageprocess.get_aruco_upscale_options() or {}

            up_factors = up_cfg.get("upscale_factors", [])
            self._set_capture_ui_values(cam_cfg, up_factors)
            params_file = state.get("params_file", "runtime_capture_params.json") if isinstance(state, dict) else "runtime_capture_params.json"
            self._set_capture_status(
                f"reloaded saved {cam_cfg.get('width', 'N/A')}x{cam_cfg.get('height', 'N/A')}@{cam_cfg.get('fps', 'N/A')}  "
                f"Upscale={up_factors}  file={params_file}"
            )
            _add_event("reload capture params from saved file")
        except Exception as err:
            self._set_capture_status(f"reload saved capture params failed: {err}")

    def apply_capture_params_now(self, notify=True, persist=False):
        cam_cfg, upscale_factors = self._collect_capture_params_from_ui()
        cam_state = {}
        up_state = {}
        persist_flag = bool(persist)
        try:
            if hasattr(imageprocess, "set_camera_capture_config"):
                try:
                    cam_state = imageprocess.set_camera_capture_config(
                        cam_cfg, apply_now=True, replace=False, persist=persist_flag
                    ) or {}
                except TypeError:
                    cam_state = imageprocess.set_camera_capture_config(
                        cam_cfg, apply_now=True, replace=False
                    ) or {}
                    if persist_flag and hasattr(imageprocess, "save_runtime_capture_params"):
                        imageprocess.save_runtime_capture_params()
            else:
                self._set_capture_status("set_camera_capture_config unavailable.")
                return

            if hasattr(imageprocess, "set_aruco_upscale_options"):
                try:
                    up_state = imageprocess.set_aruco_upscale_options(
                        {"upscale_factors": upscale_factors}, replace=False, persist=persist_flag
                    ) or {}
                except TypeError:
                    up_state = imageprocess.set_aruco_upscale_options(
                        {"upscale_factors": upscale_factors}, replace=False
                    ) or {}
                    if persist_flag and hasattr(imageprocess, "save_runtime_capture_params"):
                        imageprocess.save_runtime_capture_params()
            applied_cfg = cam_state.get("config") if isinstance(cam_state, dict) else cam_cfg
            if not isinstance(applied_cfg, dict):
                applied_cfg = cam_cfg
            applied_up = up_state.get("upscale_factors") if isinstance(up_state, dict) else upscale_factors
            if not isinstance(applied_up, (list, tuple)) or not applied_up:
                applied_up = upscale_factors
            self._set_capture_ui_values(applied_cfg, applied_up)
            mode = "persisted" if persist_flag else "applied (live non-persist)"
            self._set_capture_status(
                f"{mode} {applied_cfg.get('width')}x{applied_cfg.get('height')}@{applied_cfg.get('fps')}  "
                f"AE={int(bool(applied_cfg.get('auto_exposure')))} AF={int(bool(applied_cfg.get('auto_focus')))}  "
                f"Upscale={applied_up}"
            )
            if notify:
                _add_event(
                    f"{mode} capture cfg {applied_cfg.get('width')}x{applied_cfg.get('height')}@{applied_cfg.get('fps')} "
                    f"AE={int(bool(applied_cfg.get('auto_exposure')))} AF={int(bool(applied_cfg.get('auto_focus')))} "
                    f"up={applied_up}"
                )
        except Exception as err:
            self._set_capture_status(f"apply capture params failed: {err}")

    def save_capture_params(self):
        self.apply_capture_params_now(notify=True, persist=True)

    def apply_strong_multiscale_preset(self):
        preset = STRONG_CAPTURE_PRESET or {}
        w = int(preset.get("width", 1280))
        h = int(preset.get("height", 720))
        self.capture_resolution_var.set(f"{w}x{h}")
        self.capture_fps_var.set(int(preset.get("fps", 30)))
        self.capture_auto_exposure_var.set(bool(preset.get("auto_exposure", False)))
        self.capture_exposure_var.set(float(preset.get("exposure", -7.0)))
        self.capture_auto_focus_var.set(bool(preset.get("auto_focus", False)))
        self.capture_focus_var.set(float(preset.get("focus", 120.0)))
        self.capture_gain_var.set(float(preset.get("gain", 8.0)))
        self.capture_brightness_var.set(float(preset.get("brightness", -1.0)))
        factors = preset.get("upscale_factors", [1.4, 1.8, 2.2, 2.6])
        txt = ",".join(f"{float(v):.2f}".rstrip("0").rstrip(".") for v in factors)
        self.capture_multiscale_var.set(txt)
        self.apply_capture_params_now(notify=True, persist=False)

    def _update_param_help(self, key):
        spec = next((x for x in ARUCO_PARAM_SPECS if x.get("key") == key), None)
        if spec is None:
            return
        impact = ARUCO_PARAM_IMPACTS.get(key, {})
        self.param_help_title_var.set(spec.get("label", key))
        self.param_help_up_var.set(f"↑ 增加: {impact.get('up', '影響依場景而定，請觀察檢出數與延遲。')}")
        self.param_help_down_var.set(f"↓ 減少: {impact.get('down', '影響依場景而定，請觀察檢出數與延遲。')}")
        self.param_help_tip_var.set(f"建議: {impact.get('tip', '每次只調 1 個參數並做 A/B 比對。')}")

    def _load_aruco_params_from_runtime(self):
        if not hasattr(imageprocess, "get_aruco_detector_state"):
            self._set_param_status("Aruco Params API not available.")
            return
        try:
            state = imageprocess.get_aruco_detector_state() or {}
            params = state.get("params") or {}
            rev = state.get("revision")
            params_file = state.get("params_file", "N/A")
            self._set_aruco_ui_values(params)
            self._set_param_status(f"rev={rev}  file={params_file}")
        except Exception as err:
            self._set_param_status(f"load state failed: {err}")

    def apply_aruco_params_now(self):
        self._param_apply_job = None
        if not hasattr(imageprocess, "set_aruco_detector_params"):
            self._set_param_status("set_aruco_detector_params unavailable.")
            return
        params = self._collect_aruco_params_from_ui()
        try:
            state = imageprocess.set_aruco_detector_params(params, persist=False, replace=True) or {}
            rev = state.get("revision")
            applied_params = state.get("params") or {}
            if applied_params:
                self._set_aruco_ui_values(applied_params)
            self._set_param_status(f"applied rev={rev} (live hot-swap)")
            _add_event(f"apply aruco params rev={rev}")
        except Exception as err:
            self._set_param_status(f"apply failed: {err}")

    def save_aruco_params(self):
        params = self._collect_aruco_params_from_ui()
        try:
            if hasattr(imageprocess, "set_aruco_detector_params"):
                imageprocess.set_aruco_detector_params(params, persist=False, replace=True)
            path = imageprocess.save_aruco_detector_params() if hasattr(imageprocess, "save_aruco_detector_params") else "N/A"
            self._set_param_status(f"saved: {path}")
            _add_event(f"save aruco params -> {path}")
        except Exception as err:
            self._set_param_status(f"save failed: {err}")

    def reload_aruco_params(self):
        try:
            if hasattr(imageprocess, "load_aruco_detector_params"):
                state = imageprocess.load_aruco_detector_params() or {}
                params = state.get("params") or {}
                self._set_aruco_ui_values(params)
                self._set_param_status(f"reloaded rev={state.get('revision')}")
                _add_event("reload aruco params from file")
            else:
                self._load_aruco_params_from_runtime()
        except Exception as err:
            self._set_param_status(f"reload failed: {err}")

    def reset_aruco_params(self):
        try:
            if hasattr(imageprocess, "reset_aruco_detector_params"):
                state = imageprocess.reset_aruco_detector_params(persist=False) or {}
                params = state.get("params") or {}
                self._set_aruco_ui_values(params)
                self._set_param_status(f"reset default rev={state.get('revision')}")
                _add_event("reset aruco params to default")
            else:
                self._set_param_status("reset_aruco_detector_params unavailable.")
        except Exception as err:
            self._set_param_status(f"reset failed: {err}")

    def _marker_id_int(self, value):
        try:
            return int(value)
        except Exception:
            return None

    def _cam_data_with_hold_cache(self, cam_name, cam_data):
        if not isinstance(cam_data, dict):
            return cam_data
        cache = self._cam_info_cache.get(cam_name)
        if not isinstance(cache, dict):
            cache = {"seen_at": {}, "markers": {}, "corners": {}}
            self._cam_info_cache[cam_name] = cache
        seen_at = cache.setdefault("seen_at", {})
        marker_cache = cache.setdefault("markers", {})
        corner_cache = cache.setdefault("corners", {})

        now_t = time.time()
        hold_sec = max(0.0, float(self._cam_info_hold_sec))
        cur_ids = []
        for raw in list(cam_data.get("detected_ids") or []):
            mid = self._marker_id_int(raw)
            if mid is not None:
                cur_ids.append(mid)

        cur_markers_raw = cam_data.get("markers") or {}
        cur_markers = {}
        if isinstance(cur_markers_raw, dict):
            for raw_mid, marker in cur_markers_raw.items():
                mid = self._marker_id_int(raw_mid)
                if mid is None:
                    continue
                cur_markers[mid] = copy.deepcopy(marker)

        cur_corners_raw = cam_data.get("detected_corner_view") or {}
        cur_corners = {}
        if isinstance(cur_corners_raw, dict):
            for raw_mid, pts in cur_corners_raw.items():
                mid = self._marker_id_int(raw_mid)
                if mid is None:
                    continue
                cur_corners[mid] = copy.deepcopy(pts)

        current_seen = set(cur_ids) | set(cur_markers.keys()) | set(cur_corners.keys())
        for mid in current_seen:
            seen_at[mid] = now_t
            if mid in cur_markers:
                marker_cache[mid] = cur_markers[mid]
            if mid in cur_corners:
                corner_cache[mid] = cur_corners[mid]

        for mid, ts in list(seen_at.items()):
            age = now_t - float(ts)
            if age > hold_sec:
                seen_at.pop(mid, None)
                marker_cache.pop(mid, None)
                corner_cache.pop(mid, None)

        merged_ids = list(cur_ids)
        merged_markers = dict(cur_markers)
        merged_corners = dict(cur_corners)
        for mid, ts in sorted(seen_at.items()):
            age = now_t - float(ts)
            if age > hold_sec:
                continue
            if mid not in merged_ids:
                merged_ids.append(mid)
            if mid not in merged_markers and mid in marker_cache:
                stale_marker = copy.deepcopy(marker_cache[mid])
                stale_marker["__stale_age_sec"] = float(age)
                merged_markers[mid] = stale_marker
            if mid not in merged_corners and mid in corner_cache:
                merged_corners[mid] = copy.deepcopy(corner_cache[mid])

        out = copy.deepcopy(cam_data)
        out["detected_ids"] = merged_ids
        out["markers"] = merged_markers
        out["detected_corner_view"] = merged_corners
        return out

    def _robot_data_with_hold_cache(self, rid, data):
        now_t = time.time()
        hold_sec = max(0.0, float(self._robot_info_hold_sec))
        if isinstance(data, dict) and data:
            self._robot_info_cache[rid] = {
                "seen_at": now_t,
                "data": copy.deepcopy(data),
            }
            return data, False

        slot = self._robot_info_cache.get(rid) or {}
        ts = float(slot.get("seen_at", 0.0) or 0.0)
        cached = slot.get("data")
        if isinstance(cached, dict) and (now_t - ts) <= hold_sec:
            return copy.deepcopy(cached), True
        return None, False

    def _smooth_scalar(self, prev, curr, alpha, deadband):
        if prev is None:
            return float(curr)
        p = float(prev)
        c = float(curr)
        if abs(c - p) < float(deadband):
            return p
        return p + float(alpha) * (c - p)

    def _smooth_angle(self, prev, curr, alpha, deadband):
        if prev is None:
            return float(curr)
        p = float(prev)
        c = float(curr)
        diff = ((c - p + 180.0) % 360.0) - 180.0
        if abs(diff) < float(deadband):
            return p
        out = p + float(alpha) * diff
        out = out % 360.0
        if out < 0:
            out += 360.0
        return out

    def _smooth_robot_data(self, rid, data):
        if not isinstance(data, dict):
            return data
        cfg = self._display_filter_cfg
        slot = self._robot_display_filter.setdefault(rid, {})
        out = copy.deepcopy(data)
        for branch in ("final", "left", "right"):
            cur = out.get(branch) or {}
            prev = slot.get(branch) or {}
            if not isinstance(cur, dict):
                continue
            if _finite_xy(cur.get("x"), cur.get("y")):
                cur["x"] = self._smooth_scalar(prev.get("x"), cur.get("x"), cfg["alpha_xy"], cfg["deadband_xy_cm"])
                cur["y"] = self._smooth_scalar(prev.get("y"), cur.get("y"), cfg["alpha_xy"], cfg["deadband_xy_cm"])
            if cur.get("deg") is not None:
                try:
                    cur["deg"] = self._smooth_angle(prev.get("deg"), cur.get("deg"), cfg["alpha_deg"], cfg["deadband_deg"])
                except Exception:
                    pass
            for vk in ("vec_x", "vec_y"):
                if cur.get(vk) is None:
                    continue
                try:
                    cur[vk] = self._smooth_scalar(prev.get(vk), cur.get(vk), cfg["alpha_vec"], cfg["deadband_vec"])
                except Exception:
                    pass
            slot[branch] = {
                "x": cur.get("x"),
                "y": cur.get("y"),
                "deg": cur.get("deg"),
                "vec_x": cur.get("vec_x"),
                "vec_y": cur.get("vec_y"),
            }
            out[branch] = cur
        return out

    def _cam_detail_opts(self, cam_name):
        v_on = self.cam_detail_enable_vars.get(cam_name)
        enabled = bool(v_on.get()) if v_on is not None else False
        v_id = self.cam_detail_id_vars.get(cam_name)
        rid = str((v_id.get() if v_id is not None else "auto") or "auto").strip()
        if not rid:
            rid = "auto"
        return enabled, rid

    def _push_ui_alert(self, key, message, cooldown_sec=1.5):
        now_t = time.time()
        last = float(self._ui_alert_last_ts.get(key, 0.0) or 0.0)
        if (now_t - last) < float(cooldown_sec):
            return
        self._ui_alert_last_ts[key] = now_t
        stamp = time.strftime("%H:%M:%S", time.localtime(now_t))
        self._ui_alert_history.appendleft(f"{stamp} | {message}")

    def _refresh_alert_panel(self):
        if not self._ui_alert_history:
            self.debug_alerts_var.set("事件提醒:\n-")
            return
        lines = ["事件提醒:"]
        lines.extend(list(self._ui_alert_history)[:4])
        self.debug_alerts_var.set("\n".join(lines))

    def _update_ui_alerts(self, state):
        cams = state.get("cams", {}) if isinstance(state, dict) else {}
        for cam_name in ("Cam1", "Cam2"):
            c = cams.get(cam_name) or {}
            ids = list(c.get("detected_ids") or [])
            has_ids = bool(ids)
            key_seen = f"{cam_name}_has_ids"
            prev_has = self._ui_alert_flags.get(key_seen)
            if prev_has is None:
                self._ui_alert_flags[key_seen] = has_ids
            elif bool(prev_has) != has_ids:
                if has_ids:
                    self._push_ui_alert(f"{cam_name}_recover", f"{cam_name} 恢復偵測 IDs={ids}", cooldown_sec=0.8)
                else:
                    self._push_ui_alert(f"{cam_name}_lost", f"{cam_name} ID 全部丟失", cooldown_sec=0.8)
                self._ui_alert_flags[key_seen] = has_ids

            lat = ((c.get("timings") or {}).get("pipeline_total_ms")) or 0.0
            high = float(lat) >= 45.0
            key_lat = f"{cam_name}_lat_high"
            prev_high = self._ui_alert_flags.get(key_lat)
            if prev_high is None:
                self._ui_alert_flags[key_lat] = high
            elif bool(prev_high) != high:
                if high:
                    self._push_ui_alert(f"{cam_name}_lat_enter", f"{cam_name} 延遲升高 total={_fmt_ms(lat)}", cooldown_sec=1.2)
                else:
                    self._push_ui_alert(f"{cam_name}_lat_leave", f"{cam_name} 延遲恢復 total={_fmt_ms(lat)}", cooldown_sec=1.2)
                self._ui_alert_flags[key_lat] = high

        for rid in range(3):
            q = self._robot_quality_latest.get(rid, {}) if isinstance(self._robot_quality_latest, dict) else {}
            overall = q.get("overall")
            if not isinstance(overall, (int, float)):
                continue
            low = float(overall) < 0.35
            key_q = f"robot{rid}_q_low"
            prev_low = self._ui_alert_flags.get(key_q)
            if prev_low is None:
                self._ui_alert_flags[key_q] = low
            elif bool(prev_low) != low:
                if low:
                    self._push_ui_alert(f"robot{rid}_q_drop", f"Robot{rid} 品質下降 {float(overall) * 100.0:.1f}%", cooldown_sec=1.5)
                else:
                    self._push_ui_alert(f"robot{rid}_q_recover", f"Robot{rid} 品質回升 {float(overall) * 100.0:.1f}%", cooldown_sec=1.5)
                self._ui_alert_flags[key_q] = low
        self._refresh_alert_panel()

    def _set_text(self, widget, text):
        key = str(widget)
        prev = self._text_cache.get(key)
        if prev == text:
            return
        self._text_cache[key] = text
        # Preserve user scroll position while refreshing live text content.
        try:
            y_first, _ = widget.yview()
        except Exception:
            y_first = 0.0
        try:
            x_first, _ = widget.xview()
        except Exception:
            x_first = 0.0
        widget.configure(state="normal")
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, text)
        widget.configure(state="disabled")
        try:
            widget.yview_moveto(y_first)
            widget.xview_moveto(x_first)
        except Exception:
            pass

    def _update_status_style(self, running, show_windows):
        if running:
            self.status_label.configure(bg="#d9f7e8", fg="#0b6b3a")
            if show_windows:
                self.status_var.set("狀態: 執行中 (含影像視窗)")
            else:
                self.status_var.set("狀態: 執行中 (背景模式，僅保留Debug視窗)")
        else:
            self.status_label.configure(bg="#fde8e8", fg="#7a1e1e")
            self.status_var.set("狀態: 已停止")

    def _draw_field_map(self, robots, canvas=None):
        canvas = canvas or self.map_canvas
        if canvas is None:
            return
        try:
            canvas.delete("all")
            w = max(int(canvas.winfo_width()), 320)
            h = max(int(canvas.winfo_height()), 220)
            margin = 16.0
            field_w = float(getattr(imageprocess, "REAL_WIDTH", 360.0))
            field_h = float(getattr(imageprocess, "REAL_HEIGHT", 260.0))
            scale = min((w - 2.0 * margin) / max(field_w, 1.0), (h - 2.0 * margin) / max(field_h, 1.0))
            offset_x = (w - field_w * scale) / 2.0
            offset_y = (h - field_h * scale) / 2.0

            def map_xy(x, y):
                return offset_x + float(x) * scale, offset_y + float(y) * scale

            # Board background + field
            canvas.create_rectangle(4, 4, w - 4, h - 4, fill="#eef2f7", outline="")
            x0, y0 = map_xy(0.0, 0.0)
            x1, y1 = map_xy(field_w, field_h)
            canvas.create_rectangle(x0, y0, x1, y1, fill="#ecfff1", outline="#2d8a52", width=2)
            cx0, cy0 = map_xy(field_w / 2.0, 0.0)
            cx1, cy1 = map_xy(field_w / 2.0, field_h)
            canvas.create_line(cx0, cy0, cx1, cy1, fill="#8eb89f", dash=(4, 3))

            # center circle
            ccx, ccy = map_xy(field_w / 2.0, field_h / 2.0)
            cr = max(10.0, 10.0 * scale)
            canvas.create_oval(ccx - cr, ccy - cr, ccx + cr, ccy + cr, outline="#8eb89f")

            def draw_robot_body(x, y, vx, vy, label):
                px, py = map_xy(x, y)
                mag = math.hypot(vx, vy)
                if mag < 1e-6:
                    canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill="#60a5fa", outline="#1e3a8a", width=2)
                    canvas.create_text(px + 9, py - 8, text=label, anchor="sw", fill="#1e3a8a", font=("Consolas", 9, "bold"))
                    return
                ux, uy = vx / mag, vy / mag
                rx, ry = uy, -ux
                half_depth = 8.75
                half_width = 14.5
                fl = map_xy(x + ux * half_depth + rx * half_width, y + uy * half_depth + ry * half_width)
                fr = map_xy(x + ux * half_depth - rx * half_width, y + uy * half_depth - ry * half_width)
                rr = map_xy(x - ux * half_depth - rx * half_width, y - uy * half_depth - ry * half_width)
                rl = map_xy(x - ux * half_depth + rx * half_width, y - uy * half_depth + ry * half_width)
                canvas.create_polygon(
                    [fl[0], fl[1], fr[0], fr[1], rr[0], rr[1], rl[0], rl[1]],
                    fill="#60a5fa",
                    outline="#1e3a8a",
                    width=2,
                )
                canvas.create_line(fl[0], fl[1], fr[0], fr[1], fill="#f8fafc", width=2)
                arrow_len = 18.0
                aex, aey = map_xy(x + ux * arrow_len, y + uy * arrow_len)
                canvas.create_line(px, py, aex, aey, fill="#1e3a8a", width=2, arrow=tk.LAST, arrowshape=(10, 12, 4))
                canvas.create_text(
                    px + 10,
                    py - 10,
                    text=label,
                    anchor="sw",
                    fill="#0f172a",
                    font=("Consolas", 9, "bold"),
                )

            for rid in sorted(robots.keys()):
                data = robots[rid]
                left_count = int(data.get("left", {}).get("count", 0))
                right_count = int(data.get("right", {}).get("count", 0))
                if left_count <= 0 and right_count <= 0:
                    continue
                final = data.get("final", {})
                x = float(final.get("x", 0.0))
                y = float(final.get("y", 0.0))
                vx = float(final.get("vec_x", 0.0))
                vy = float(final.get("vec_y", 0.0))
                if not (math.isfinite(x) and math.isfinite(y)):
                    continue
                draw_robot_body(x, y, vx, vy, f"R{rid} ({x:.1f},{y:.1f})")

            max_ball_count = 1

            ball_points = []
            raw_centers = list(getattr(imageprocess, "ball_centers", []))
            for pt in raw_centers[:max_ball_count]:
                if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                    try:
                        bx = float(pt[0])
                        by = float(pt[1])
                    except Exception:
                        continue
                    if math.isfinite(bx) and math.isfinite(by):
                        ball_points.append((bx, by))
            if not ball_points:
                ball = list(getattr(imageprocess, "ball_center", []))
                if len(ball) >= 2:
                    try:
                        bx = float(ball[0])
                        by = float(ball[1])
                    except Exception:
                        bx = by = float("nan")
                    if math.isfinite(bx) and math.isfinite(by):
                        ball_points.append((bx, by))

            for idx, (bx, by) in enumerate(ball_points[:max_ball_count]):
                px, py = map_xy(bx, by)
                br = max(4.0, 4.0 * scale)
                fill = "#e11d48"
                outline = "#9f1239"
                label = "Ball"
                canvas.create_oval(px - br, py - br, px + br, py + br, fill=fill, outline=outline, width=2)
                canvas.create_text(px + 10, py + 8, text=f"{label} ({bx:.1f},{by:.1f})", anchor="w", fill=outline, font=("Consolas", 9, "bold"))
        except Exception:
            pass

    def _draw_ball_quality_bars(self, ball_stats):
        canvas = getattr(self, "ball_quality_canvas", None)
        if canvas is None:
            return
        try:
            canvas.delete("all")
            w = max(int(canvas.winfo_width()), 320)
            h = max(int(canvas.winfo_height()), 132)
            canvas.create_rectangle(1, 1, w - 1, h - 1, outline="#e2e8f0", fill="#f8fafc")

            scores = _ball_quality_scores(ball_stats or {})
            overall = _clamp01(scores.get("overall", 0.0))
            status_color = _mix_red_green(overall)
            self.ball_quality_summary_var.set(f"品質燈條: {overall * 100.0:5.1f}%")
            self.ball_quality_summary_label.configure(fg=status_color)

            bar_l = 96
            bar_r = w - 66
            if bar_r <= bar_l + 8:
                bar_l = 82
                bar_r = w - 50
            row_h = max(22.0, float(h - 12) / 5.0)
            top = 6.0
            rows = [
                ("整體", scores.get("overall", 0.0)),
                ("追蹤", scores.get("tracking", 0.0)),
                ("穩定", scores.get("stability", 0.0)),
                ("延遲", scores.get("latency", 0.0)),
                ("數量", scores.get("count", 0.0)),
            ]
            for idx, (label, score) in enumerate(rows):
                y = top + idx * row_h
                s = _clamp01(score)
                color = _mix_red_green(s)
                canvas.create_text(10, y + 9, anchor="nw", text=label, fill="#0f172a", font=("Microsoft JhengHei UI", 10, "bold"))
                canvas.create_rectangle(bar_l, y + 4, bar_r, y + 16, fill="#e2e8f0", outline="#cbd5e1")
                fill_r = bar_l + (bar_r - bar_l) * s
                if fill_r > bar_l:
                    canvas.create_rectangle(bar_l, y + 4, fill_r, y + 16, fill=color, outline="")
                canvas.create_text(bar_r + 8, y + 10, anchor="w", text=f"{s * 100.0:4.0f}%", fill=color, font=("Consolas", 10, "bold"))
        except Exception:
            pass

    def _update_param_robot_top_right(self, robots):
        for rid in range(3):
            widgets = self._param_rt_robot_widgets.get(rid)
            if not isinstance(widgets, dict):
                continue
            lamp = widgets.get("lamp")
            info_var = widgets.get("info_var")
            if lamp is None or info_var is None:
                continue

            data = robots.get(rid) if isinstance(robots, dict) else None
            final = (data or {}).get("final") or {}
            if _finite_xy(final.get("x"), final.get("y")):
                x_txt = _fmt_num(final.get("x"))
                y_txt = _fmt_num(final.get("y"))
            else:
                x_txt = "-"
                y_txt = "-"

            q = self._robot_quality_latest.get(rid, {}) if isinstance(self._robot_quality_latest, dict) else {}
            overall = q.get("overall")
            if isinstance(overall, (int, float)):
                s = _clamp01(overall)
                lamp_color = _mix_red_green(s)
                q_txt = f"{s * 100.0:4.0f}%"
            else:
                lamp_color = "#94a3b8"
                q_txt = "-"

            info_var.set(f"R{rid}  X:{x_txt} Y:{y_txt}  Q:{q_txt}")
            try:
                lamp.configure(fg=lamp_color)
            except Exception:
                pass

    def _draw_robot_quality_bars(self, robots, state):
        canvas = getattr(self, "robot_quality_canvas", None)
        if canvas is None:
            return
        quality_snapshot = {}
        try:
            canvas.delete("all")
            w = max(int(canvas.winfo_width()), 640)
            h = max(int(canvas.winfo_height()), 140)
            canvas.create_rectangle(1, 1, w - 1, h - 1, outline="#e2e8f0", fill="#f8fafc")
            cam1_ids = set(int(v) for v in (state.get("cams", {}).get("Cam1", {}).get("detected_ids") or []))
            cam2_ids = set(int(v) for v in (state.get("cams", {}).get("Cam2", {}).get("detected_ids") or []))
            row_h = max(40.0, float(h - 12) / 3.0)
            top = 6.0
            bar_l = 120.0
            bar_r = max(bar_l + 200.0, float(w - 320.0))

            for rid in range(3):
                y = top + rid * row_h
                target_ids = set()
                if rid < len(self._robot_id_groups):
                    for v in self._robot_id_groups[rid]:
                        try:
                            target_ids.add(int(v))
                        except Exception:
                            continue
                if not target_ids:
                    target_ids.add(int(rid))

                seen_l = bool(cam1_ids.intersection(target_ids))
                seen_r = bool(cam2_ids.intersection(target_ids))
                source_score = 1.0 if (seen_l and seen_r) else (0.62 if (seen_l or seen_r) else 0.0)

                data = robots.get(rid) or {}
                left = data.get("left", {})
                right = data.get("right", {})
                final = data.get("final", {})
                diff_cm = None
                if seen_l and seen_r and _finite_xy(left.get("x"), left.get("y")) and _finite_xy(right.get("x"), right.get("y")):
                    diff_cm = float(math.hypot(float(left.get("x")) - float(right.get("x")), float(left.get("y")) - float(right.get("y"))))
                    agree_score = _clamp01(1.0 - diff_cm / 45.0)
                elif seen_l or seen_r:
                    agree_score = 0.5
                else:
                    agree_score = 0.0

                step_cm = None
                if _finite_xy(final.get("x"), final.get("y")):
                    fx, fy = float(final.get("x")), float(final.get("y"))
                    prev = self._robot_prev_final.get(rid)
                    if isinstance(prev, tuple) and _finite_xy(prev[0], prev[1]):
                        step_cm = float(math.hypot(fx - float(prev[0]), fy - float(prev[1])))
                        stable_score = _clamp01(1.0 - step_cm / 16.0)
                    else:
                        stable_score = 0.8 if source_score > 0.0 else 0.0
                    self._robot_prev_final[rid] = (fx, fy)
                else:
                    stable_score = 0.0

                overall = _clamp01(0.40 * source_score + 0.40 * agree_score + 0.20 * stable_score)
                color = _mix_red_green(overall)
                quality_snapshot[rid] = {
                    "overall": float(overall),
                    "source": float(source_score),
                    "agree": float(agree_score),
                    "stable": float(stable_score),
                    "color": color,
                }

                canvas.create_text(10, y + 11, anchor="nw", text=f"Robot{rid}", fill="#1e3a8a", font=("Consolas", 11, "bold"))
                canvas.create_rectangle(bar_l, y + 8, bar_r, y + 24, fill="#e2e8f0", outline="#cbd5e1")
                fill_r = bar_l + (bar_r - bar_l) * overall
                if fill_r > bar_l:
                    canvas.create_rectangle(bar_l, y + 8, fill_r, y + 24, fill=color, outline="")
                canvas.create_text(bar_r + 8, y + 16, anchor="w", text=f"{overall * 100.0:5.1f}%", fill=color, font=("Consolas", 11, "bold"))

                diff_txt = "N/A" if diff_cm is None else f"{diff_cm:.1f}"
                step_txt = "N/A" if step_cm is None else f"{step_cm:.2f}"
                detail = (
                    f"L/R seen={int(seen_l)}/{int(seen_r)}  "
                    f"一致差(cm)={diff_txt}  步進(cm)={step_txt}  "
                    f"S/A/T={source_score * 100.0:4.0f}/{agree_score * 100.0:4.0f}/{stable_score * 100.0:4.0f}"
                )
                canvas.create_text(120, y + 28, anchor="nw", text=detail, fill="#334155", font=("Consolas", 9, "bold"))
        except Exception:
            pass
        self._robot_quality_latest = quality_snapshot

    def _set_debug_metric_help_expanded(self, expanded):
        self._debug_metric_help_expanded = bool(expanded)
        body = self._debug_metric_help_body
        if body is None:
            return
        if self._debug_metric_help_expanded:
            if not body.winfo_manager():
                body.pack(fill="x", padx=0, pady=(0, 0))
            self._debug_metric_help_toggle_var.set("收合說明")
        else:
            try:
                body.pack_forget()
            except Exception:
                pass
            self._debug_metric_help_toggle_var.set("展開說明")

    def _toggle_debug_metric_help(self):
        self._set_debug_metric_help_expanded(not bool(self._debug_metric_help_expanded))

    def _set_debug_pause_state(self, paused):
        self._debug_updates_paused = bool(paused)
        if self._debug_updates_paused:
            self._debug_pause_btn_text.set("恢復Debug更新")
            try:
                self.debug_pause_btn.configure(bg="#b45309", activebackground="#92400e")
            except Exception:
                pass
        else:
            self._debug_pause_btn_text.set("暫停Debug更新")
            try:
                self.debug_pause_btn.configure(bg="#7c3aed", activebackground="#6d28d9")
            except Exception:
                pass

    def toggle_debug_updates(self):
        paused = not bool(self._debug_updates_paused)
        self._set_debug_pause_state(paused)
        _add_event("debug updates paused" if paused else "debug updates resumed")

    def show_debug_tab(self):
        self.notebook.select(self.debug_tab)

    def show_monitor_tab(self):
        self.notebook.select(self.monitor_tab)

    def show_ball_tab(self):
        self.notebook.select(self.ball_tab)

    def show_param_tab(self):
        self.notebook.select(self.param_tab)

    def _set_ball_mode_runtime(self, mode, notify=True):
        mode = "single"
        ok = False
        if hasattr(imageprocess, "set_ball_detection_mode"):
            try:
                ok = bool(imageprocess.set_ball_detection_mode(mode))
            except Exception:
                ok = False
        if ok and notify:
            _add_event(f"set ball mode -> {mode}")
        elif (not ok) and notify:
            _add_event(f"set ball mode failed -> {mode}")
        return ok

    def refresh_ball_mode(self):
        mode = "single"
        if hasattr(imageprocess, "get_ball_detection_mode"):
            try:
                mode = str(imageprocess.get_ball_detection_mode() or "single").lower()
            except Exception:
                mode = "single"
        self._ball_mode_syncing = True
        self.ball_mode_var.set("single")
        self._ball_mode_syncing = False
        _add_event(f"refresh ball mode <- {self.ball_mode_var.get()}")

    def on_ball_mode_change(self):
        return

    def start_detection(self):
        ball_mode = "single"
        self._set_ball_mode_runtime(ball_mode, notify=False)
        self.apply_capture_params_now(notify=False)

        use_saved = messagebox.askyesno(
            "四點校正模式",
            "要使用已儲存的四個角點嗎？\n\n是：使用預設四點\n否：重新手動選取四點",
        )

        imageprocess.stop_image_thread()
        time.sleep(0.15)

        mode = "manual"
        if use_saved:
            if apply_saved_calibration_points():
                mode = "saved"
            else:
                imageprocess.points_cam0.clear()
                imageprocess.points_cam2.clear()
                messagebox.showwarning("四點校正", "找不到有效的已儲存四點，將改為手動選點。")
        else:
            imageprocess.points_cam0.clear()
            imageprocess.points_cam2.clear()

        imageprocess.set_display_mode("full")
        imageprocess.start_image_thread(show_windows=True)
        _add_event(f"start_image_thread(show_windows=True) - corner mode={mode}, ball mode={ball_mode}")

    def stop_detection(self):
        imageprocess.stop_image_thread()
        _add_event("stop_image_thread()")

    def restart_detection(self):
        imageprocess.stop_image_thread()
        time.sleep(0.15)
        self.start_detection()
        _add_event("restart detection")

    def switch_to_background(self):
        cam0_pts = _normalize_points(getattr(imageprocess, "points_cam0", []))
        cam2_pts = _normalize_points(getattr(imageprocess, "points_cam2", []))
        if not _valid_points(cam0_pts) or not _valid_points(cam2_pts):
            _add_event("switch blocked: 請先在 Cam1/Cam2 各完成 4 點校正")
            return

        imageprocess.stop_image_thread()
        time.sleep(0.15)
        imageprocess.points_cam0[:] = cam0_pts
        imageprocess.points_cam2[:] = cam2_pts
        imageprocess.set_display_mode("compact")
        imageprocess.start_image_thread(show_windows=False)
        # 再套一次，避免 thread 切換瞬間被覆蓋
        imageprocess.points_cam0[:] = cam0_pts
        imageprocess.points_cam2[:] = cam2_pts
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        _add_event("switch to background mode (locked calibration points)")

    def on_close(self):
        try:
            if self._param_apply_job is not None:
                try:
                    self.root.after_cancel(self._param_apply_job)
                except Exception:
                    pass
                self._param_apply_job = None
            if self._bayes_running:
                self._bayes_stop_event.set()
            worker = self._bayes_worker
            if worker is not None and worker.is_alive():
                worker.join(timeout=2.0)
            imageprocess.stop_image_thread()
        finally:
            self.root.destroy()

    def tick(self):
        state = _deep_state_copy()
        runtime = imageprocess.get_runtime_state()
        running = bool(runtime.get("running"))
        show_windows = bool(runtime.get("show_windows"))
        self._update_status_style(running, show_windows)
        if self._debug_updates_paused:
            self.root.after(int(self._ui_update_ms), self.tick)
            return

        cam1_show = self._cam_data_with_hold_cache("Cam1", state["cams"]["Cam1"])
        cam2_show = self._cam_data_with_hold_cache("Cam2", state["cams"]["Cam2"])
        cam1_detail_on, cam1_detail_id = self._cam_detail_opts("Cam1")
        cam2_detail_on, cam2_detail_id = self._cam_detail_opts("Cam2")
        self._set_text(self.cam1_text, _fmt_cam_text("Cam1", cam1_show, cam1_detail_on, cam1_detail_id))
        self._set_text(self.cam2_text, _fmt_cam_text("Cam2", cam2_show, cam2_detail_on, cam2_detail_id))

        robots_raw = state.get("robots", {})
        robots = {}
        for rid in range(3):
            vars_map = self.robot_vars[rid]
            data, is_stale = self._robot_data_with_hold_cache(rid, robots_raw.get(rid))
            if not data:
                vars_map["chosen"].set("Chosen: N/A")
                vars_map["final"].set("Final: X:- Y:- Ang:- Vec:(-, -)")
                vars_map["left"].set("Cam1: X:- Y:- Ang:- Vec:(-, -)")
                vars_map["right"].set("Cam2: X:- Y:- Ang:- Vec:(-, -)")
                mon_vars = self.monitor_robot_vars.get(rid)
                if mon_vars is not None:
                    mon_vars["line1"].set("X:- Y:- Ang:-")
                    mon_vars["line2"].set("Vec:(-, -) Source:N/A")
                continue

            data = self._smooth_robot_data(rid, data)
            robots[rid] = data
            final = data["final"]
            left = data["left"]
            right = data["right"]
            stale_tag = " [CACHE]" if is_stale else ""
            vars_map["chosen"].set(f"Chosen: {data['chosen']}{stale_tag} (左右相機融合選擇)")
            vars_map["final"].set(
                f"Final: X:{_fmt_num(final['x'])} Y:{_fmt_num(final['y'])} "
                f"Ang:{_fmt_num(final['deg'])} Vec:({_fmt_num(final['vec_x'], 3)}, {_fmt_num(final['vec_y'], 3)})"
            )
            vars_map["left"].set(
                f"Cam1: X:{_fmt_num(left['x'])} Y:{_fmt_num(left['y'])} "
                f"Ang:{_fmt_num(left['deg'])} Vec:({_fmt_num(left['vec_x'], 3)}, {_fmt_num(left['vec_y'], 3)})"
            )
            vars_map["right"].set(
                f"Cam2: X:{_fmt_num(right['x'])} Y:{_fmt_num(right['y'])} "
                f"Ang:{_fmt_num(right['deg'])} Vec:({_fmt_num(right['vec_x'], 3)}, {_fmt_num(right['vec_y'], 3)})"
            )

            mon_vars = self.monitor_robot_vars.get(rid)
            if mon_vars is not None:
                mon_vars["line1"].set(
                    f"X:{_fmt_num(final['x'])} Y:{_fmt_num(final['y'])} Ang:{_fmt_num(final['deg'])}"
                )
                mon_vars["line2"].set(
                    f"Vec:({_fmt_num(final['vec_x'], 3)}, {_fmt_num(final['vec_y'], 3)}) Source:{data['chosen']}"
                )

        self._draw_field_map(robots, canvas=self.map_canvas)
        self._draw_field_map(robots, canvas=self.monitor_map_canvas)
        self._draw_field_map(robots, canvas=self.ball_map_canvas)
        self._draw_robot_quality_bars(robots, state)
        self._update_param_robot_top_right(robots)
        self._update_ui_alerts(state)
        ball_stats = {}
        if hasattr(imageprocess, "get_ball_debug_stats"):
            try:
                ball_stats = imageprocess.get_ball_debug_stats() or {}
            except Exception as err:
                ball_stats = {"error": str(err)}
        self._last_ball_stats = ball_stats
        self._draw_ball_quality_bars(ball_stats)
        latest_ball = ball_stats.get("latest") or {}
        center_real = latest_ball.get("center_real")
        if isinstance(center_real, (list, tuple)) and len(center_real) >= 2:
            self.ball_coord_var.set(f"X: {_fmt_num(center_real[0])}    Y: {_fmt_num(center_real[1])}")
        else:
            fallback_ball = list(getattr(imageprocess, "ball_center", []))
            if len(fallback_ball) >= 2:
                self.ball_coord_var.set(f"X: {_fmt_num(fallback_ball[0])}    Y: {_fmt_num(fallback_ball[1])}")
            else:
                self.ball_coord_var.set("X: -    Y: -")
        centers_real = latest_ball.get("centers_real") or []
        if centers_real:
            self.ball_centers_var.set(f"Ball: {_fmt_ball_centers(centers_real)}")
        else:
            fallback_centers = list(getattr(imageprocess, "ball_centers", []))
            self.ball_centers_var.set(f"Ball: {_fmt_ball_centers(fallback_centers)}")
        self._set_text(self.ball_text, _fmt_ball_text(ball_stats))
        self._poll_bayes_updates()
        self.ball_mode_var.set("single")
        self.root.after(int(self._ui_update_ms), self.tick)


def main():
    install_hooks()
    app = PipelineDebugWindow()
    app.root.mainloop()


if __name__ == "__main__":
    main()
