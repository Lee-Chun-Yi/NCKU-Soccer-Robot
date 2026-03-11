import json
import threading
from pathlib import Path

import cv2


ARUCO_PARAMS_FILE = Path(__file__).resolve().parent / "data" / "aruco_detector_params.json"
ARUCO_DEFAULT_PARAMS = {
    "cornerRefinementMethod": int(cv2.aruco.CORNER_REFINE_APRILTAG),
    "cornerRefinementWinSize": 5,
    "cornerRefinementMaxIterations": 50,
    "cornerRefinementMinAccuracy": 0.01,
    "adaptiveThreshWinSizeMin": 3,
    "adaptiveThreshWinSizeMax": 71,
    "adaptiveThreshWinSizeStep": 2,
    "adaptiveThreshConstant": 5.0,
    "minMarkerPerimeterRate": 0.0015,
    "maxMarkerPerimeterRate": 6.0,
    "minCornerDistanceRate": 0.003,
    "minMarkerDistanceRate": 0.003,
    "minDistanceToBorder": 2,
    "minOtsuStdDev": 2.0,
    "perspectiveRemoveIgnoredMarginPerCell": 0.12,
    "errorCorrectionRate": 0.7,
    "detectInvertedMarker": True,
}

_ARUCO_REFINE_METHODS = {
    int(cv2.aruco.CORNER_REFINE_NONE),
    int(cv2.aruco.CORNER_REFINE_SUBPIX),
    int(cv2.aruco.CORNER_REFINE_CONTOUR),
    int(cv2.aruco.CORNER_REFINE_APRILTAG),
}

_aruco_lock = threading.Lock()
_aruco_detector_params = ARUCO_DEFAULT_PARAMS.copy()
_aruco_detector = None
_aruco_revision = 0


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


def _normalize_aruco_params(raw):
    base = ARUCO_DEFAULT_PARAMS.copy()
    if isinstance(raw, dict):
        for key in base.keys():
            if key in raw:
                base[key] = raw[key]

    out = {}
    out["cornerRefinementMethod"] = _to_int(
        base.get("cornerRefinementMethod"),
        ARUCO_DEFAULT_PARAMS["cornerRefinementMethod"],
    )
    if out["cornerRefinementMethod"] not in _ARUCO_REFINE_METHODS:
        out["cornerRefinementMethod"] = ARUCO_DEFAULT_PARAMS["cornerRefinementMethod"]

    out["cornerRefinementWinSize"] = _clamp(_to_int(base.get("cornerRefinementWinSize"), 5), 1, 31)
    out["cornerRefinementMaxIterations"] = _clamp(_to_int(base.get("cornerRefinementMaxIterations"), 50), 1, 200)
    out["cornerRefinementMinAccuracy"] = _clamp(_to_float(base.get("cornerRefinementMinAccuracy"), 0.01), 1e-6, 1.0)

    win_min = _clamp(_to_int(base.get("adaptiveThreshWinSizeMin"), 3), 3, 151)
    win_max = _clamp(_to_int(base.get("adaptiveThreshWinSizeMax"), 71), 3, 201)
    if win_min % 2 == 0:
        win_min += 1
    if win_max % 2 == 0:
        win_max += 1
    if win_max < win_min:
        win_max = win_min
    out["adaptiveThreshWinSizeMin"] = win_min
    out["adaptiveThreshWinSizeMax"] = win_max
    out["adaptiveThreshWinSizeStep"] = _clamp(_to_int(base.get("adaptiveThreshWinSizeStep"), 2), 1, 51)
    out["adaptiveThreshConstant"] = _clamp(_to_float(base.get("adaptiveThreshConstant"), 5.0), -30.0, 30.0)

    out["minMarkerPerimeterRate"] = _clamp(_to_float(base.get("minMarkerPerimeterRate"), 0.0015), 1e-6, 1.0)
    out["maxMarkerPerimeterRate"] = _clamp(_to_float(base.get("maxMarkerPerimeterRate"), 6.0), out["minMarkerPerimeterRate"], 20.0)
    out["minCornerDistanceRate"] = _clamp(_to_float(base.get("minCornerDistanceRate"), 0.003), 1e-6, 1.0)
    out["minMarkerDistanceRate"] = _clamp(_to_float(base.get("minMarkerDistanceRate"), 0.003), 1e-6, 1.0)
    out["minDistanceToBorder"] = _clamp(_to_int(base.get("minDistanceToBorder"), 2), 0, 100)
    out["minOtsuStdDev"] = _clamp(_to_float(base.get("minOtsuStdDev"), 2.0), 0.0, 50.0)
    out["perspectiveRemoveIgnoredMarginPerCell"] = _clamp(
        _to_float(base.get("perspectiveRemoveIgnoredMarginPerCell"), 0.12), 0.0, 1.0
    )
    out["errorCorrectionRate"] = _clamp(_to_float(base.get("errorCorrectionRate"), 0.7), 0.0, 1.0)

    raw_inv = base.get("detectInvertedMarker")
    if isinstance(raw_inv, str):
        out["detectInvertedMarker"] = raw_inv.strip().lower() in ("1", "true", "yes", "on")
    else:
        out["detectInvertedMarker"] = bool(raw_inv)

    return out


def _build_aruco_detector_from_params(params):
    cfg = _normalize_aruco_params(params)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = int(cfg["cornerRefinementMethod"])
    detector_params.cornerRefinementWinSize = int(cfg["cornerRefinementWinSize"])
    detector_params.cornerRefinementMaxIterations = int(cfg["cornerRefinementMaxIterations"])
    detector_params.cornerRefinementMinAccuracy = float(cfg["cornerRefinementMinAccuracy"])
    detector_params.adaptiveThreshWinSizeMin = int(cfg["adaptiveThreshWinSizeMin"])
    detector_params.adaptiveThreshWinSizeMax = int(cfg["adaptiveThreshWinSizeMax"])
    detector_params.adaptiveThreshWinSizeStep = int(cfg["adaptiveThreshWinSizeStep"])
    detector_params.adaptiveThreshConstant = float(cfg["adaptiveThreshConstant"])
    detector_params.minMarkerPerimeterRate = float(cfg["minMarkerPerimeterRate"])
    detector_params.maxMarkerPerimeterRate = float(cfg["maxMarkerPerimeterRate"])
    detector_params.minCornerDistanceRate = float(cfg["minCornerDistanceRate"])
    detector_params.minMarkerDistanceRate = float(cfg["minMarkerDistanceRate"])
    detector_params.minDistanceToBorder = int(cfg["minDistanceToBorder"])
    detector_params.minOtsuStdDev = float(cfg["minOtsuStdDev"])
    detector_params.perspectiveRemoveIgnoredMarginPerCell = float(cfg["perspectiveRemoveIgnoredMarginPerCell"])
    detector_params.errorCorrectionRate = float(cfg["errorCorrectionRate"])
    detector_params.detectInvertedMarker = bool(cfg["detectInvertedMarker"])
    return cv2.aruco.ArucoDetector(dictionary, detector_params)


def _set_aruco_detector_params_internal(params, replace=False):
    global _aruco_detector_params, _aruco_detector, _aruco_revision
    with _aruco_lock:
        merged = dict(params or {}) if replace else _aruco_detector_params.copy()
    if not replace and isinstance(params, dict):
        merged.update(params)
    normalized = _normalize_aruco_params(merged)
    detector = _build_aruco_detector_from_params(normalized)
    with _aruco_lock:
        _aruco_detector_params = normalized
        _aruco_detector = detector
        _aruco_revision += 1
        return {
            "params": _aruco_detector_params.copy(),
            "revision": int(_aruco_revision),
            "params_file": str(ARUCO_PARAMS_FILE),
        }


def get_aruco_detector():
    with _aruco_lock:
        detector = _aruco_detector
        params_snapshot = _aruco_detector_params.copy()
    if detector is None:
        _set_aruco_detector_params_internal(params_snapshot, replace=True)
        with _aruco_lock:
            detector = _aruco_detector
    return detector


def get_aruco_detector_state():
    with _aruco_lock:
        return {
            "params": _aruco_detector_params.copy(),
            "revision": int(_aruco_revision),
            "params_file": str(ARUCO_PARAMS_FILE),
        }


def get_aruco_detector_params():
    return get_aruco_detector_state()["params"]


def set_aruco_detector_params(params, persist=False, replace=False):
    state = _set_aruco_detector_params_internal(params, replace=replace)
    try:
        print(f"[aruco] applied params rev={state.get('revision')}")
    except Exception:
        pass
    if persist:
        save_aruco_detector_params()
    return state


def save_aruco_detector_params(path=None):
    params_path = Path(path) if path else ARUCO_PARAMS_FILE
    params_path.parent.mkdir(parents=True, exist_ok=True)
    with _aruco_lock:
        payload = _aruco_detector_params.copy()
    with params_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return str(params_path)


def load_aruco_detector_params(path=None):
    params_path = Path(path) if path else ARUCO_PARAMS_FILE
    with params_path.open("r", encoding="utf-8") as f:
        loaded = json.load(f)
    return _set_aruco_detector_params_internal(loaded, replace=True)


def reset_aruco_detector_params(persist=False):
    state = _set_aruco_detector_params_internal(ARUCO_DEFAULT_PARAMS, replace=True)
    if persist:
        save_aruco_detector_params()
    return state


def init_aruco_detector_params():
    if ARUCO_PARAMS_FILE.exists():
        try:
            load_aruco_detector_params()
        except Exception as err:
            print(f"[aruco] load params failed, fallback to default: {err}")
            reset_aruco_detector_params(persist=False)
    else:
        reset_aruco_detector_params(persist=False)
