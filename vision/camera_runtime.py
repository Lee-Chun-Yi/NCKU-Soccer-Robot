import json
import threading
from pathlib import Path

import cv2


RUNTIME_CAPTURE_PARAMS_FILE = Path(__file__).resolve().parent / "data" / "runtime_capture_params.json"
CAMERA_CAPTURE_DEFAULTS = {
    "width": 640,
    "height": 480,
    "fps": 30,
    "use_mjpg": True,
    "auto_exposure": True,
    "exposure": -6.0,
    "auto_focus": True,
    "focus": 0.0,
    "gain": 0.0,
    "brightness": -1.0,  # <0 means "skip setting brightness"
}
ARUCO_UPSCALE_DEFAULTS = {
    "upscale_factors": [1.6],
}

_camera_capture_lock = threading.Lock()
_camera_capture_config = CAMERA_CAPTURE_DEFAULTS.copy()
_active_caps = []

_aruco_upscale_lock = threading.Lock()
_aruco_upscale_options = ARUCO_UPSCALE_DEFAULTS.copy()


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


def _normalize_camera_capture_config(raw):
    base = CAMERA_CAPTURE_DEFAULTS.copy()
    if isinstance(raw, dict):
        for key in base.keys():
            if key in raw:
                base[key] = raw[key]
    out = {}
    out["width"] = max(320, min(3840, _to_int(base.get("width"), 640)))
    out["height"] = max(240, min(2160, _to_int(base.get("height"), 480)))
    out["fps"] = max(1, min(120, _to_int(base.get("fps"), 30)))
    out["use_mjpg"] = bool(base.get("use_mjpg", True))
    out["auto_exposure"] = bool(base.get("auto_exposure", True))
    out["exposure"] = max(-16.0, min(0.0, _to_float(base.get("exposure"), -6.0)))
    out["auto_focus"] = bool(base.get("auto_focus", True))
    out["focus"] = max(0.0, min(255.0, _to_float(base.get("focus"), 0.0)))
    out["gain"] = max(0.0, min(255.0, _to_float(base.get("gain"), 0.0)))
    out["brightness"] = max(-1.0, min(255.0, _to_float(base.get("brightness"), -1.0)))
    return out


def _set_camera_prop(cap, prop_id, value):
    try:
        cap.set(prop_id, value)
        return True
    except Exception:
        return False


def _apply_camera_capture_config_to_cap(cap, cfg=None):
    if cap is None:
        return False
    if cfg is None:
        with _camera_capture_lock:
            cfg = _camera_capture_config.copy()
    if not getattr(cap, "isOpened", lambda: False)():
        return False

    _set_camera_prop(cap, cv2.CAP_PROP_FRAME_WIDTH, int(cfg["width"]))
    _set_camera_prop(cap, cv2.CAP_PROP_FRAME_HEIGHT, int(cfg["height"]))
    _set_camera_prop(cap, cv2.CAP_PROP_FPS, int(cfg["fps"]))
    if bool(cfg.get("use_mjpg", True)):
        _set_camera_prop(cap, cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    if bool(cfg.get("auto_exposure", True)):
        _set_camera_prop(cap, cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        _set_camera_prop(cap, cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
    else:
        _set_camera_prop(cap, cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        _set_camera_prop(cap, cv2.CAP_PROP_AUTO_EXPOSURE, 0.0)
        _set_camera_prop(cap, cv2.CAP_PROP_EXPOSURE, float(cfg["exposure"]))

    if bool(cfg.get("auto_focus", True)):
        _set_camera_prop(cap, cv2.CAP_PROP_AUTOFOCUS, 1)
    else:
        _set_camera_prop(cap, cv2.CAP_PROP_AUTOFOCUS, 0)
        _set_camera_prop(cap, cv2.CAP_PROP_FOCUS, float(cfg["focus"]))

    _set_camera_prop(cap, cv2.CAP_PROP_GAIN, float(cfg["gain"]))
    if float(cfg.get("brightness", -1.0)) >= 0.0:
        _set_camera_prop(cap, cv2.CAP_PROP_BRIGHTNESS, float(cfg["brightness"]))
    return True


def _apply_camera_capture_config_to_caps(caps):
    with _camera_capture_lock:
        cfg = _camera_capture_config.copy()
    for cap in list(caps or []):
        _apply_camera_capture_config_to_cap(cap, cfg)


def _set_active_caps(caps):
    global _active_caps
    with _camera_capture_lock:
        _active_caps = [c for c in (caps or []) if c is not None]


def _clear_active_caps():
    global _active_caps
    with _camera_capture_lock:
        _active_caps = []


def get_camera_capture_config():
    with _camera_capture_lock:
        return _camera_capture_config.copy()


def set_camera_capture_config(config, apply_now=True, replace=False, persist=False):
    global _camera_capture_config
    with _camera_capture_lock:
        merged = dict(config or {}) if replace else _camera_capture_config.copy()
        if not replace and isinstance(config, dict):
            merged.update(config)
        normalized = _normalize_camera_capture_config(merged)
        _camera_capture_config = normalized
        active_caps = list(_active_caps)
    if apply_now:
        _apply_camera_capture_config_to_caps(active_caps)
    state = {
        "config": normalized.copy(),
        "applied_caps": len(active_caps) if apply_now else 0,
    }
    if persist:
        save_runtime_capture_params()
    return state


def _normalize_upscale_factors(raw):
    if isinstance(raw, str):
        tokens = raw.replace(";", ",").replace("|", ",").split(",")
        values = []
        for tok in tokens:
            tok = tok.strip()
            if not tok:
                continue
            try:
                values.append(float(tok))
            except Exception:
                continue
        raw = values
    if not isinstance(raw, (list, tuple)):
        raw = ARUCO_UPSCALE_DEFAULTS["upscale_factors"]
    out = []
    for val in raw:
        try:
            f = float(val)
        except Exception:
            continue
        if f <= 1.0:
            continue
        out.append(max(1.05, min(4.0, f)))
    if not out:
        out = list(ARUCO_UPSCALE_DEFAULTS["upscale_factors"])
    unique = []
    for f in sorted(out):
        if not unique or abs(f - unique[-1]) >= 0.05:
            unique.append(round(float(f), 2))
    return unique


def get_aruco_upscale_options():
    with _aruco_upscale_lock:
        return {
            "upscale_factors": list(_aruco_upscale_options.get("upscale_factors", [])),
        }


def set_aruco_upscale_options(options, replace=False, persist=False):
    global _aruco_upscale_options
    with _aruco_upscale_lock:
        merged = dict(options or {}) if replace else _aruco_upscale_options.copy()
        if not replace and isinstance(options, dict):
            merged.update(options)
        factors = _normalize_upscale_factors(merged.get("upscale_factors"))
        _aruco_upscale_options = {
            "upscale_factors": factors,
        }
        state = {
            "upscale_factors": list(_aruco_upscale_options["upscale_factors"]),
        }
    if persist:
        save_runtime_capture_params()
    return state


def _runtime_capture_default_payload():
    return {
        "camera_capture": CAMERA_CAPTURE_DEFAULTS.copy(),
        "aruco_upscale": {
            "upscale_factors": list(ARUCO_UPSCALE_DEFAULTS["upscale_factors"]),
        },
    }


def _read_runtime_capture_payload(path=None):
    params_path = Path(path) if path else RUNTIME_CAPTURE_PARAMS_FILE
    if not params_path.exists():
        return _runtime_capture_default_payload(), params_path, False
    with params_path.open("r", encoding="utf-8") as f:
        loaded = json.load(f)
    if not isinstance(loaded, dict):
        loaded = {}
    return loaded, params_path, True


def save_runtime_capture_params(path=None):
    params_path = Path(path) if path else RUNTIME_CAPTURE_PARAMS_FILE
    params_path.parent.mkdir(parents=True, exist_ok=True)
    with _camera_capture_lock:
        cam_cfg = _camera_capture_config.copy()
    with _aruco_upscale_lock:
        up_cfg = {
            "upscale_factors": list(_aruco_upscale_options.get("upscale_factors", [])),
        }
    payload = {
        "camera_capture": cam_cfg,
        "aruco_upscale": up_cfg,
    }
    with params_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return str(params_path)


def load_runtime_capture_params(path=None, apply_now=True):
    global _camera_capture_config, _aruco_upscale_options
    loaded, params_path, _exists = _read_runtime_capture_payload(path)
    cam_raw = loaded.get("camera_capture") if isinstance(loaded.get("camera_capture"), dict) else {}
    if not cam_raw:
        cam_raw = {k: loaded.get(k) for k in CAMERA_CAPTURE_DEFAULTS.keys() if k in loaded}
    up_raw = loaded.get("aruco_upscale") if isinstance(loaded.get("aruco_upscale"), dict) else {}
    if "upscale_factors" not in up_raw and "upscale_factors" in loaded:
        up_raw = {"upscale_factors": loaded.get("upscale_factors")}

    cam_cfg = _normalize_camera_capture_config(cam_raw)
    up_cfg = {
        "upscale_factors": _normalize_upscale_factors(up_raw.get("upscale_factors")),
    }
    with _camera_capture_lock:
        _camera_capture_config = cam_cfg
        active_caps = list(_active_caps)
    with _aruco_upscale_lock:
        _aruco_upscale_options = up_cfg

    if apply_now:
        _apply_camera_capture_config_to_caps(active_caps)
    return {
        "camera_capture": cam_cfg.copy(),
        "aruco_upscale": {"upscale_factors": list(up_cfg["upscale_factors"])},
        "params_file": str(params_path),
        "applied_caps": len(active_caps) if apply_now else 0,
    }


def init_runtime_capture_params():
    if RUNTIME_CAPTURE_PARAMS_FILE.exists():
        try:
            load_runtime_capture_params(apply_now=False)
        except Exception as err:
            print(f"[runtime_capture] load params failed, fallback to default: {err}")
            with _camera_capture_lock:
                globals()["_camera_capture_config"] = CAMERA_CAPTURE_DEFAULTS.copy()
            with _aruco_upscale_lock:
                globals()["_aruco_upscale_options"] = {
                    "upscale_factors": list(ARUCO_UPSCALE_DEFAULTS["upscale_factors"]),
                }
    else:
        try:
            load_runtime_capture_params(apply_now=False)
            save_runtime_capture_params()
        except Exception as err:
            print(f"[runtime_capture] init default params failed: {err}")
