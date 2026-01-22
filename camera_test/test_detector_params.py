"""
[第一階段] 先讓標記「被看見」(Detection Rate)

如果畫面中有標記但抓不到，或者距離稍遠就丟失，請優先調整此區。

1. p.minMarkerPerimeterRate (最優先)
   - 預設: 0.03 (建議改 0.01)
   - 用途: 允許更小的標記被檢測到。若標記離鏡頭遠，務必調低。

2. p.adaptiveThreshConstant
   - 預設: 7
   - 用途: 二值化閾值常數。
   - 調整: 光線暗/對比低 -> 調低(3~5)；雜訊太多 -> 調高。

3. p.adaptiveThreshWinSize (Min/Max/Step)
   - 用途: 搜尋窗口大小。
   - 調整: 若要提升 FPS (運算速度)，可降低 Max (如 23) 並加大 Step (如 10)。
"""

"""
[第二階段] 讓 ID 讀取更穩定 (Identification Stability)

如果標記有框出來，但 ID 讀錯，或在兩個 ID 間跳動。

4. p.errorCorrectionRate
   - 預設: 0.6
   - 用途: 錯誤修正率。
   - 調整: 設太高容易讀錯 ID。若發生誤判，請調回 0.6。

5. p.perspectiveRemoveIgnoredMarginPerCell
   - 預設: 0.13
   - 用途: 讀取位元時忽略邊緣的比例。
   - 調整: 標記傾斜或模糊時，可調高此值。
"""


"""
[第三階段] 讓座標更精準 (Pose Estimation Accuracy)

6. p.cornerRefinementMethod
   - 選項: 
     * CORNER_REFINE_NONE: 快，不準。
     * CORNER_REFINE_SUBPIX: 標準。
     * CORNER_REFINE_APRILTAG: 最準、抗變形強，但最慢 (運算成本高)。
   - 調整: 若 FPS 太低無法接受，請改回 SUBPIX。

7. p.cornerRefinementWinSize
   - 用途: 優化窗口大小。標記很小時保持 5 即可。
"""

import argparse
from collections import defaultdict
from copy import deepcopy
import cv2
import numpy as np
import zmq

TUNE_STEPS = [
    ("cornerRefinementWinSize", [ -2, -1, 1, 2 ]),
    ("cornerRefinementMaxIterations", [ 20, 40, -20 ]),
    ("cornerRefinementMinAccuracy", [ -0.002, 0.002, -0.005 ]),
    ("adaptiveThreshWinSizeMin", [ -1, 1 ]),
    ("adaptiveThreshWinSizeStep", [ -2, -1, 1, 2 ]),
    ("adaptiveThreshConstant", [ -2, -1, 1, 2 ]),
    ("minMarkerPerimeterRate", [ -0.0003, 0.0003 ]),
    ("perspectiveRemoveIgnoredMarginPerCell", [ 0.02, -0.02 ]),
    ("errorCorrectionRate", [ 0.05, -0.05 ]),
]


DEFAULT_OVERRIDES = {
    # Align baseline with latest imageprocess_complete1.py settings
    "cornerRefinementWinSize": 5,
    "cornerRefinementMaxIterations": 50,
    "cornerRefinementMinAccuracy": 0.01,
    "adaptiveThreshWinSizeMin": 3,
    "adaptiveThreshWinSizeMax": 33,
    "adaptiveThreshWinSizeStep": 8,
    "adaptiveThreshConstant": 5,
    "minMarkerPerimeterRate": 0.01,
    "maxMarkerPerimeterRate": 6.0,
    "minCornerDistanceRate": 0.01,
    "minOtsuStdDev": 2.0,
    "perspectiveRemoveIgnoredMarginPerCell": 0.1,
    "errorCorrectionRate": 0.7,
}

PARAM_META = [
    ("cornerRefinementWinSize", "int", 1),
    ("cornerRefinementMaxIterations", "int", 5),
    ("cornerRefinementMinAccuracy", "float", 0.001),
    ("adaptiveThreshWinSizeMin", "int", 1),
    ("adaptiveThreshWinSizeMax", "int", 2),
    ("adaptiveThreshWinSizeStep", "int", 1),
    ("adaptiveThreshConstant", "float", 1.0),
    ("minMarkerPerimeterRate", "float", 0.0002),
    ("maxMarkerPerimeterRate", "float", 0.1),
    ("minCornerDistanceRate", "float", 0.002),
    ("minOtsuStdDev", "float", 0.2),
    ("perspectiveRemoveIgnoredMarginPerCell", "float", 0.02),
    ("errorCorrectionRate", "float", 0.05),
]

# ==========================
# 預設來源設定（可直接改這裡）
# ==========================
# 若要用網路輸入（例如 cam_send/cam_recieve 的 PUB/SUB），把 USE_NET_MODE 設 True
# 並調整 NET_PORT / NET_TOPIC。一次只會使用一個來源。
USE_NET_MODE = False
NET_IP = "127.0.0.1"
NET_PORT = 5555
NET_TOPIC = "cam0"  # cam0 / cam2
LOCAL_CAMERA_ID = 0
def _fmt_delta(delta: float) -> str:
    """Format delta for display, suppressing floating drift."""
    if abs(delta) < 1e-6:
        return "+0"
    if abs(delta) < 1e-3:
        return f"{delta:+.3e}"
    return f"{delta:+.4g}"


def _fmt_val(val: float) -> str:
    """Friendly value formatter to avoid long floating tails."""
    if isinstance(val, int):
        return str(val)
    if abs(val) < 1e-6:
        return "0"
    if abs(val) < 1e-3:
        return f"{val:.3e}"
    return f"{val:.6g}"


def build_detector(overrides):
    """Instantiate detector and apply parameter overrides (by constructing a fresh ArucoDetector)."""
    params = cv2.aruco.DetectorParameters()
    for k, v in DEFAULT_OVERRIDES.items():
        if hasattr(params, k):
            setattr(params, k, v)
    for k, v in overrides.items():
        if hasattr(params, k):
            setattr(params, k, v)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    return cv2.aruco.ArucoDetector(dictionary, params)


def summarize_params(detector, title: str):
    params = detector.getDetectorParameters()
    print(f"== {title} ==")
    print(f"cornerRefinementWinSize      : {params.cornerRefinementWinSize}")
    print(f"cornerRefinementMaxIterations: {params.cornerRefinementMaxIterations}")
    print(f"cornerRefinementMinAccuracy  : {params.cornerRefinementMinAccuracy}")
    print(f"adaptiveThreshWinSizeMin     : {params.adaptiveThreshWinSizeMin}")
    print(f"adaptiveThreshWinSizeMax     : {params.adaptiveThreshWinSizeMax}")
    print(f"adaptiveThreshWinSizeStep    : {params.adaptiveThreshWinSizeStep}")
    print(f"adaptiveThreshConstant       : {params.adaptiveThreshConstant}")
    print(f"minMarkerPerimeterRate       : {params.minMarkerPerimeterRate}")
    print(f"maxMarkerPerimeterRate       : {params.maxMarkerPerimeterRate}")
    print(f"minCornerDistanceRate        : {params.minCornerDistanceRate}")
    print(f"minOtsuStdDev                : {params.minOtsuStdDev}")
    print(f"perspectiveRemoveIgnoredMarginPerCell: {params.perspectiveRemoveIgnoredMarginPerCell}")
    print(f"errorCorrectionRate          : {params.errorCorrectionRate}")
    print()


def clamp_params(params):
    """Ensure required bounds."""
    params["adaptiveThreshWinSizeMin"] = max(3, int(round(params.get("adaptiveThreshWinSizeMin", 3))))
    params["adaptiveThreshWinSizeMax"] = max(params["adaptiveThreshWinSizeMin"], params.get("adaptiveThreshWinSizeMax", 33))
    params["cornerRefinementWinSize"] = max(1, int(round(params.get("cornerRefinementWinSize", 5))))
    if "cornerRefinementMaxIterations" in params:
        params["cornerRefinementMaxIterations"] = max(1, int(round(params["cornerRefinementMaxIterations"])))
    if "cornerRefinementMinAccuracy" in params:
        params["cornerRefinementMinAccuracy"] = max(1e-5, float(params["cornerRefinementMinAccuracy"]))
    if "adaptiveThreshWinSizeStep" in params:
        params["adaptiveThreshWinSizeStep"] = max(1, int(round(params["adaptiveThreshWinSizeStep"])))
    return params


def extract_default_params():
    """Pull defaults from build_detector({}) using DEFAULT_OVERRIDES as baseline."""
    detector = build_detector({})
    p = detector.getDetectorParameters()
    return {
        "cornerRefinementWinSize": p.cornerRefinementWinSize,
        "cornerRefinementMaxIterations": p.cornerRefinementMaxIterations,
        "cornerRefinementMinAccuracy": p.cornerRefinementMinAccuracy,
        "adaptiveThreshWinSizeMin": p.adaptiveThreshWinSizeMin,
        "adaptiveThreshWinSizeMax": p.adaptiveThreshWinSizeMax,
        "adaptiveThreshWinSizeStep": p.adaptiveThreshWinSizeStep,
        "adaptiveThreshConstant": p.adaptiveThreshConstant,
        "minMarkerPerimeterRate": p.minMarkerPerimeterRate,
        "maxMarkerPerimeterRate": p.maxMarkerPerimeterRate,
        "minCornerDistanceRate": p.minCornerDistanceRate,
        "minOtsuStdDev": p.minOtsuStdDev,
        "perspectiveRemoveIgnoredMarginPerCell": p.perspectiveRemoveIgnoredMarginPerCell,
        "errorCorrectionRate": p.errorCorrectionRate,
    }


def apply_overrides(base_params, overrides):
    merged = deepcopy(base_params)
    merged.update(overrides)
    return clamp_params(merged)


def _draw_detection(frame, corners, ids, title, score, param_label=""):
    """Overlay detection result for visualization."""
    disp = frame.copy()
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(disp, corners, ids)
        for i, mid in enumerate(ids.flatten()):
            c = corners[i][0].mean(axis=0).astype(int)
            cv2.putText(disp, f"ID:{mid}", (c[0], c[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
    cv2.putText(disp, title, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    cv2.putText(disp, f"score:{score:.1f}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    if param_label:
        cv2.putText(disp, param_label, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,200,0), 2)
    return disp


def score_detections(detector, cap, frames_per_round, weight_count, weight_stability, show=False, title="", param_label=""):
    """Capture frames_per_round frames, run detection, and compute score (optionally visualized)."""
    total_dets = 0
    best_streak = defaultdict(int)
    current_streak = defaultdict(int)
    aborted = False

    for _ in range(frames_per_round):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        present = set()
        if ids is not None:
            ids_flat = ids.flatten().tolist()
            total_dets += len(ids_flat)
            present.update(ids_flat)
        # 更新穩定度：連續出現的幀數當作每個 id 的穩定性
        for mid in present:
            current_streak[mid] += 1
            best_streak[mid] = max(best_streak[mid], current_streak[mid])
        for mid in list(current_streak.keys()):
            if mid not in present:
                current_streak[mid] = 0

        if show:
            disp = _draw_detection(frame, corners, ids, title, total_dets, param_label)
            cv2.imshow("Aruco Param Test", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC to abort
                aborted = True
                break

    stability = sum(best_streak.values())
    score = weight_count * total_dets + weight_stability * stability
    return total_dets, stability, score, aborted


def render_param_panel(params, baseline, selected_idx):
    panel = np.zeros((520, 520, 3), dtype=np.uint8)
    cv2.putText(panel, "Manual Param Panel", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    y = 50
    for i, (name, _, _) in enumerate(PARAM_META):
        val = params.get(name, 0)
        base = baseline.get(name, 0)
        delta = val - base
        text = f"{'>' if i==selected_idx else ' '} {name}: {_fmt_val(val)} (d{_fmt_delta(delta)})"
        color = (0, 255, 0) if i == selected_idx else (200, 200, 200)
        cv2.putText(panel, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        y += 20
    cv2.putText(panel, "Up/Down: select  +/-: adjust   ESC/Q: quit", (10, panel.shape[0]-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 255), 1)
    return panel


def apply_delta(params, name, delta, p_type):
    if p_type == "int":
        params[name] = int(round(params.get(name, 0) + delta))
    else:
        params[name] = float(params.get(name, 0) + delta)
    return clamp_params(params)


def _make_net_receiver(ip: str, port: int, topic: str):
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 1)
    try:
        sock.setsockopt_string(zmq.SUBSCRIBE, topic)
    except AttributeError:
        sock.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
    sock.connect(f"tcp://{ip}:{port}")
    return ctx, sock


def _recv_frame(sock):
    if sock.poll(50):
        parts = sock.recv_multipart(flags=zmq.NOBLOCK)
        if len(parts) != 2:
            return None
        _, jpg = parts
        arr = np.frombuffer(jpg, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return None


def run_manual(camera_id=None, step_scale=1.0, net_cfg=None):
    cap = None
    net_ctx = net_sock = None
    if net_cfg:
        net_ctx, net_sock = _make_net_receiver(net_cfg["ip"], net_cfg["port"], net_cfg["topic"])
        print(f"[INFO] Net mode: tcp://{net_cfg['ip']}:{net_cfg['port']} topic='{net_cfg['topic']}'")
    else:
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            print(f"[ERROR] Cannot open camera {camera_id}")
            return

    baseline = extract_default_params()
    current = clamp_params(deepcopy(baseline))
    selected_idx = 0
    detector = build_detector(current)

    # 滑鼠點擊 Param Panel 選擇項目
    def on_panel_mouse(event, x, y, flags, userdata):
        nonlocal selected_idx
        if event == cv2.EVENT_LBUTTONDOWN:
            # 對應 render_param_panel 的行距
            start_y = 50
            line_h = 20
            idx = (y - start_y) // line_h
            if 0 <= idx < len(PARAM_META):
                selected_idx = int(idx)

    while True:
        if net_sock is not None:
            frame = _recv_frame(net_sock)
            if frame is None:
                continue
        else:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        total = len(ids) if ids is not None else 0
        name_sel = PARAM_META[selected_idx][0]
        delta_sel = current.get(name_sel, 0) - baseline.get(name_sel, 0)
        disp = _draw_detection(frame, corners, ids, "Manual Mode", float(total), param_label=f"{name_sel} {_fmt_delta(delta_sel)}")
        panel = render_param_panel(current, baseline, selected_idx)
        cv2.imshow("Aruco Param Test", disp)
        cv2.imshow("Param Panel", panel)
        cv2.setMouseCallback("Param Panel", on_panel_mouse)
        raw_key = cv2.waitKeyEx(1)
        key = raw_key & 0xFF
        if key in (27, ord('q'), ord('Q')):
            break
        elif key == 82:  # up arrow
            selected_idx = (selected_idx - 1) % len(PARAM_META)
        elif key == 84:  # down arrow
            selected_idx = (selected_idx + 1) % len(PARAM_META)
        elif key in (43, 61):  # + or =
            name, p_type, base_step = PARAM_META[selected_idx]
            delta = base_step * step_scale
            current = apply_delta(current, name, delta, p_type)
            detector = build_detector(current)
        elif key == 45:  # -
            name, p_type, base_step = PARAM_META[selected_idx]
            delta = -base_step * step_scale
            current = apply_delta(current, name, delta, p_type)
            detector = build_detector(current)

    if cap is not None:
        cap.release()
    if net_sock is not None:
        net_sock.close()
    if net_ctx is not None:
        net_ctx.term()
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass

    print("\n[Manual Mode] Final parameters:")
    for k, v in current.items():
        print(f"  {k}: {v}")


def main():
    parser = argparse.ArgumentParser(description="Manual tuning of ArUco detector parameters (auto mode removed).")
    parser.add_argument("--camera", type=int, default=None, help="Camera index to open (overrides LOCAL_CAMERA_ID).")
    parser.add_argument("--net-port", type=int, default=None, help="Use network subscriber mode on this port (e.g., 5555).")
    parser.add_argument("--net-ip", type=str, default=None, help="Network subscriber IP (default from NET_IP).")
    parser.add_argument("--net-topic", type=str, default=None, help="Network topic to subscribe (default from NET_TOPIC).")
    parser.add_argument("--step-scale", type=float, default=1.0, help="Scale factor applied to each +/- adjustment step.")
    args = parser.parse_args()

    # 1) 程式碼內的預設；2) 參數覆蓋
    net_cfg = None
    use_net = USE_NET_MODE
    if args.net_port is not None:
        use_net = True
    if use_net:
        net_cfg = {
            "ip": args.net_ip if args.net_ip is not None else NET_IP,
            "port": args.net_port if args.net_port is not None else NET_PORT,
            "topic": args.net_topic if args.net_topic is not None else NET_TOPIC,
        }
    cam_id = args.camera if args.camera is not None else LOCAL_CAMERA_ID
    run_manual(cam_id if not use_net else None, step_scale=args.step_scale, net_cfg=net_cfg)


if __name__ == "__main__":
    main()
