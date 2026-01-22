import threading
import time
import math
import os
import tkinter as tk
from tkinter import messagebox

_window = None
_imageprocess = None
_field = None
_center = None
_src_dir = None
_nrf_ui = None
_strategy_ui = None
_ch3_recorder = None

_ch3_last_log_ts = 0.0
_ch3_send_enabled = False
_ch3_monitor = None
_ch3_status_var = None
_ch3_robot_vars = []
_ch3_ball_var = None
_ch3_cmd_var = None
_ch3_log_text = None
_ch3_thread = None
_ch3_running = False
_ch3_thread_id = None
_ch3_latest_ball = None
_ch3_latest_cmd = None
_ch3_latest_robot_texts = []
_ch3_log_queue = []
_ch3_ui_tick_started = False

_ch3_attacker_mod = None
_ch3_keeper_mod = None


def init(parent_window, imageprocess, field, center, src_dir, nrf_ui, strategy_ui, attacker_mod=None, keeper_mod=None):
    global _window, _imageprocess, _field, _center, _src_dir, _nrf_ui, _strategy_ui
    global _ch3_attacker_mod, _ch3_keeper_mod
    _window = parent_window
    _imageprocess = imageprocess
    _field = field
    _center = center
    _src_dir = src_dir
    _nrf_ui = nrf_ui
    _strategy_ui = strategy_ui
    _ch3_attacker_mod = attacker_mod
    _ch3_keeper_mod = keeper_mod


def set_recorder(ch3_recorder):
    global _ch3_recorder
    _ch3_recorder = ch3_recorder


def get_attacker():
    return _ch3_attacker_mod


def get_latest_cmd():
    return _ch3_latest_cmd


def append_log(text):
    if text:
        _ch3_log_queue.append(text)


def _display_available():
    if os.name == "nt":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _ensure_ch3_monitor():
    if not _display_available():
        print("[Challenge3 Monitor] DISPLAY not available; GUI disabled on this platform/session.")
        return
    if threading.current_thread() is not threading.main_thread():
        try:
            if _window is not None:
                _window.after(0, _ensure_ch3_monitor)
        except Exception:
            print("[Challenge3 Monitor] Cannot schedule window creation (no Tk root?)")
        return
    global _ch3_monitor, _ch3_status_var, _ch3_send_enabled, _ch3_robot_vars, _ch3_ball_var, _ch3_cmd_var, _ch3_log_text
    if _ch3_monitor and _ch3_monitor.winfo_exists():
        _start_ch3_ui_tick()
        return
    parent = _window if _window is not None else None
    _ch3_monitor = tk.Toplevel(parent)
    _ch3_monitor.title("Challenge3 Monitor")
    _ch3_monitor.geometry("460x640")
    _ch3_status_var = tk.StringVar(value="狀態: 暫停 (不送出)")

    tk.Label(_ch3_monitor, textvariable=_ch3_status_var, anchor="w").pack(fill="x", padx=10, pady=10)

    btn_frame = tk.Frame(_ch3_monitor)
    btn_frame.pack(pady=8)
    tk.Button(btn_frame, text="開始", width=8, command=lambda: _ch3_set_send(True)).pack(side="left", padx=6)
    tk.Button(btn_frame, text="暫停", width=8, command=lambda: _ch3_set_send(False)).pack(side="left", padx=6)

    record_frame = tk.Frame(_ch3_monitor)
    record_frame.pack(pady=6)
    if _ch3_recorder is not None:
        _ch3_recorder.add_buttons(record_frame, _ch3_monitor)

    _ch3_ball_var = tk.StringVar(value="Ball: X:- Y:-")
    tk.Label(_ch3_monitor, textvariable=_ch3_ball_var, anchor="w").pack(fill="x", padx=10, pady=4)

    _ch3_cmd_var = tk.StringVar(value="Cmd: N1 N1 N1")
    tk.Label(_ch3_monitor, textvariable=_ch3_cmd_var, anchor="w").pack(fill="x", padx=10, pady=4)

    robots_frame = tk.Frame(_ch3_monitor)
    robots_frame.pack(fill="both", expand=True, padx=10, pady=4)
    _ch3_robot_vars = []
    for idx in range(3):
        var = tk.StringVar(value=f"Robot{idx}\nX: - Y: -\nAng: -\nVec: (-,-)")
        _ch3_robot_vars.append(var)
        tk.Label(robots_frame, textvariable=var, anchor="w", justify="left").pack(fill="x", pady=2)

    log_frame = tk.Frame(_ch3_monitor)
    log_frame.pack(fill="both", expand=True, padx=10, pady=6)
    tk.Label(log_frame, text="Log", anchor="w").pack(fill="x")
    _ch3_log_text = tk.Text(log_frame, height=10, wrap="word")
    _ch3_log_text.pack(side="left", fill="both", expand=True)
    log_scroll = tk.Scrollbar(log_frame, command=_ch3_log_text.yview)
    log_scroll.pack(side="right", fill="y")
    _ch3_log_text.configure(yscrollcommand=log_scroll.set, state="disabled")

    def on_close():
        _ch3_set_send(False)
    _ch3_monitor.protocol("WM_DELETE_WINDOW", on_close)

    _start_ch3_ui_tick()


def _ch3_ui_tick():
    global _ch3_ui_tick_started
    if _ch3_monitor is None or not _ch3_monitor.winfo_exists():
        _ch3_ui_tick_started = False
        return
    if _ch3_ball_var is not None and _ch3_latest_ball is not None:
        _ch3_ball_var.set(_ch3_latest_ball)
    if _ch3_cmd_var is not None and _ch3_latest_cmd is not None:
        _ch3_cmd_var.set(_ch3_latest_cmd)
    if _ch3_robot_vars and _ch3_latest_robot_texts:
        for idx in range(min(len(_ch3_robot_vars), len(_ch3_latest_robot_texts))):
            _ch3_robot_vars[idx].set(_ch3_latest_robot_texts[idx])
    if _ch3_log_text is not None and _ch3_log_queue:
        try:
            _ch3_log_text.configure(state="normal")
            while _ch3_log_queue:
                _ch3_log_text.insert("end", _ch3_log_queue.pop(0))
            _ch3_log_text.see("end")
            _ch3_log_text.configure(state="disabled")
        except Exception:
            pass
    try:
        _ch3_monitor.after(200, _ch3_ui_tick)
    except Exception:
        _ch3_ui_tick_started = False


def _start_ch3_ui_tick():
    global _ch3_ui_tick_started
    if _ch3_ui_tick_started:
        return
    if _ch3_monitor is None or not _ch3_monitor.winfo_exists():
        return
    _ch3_ui_tick_started = True
    try:
        _ch3_monitor.after(0, _ch3_ui_tick)
    except Exception:
        _ch3_ui_tick_started = False


def _ch3_set_send(enabled: bool):
    global _ch3_send_enabled
    _ch3_send_enabled = enabled
    if _ch3_status_var is not None:
        _ch3_status_var.set("狀態: 傳送中" if enabled else "狀態: 暫停 (不送出)")


def _log_ch3_state(angle, location, real_oppoP, ballcenter):
    global _ch3_last_log_ts
    now = time.time()
    if now - _ch3_last_log_ts < 1.0:
        return
    _ch3_last_log_ts = now

    def fmt_positions(items):
        return "; ".join(
            f"{idx}:{[round(coord,1) for coord in (pos if isinstance(pos, (list, tuple)) else [pos])]}"
            for idx, pos in enumerate(items)
        ) if items else "[]"

    def fmt_angles(items):
        if not items:
            return []
        formatted = []
        for ang in items:
            if isinstance(ang, (int, float)):
                formatted.append(round(float(ang), 1))
            elif isinstance(ang, (list, tuple)) and len(ang) >= 2:
                try:
                    formatted.append([round(float(ang[0]), 1), round(float(ang[1]), 1)])
                except Exception:
                    formatted.append(ang)
            else:
                formatted.append(ang)
        return formatted

    angles_fmt = fmt_angles(angle)
    ball_fmt = [round(coord, 1) for coord in ballcenter] if ballcenter else []

    print(
        f"[Challenge3] pos={fmt_positions(location)} | ang={angles_fmt} | oppo={fmt_positions(real_oppoP)} | ball={ball_fmt}"
    )


def _load_strategy_module(name):
    import importlib
    return importlib.import_module(f"strategies.{name}")


def challenge3_select_and_start():
    global _ch3_attacker_mod, _ch3_keeper_mod
    if _strategy_ui is None or _window is None:
        print("[challenge3] strategy UI not initialized")
        return
    atk_default = getattr(_ch3_attacker_mod, "__name__", "challenge3_forward")
    def_default = getattr(_ch3_keeper_mod, "__name__", "challenge3_keeper_final_v2")
    atk_name = _strategy_ui.choose_strategy(_window, _src_dir, "選擇1/2號(進攻)策略", atk_default)
    def_name = _strategy_ui.choose_strategy(_window, _src_dir, "選擇3號(防守)策略", def_default)
    try:
        _ch3_attacker_mod = _load_strategy_module(atk_name)
        _ch3_keeper_mod = _load_strategy_module(def_name)
        print(f"[challenge3] 使用策略: attacker={atk_name}, keeper={def_name}")
    except Exception as err:
        messagebox.showerror("策略載入失敗", f"{err}")
        return
    challenge3()


def challenge3():
    global _ch3_thread, _ch3_running
    if _ch3_running:
        print("[challenge3] already running")
        return
    _ensure_ch3_monitor()
    _ch3_thread = threading.Thread(target=_challenge3_worker, daemon=True)
    _ch3_thread.start()


def return_sent_cmd3_1(send_data, sent):
    try:
        if _ch3_attacker_mod is not None:
            _ch3_attacker_mod.get_sent_cmd(send_data[0:3], sent)
    except AttributeError:
        pass


def return_sent_cmd3_2(send_data, sent):
    try:
        if _ch3_keeper_mod is not None:
            _ch3_keeper_mod.get_sent_cmd(send_data[0:3], sent)
    except AttributeError:
        pass


def _challenge3_worker():
    global _ch3_running, _ch3_thread_id, _ch3_latest_ball, _ch3_latest_cmd, _ch3_latest_robot_texts
    attacker = _ch3_attacker_mod
    keeper = _ch3_keeper_mod
    _ch3_running = True
    if _imageprocess is None:
        print("[challenge3] imageprocess not initialized")
        _ch3_running = False
        return
    if _field is None or _center is None:
        print("[challenge3] field/center not initialized")
        _ch3_running = False
        return
    if not getattr(_imageprocess, "_running", False):
        messagebox.showerror("影像偵測", "請先按『影像偵測』完成雙相機四點校正，再啟動挑戰")
        _ch3_running = False
        return

    import builtins
    _orig_print = builtins.print
    _ch3_thread_id = threading.get_ident()
    _ch3_latest_ball = None
    _ch3_latest_cmd = None
    _ch3_latest_robot_texts = []

    def _ch3_print(*args, **kwargs):
        sep = kwargs.get("sep", " ")
        end = kwargs.get("end", "\n")
        msg = sep.join(str(a) for a in args) + end
        _orig_print(*args, **kwargs)
        if threading.get_ident() == _ch3_thread_id:
            append_log(msg)

    builtins.print = _ch3_print

    print("[challenge3] start")
    _ensure_ch3_monitor()

    for _ in range(10):
        if _imageprocess.team_pos != [] and _imageprocess.ball_center != []:
            break
        print("Waiting for image data...")
        time.sleep(1)
    print("[challenge3] image data ready, entering loop")

    if hasattr(_imageprocess, "SUPPRESS_BALL_MASK_LOGS"):
        _imageprocess.SUPPRESS_BALL_MASK_LOGS = True

    print("Initial data:")
    print("ballcenter:", _imageprocess.ball_center)
    print("location:", _imageprocess.team_pos)
    print("angle:", _imageprocess.team_degree)
    print("oppoP:", _imageprocess.oppo_pos)

    try:
        if attacker is not None:
            attacker.strategy_update_field(1, _field, _center, 0, 0, 0, 0, 0, 0)
            attacker.Initialize()
        if keeper is not None:
            keeper.strategy_update_field(1, _field, _center, 0, 0, 0, 0, 0, 0)
            keeper.Initialize()
    except Exception:
        pass

    _ch3_set_send(False)
    count = 0
    device = _nrf_ui.get_controller_device(allow_offline=True) if _nrf_ui is not None else None
    if device is None:
        print("[challenge3] no NRF device, running in offline mode (commands not sent)")
    try:
        while _ch3_running:
            try:
                location = _imageprocess.team_pos.copy()
                angle = _imageprocess.team_degree.copy()
                oppoP = _imageprocess.oppo_pos.copy()
                ballcenter = _imageprocess.ball_center.copy()

                real_oppoP = []
                for i in range(len(oppoP)):
                    if _field[0][0] < oppoP[i][0] < _field[4][0]:
                        if _field[0][1] < oppoP[i][1] < _field[7][1]:
                            real_oppoP.append([oppoP[i][0], oppoP[i][1]])
                while len(real_oppoP) < 3:
                    real_oppoP.append([0, 0])
                real_oppoP.sort(key=lambda pos: pos[0])

                _log_ch3_state(angle, location, real_oppoP, ballcenter)
                if ballcenter:
                    bx = ballcenter[0] if len(ballcenter) > 0 else 0
                    by = ballcenter[1] if len(ballcenter) > 1 else 0
                    try:
                        _ch3_latest_ball = f"Ball: X:{bx:.1f} Y:{by:.1f}"
                    except Exception:
                        _ch3_latest_ball = f"Ball: X:{bx} Y:{by}"
                else:
                    _ch3_latest_ball = "Ball: X:- Y:-"

                robot_texts = []
                for idx in range(3):
                    loc = location[idx] if idx < len(location) else None
                    ang_val = angle[idx] if idx < len(angle) else 0.0
                    vx = vy = 0.0
                    if isinstance(ang_val, (list, tuple)) and len(ang_val) >= 2:
                        vx, vy = ang_val[0], ang_val[1]
                        ang_base = math.degrees(math.atan2(vy, vx)) if (vx or vy) else 0.0
                    else:
                        ang_base = float(ang_val) if isinstance(ang_val, (int, float)) else 0.0
                    if loc:
                        text = (
                            f"Robot{idx}\n"
                            f"X:{loc[0]:.1f} Y:{loc[1]:.1f}\n"
                            f"Ang:{ang_base:.1f}\n"
                            f"Vec:({vx:.2f},{vy:.2f})"
                        )
                    else:
                        text = f"Robot{idx}\nX:- Y:-\nAng:-\nVec:(-,-)"
                    robot_texts.append(text)
                _ch3_latest_robot_texts = robot_texts

                move_data1 = ["N1", "N1", "N1"]
                move_data2 = ["N1", "N1", "N1"]
                move_data = ["N1", "N1", "N1"]

                if angle[0] != [] and location != [] and angle[1] != [] and angle[2] != [] and real_oppoP != [] and ballcenter != []:
                    count += 1
                    if attacker is not None:
                        attacker.Update_Robo_Info(angle, location, real_oppoP, ballcenter)
                    if keeper is not None:
                        keeper.Update_Robo_Info(angle, location, real_oppoP, ballcenter)

                    if attacker is not None:
                        print("--------------\nkicker strategy\n--------------")
                        move_data1 = attacker.strategy()
                    time.sleep(1)
                    if keeper is not None:
                        print("--------------\nkeeper strategy\n--------------")
                        move_data2 = keeper.strategy()

                    move_data = [move_data1[0], move_data2[1], move_data1[2]]
                    print(count, move_data)

                print("move_data = ", move_data)
                try:
                    _ch3_latest_cmd = f"Cmd: {' '.join(move_data)}"
                except Exception:
                    _ch3_latest_cmd = "Cmd: -"

                return_sent_cmd3_1(move_data1, True)
                return_sent_cmd3_2(move_data2, True)
                if _ch3_send_enabled and _nrf_ui is not None:
                    _nrf_ui.send_cmd_safe(move_data, device, context="challenge3", allow_popup=False)
                time.sleep(1.8)
            except Exception as err:
                append_log(f"[challenge3][error] {err}\n")
                time.sleep(0.5)
    finally:
        _ch3_running = False
        if hasattr(_imageprocess, "SUPPRESS_BALL_MASK_LOGS"):
            _imageprocess.SUPPRESS_BALL_MASK_LOGS = False
        builtins.print = _orig_print
        _ch3_thread_id = None
