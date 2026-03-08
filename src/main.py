"""

Main.py is a port to connect different .py Module, click the Module you want to modify directly below.

"""

""" Vison Module
AruCo/Ball tracking and localization 
"""
from vision import imageprocess as imageprocess
from vision import vision_ui


""" Simulator View Module 
Present real-time Robot/Ball position and direction, it doesn't do any simulatation. 
"""
from simulator import simulator as sim_view
from simulator import strategy_ui


""" 
Recording Module 
1. Record Robot/Ball position and direction, strategy commmand into .csv 
2. Record camera view 
"""
from recording import recording_ui


""" 
Strategy Module 
import attack and defense strategy. 
Note: you can choose strategy directly in Main UI without changing the import file below.
"""
from strategies.ui import challenge_ui
from strategies import challenge3_forward as cha_3_1
from strategies import challenge3_keeper_final_v2 as cha_3_2


""" 
Constant
Change FIELD, CENTER, DATA_DIR here
"""
from config.constants import FIELD, CENTER, DATA_DIR


""" 
Nrf conrol Module
1. Search the radio port
2. Send command to control Robot
"""
from nrf import nrf_ui


import threading
import tkinter as tk
from tkinter import messagebox
import os


field = FIELD


import sys
from pathlib import Path

sys.modules.setdefault("main", sys.modules.get(__name__))
sys.modules.setdefault("src.main", sys.modules.get(__name__))

ROOT_DIR = Path(__file__).resolve().parent  # src/
SRC_DIR = ROOT_DIR
for extra in [ROOT_DIR, SRC_DIR / "vision", SRC_DIR / "strategies", SRC_DIR / "nrf", SRC_DIR / "utils", SRC_DIR / "recording"]:
    extra_str = str(extra)
    if extra_str not in sys.path:
        sys.path.insert(0, extra_str)
    


real_real_oppoP = []
location = []
_sim_snap_thread = None
_sim_snap_running = False



def _display_available():
    """Check if GUI display is available (Linux headless fallback)."""
    if os.name == "nt":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def open_simulator():
    """Open simulator view (with overlay)."""
    sim_view.open_realtime_view(
        provider=imageprocess,
        overlay_enabled=True,
        title="Simulator View",
        strategy_mod=cha_3_1,
        tk_parent=window,
    )


def open_monitor():
    """Open simplified monitor view (no dist/angle/arrow overlay)."""
    sim_view.open_realtime_view(
        provider=imageprocess,
        overlay_enabled=False,
        title="Monitor View",
        strategy_mod=None,
        tk_parent=window,
    )


def start_simulator_snapshot():
    """Start simulator (manual trigger) with current camera snapshot; press 's' in window to start simulation."""
    if not _display_available():
        print("[simulator snapshot] DISPLAY not available; skip opening simulator window.")
        return
    global _sim_snap_thread, _sim_snap_running
    if _sim_snap_running:
        print("[simulator snapshot] already running")
        return
    if not getattr(imageprocess, "_running", False):
        messagebox.showerror("影像偵測", "請先按『影像偵測』完成校正，再開啟 Simulator")
        return

    _sim_snap_running = True  # 先佔位，避免連點
    def _snap_loop():
        global _sim_snap_running
        try:
            team = imageprocess.team_pos.copy() if hasattr(imageprocess, "team_pos") else []
            dirs = imageprocess.team_degree.copy() if hasattr(imageprocess, "team_degree") else []
            ball = imageprocess.ball_center.copy() if hasattr(imageprocess, "ball_center") else []
            sim_view.run_snapshot(ball=ball, team_pos=team, team_vec=dirs)
        finally:
            _sim_snap_running = False

    _sim_snap_thread = threading.Thread(target=_snap_loop, daemon=True)
    _sim_snap_thread.start()

 
def send_fixed_command(cmd_list):
    """Send a fixed triple command via NRF."""
    nrf_ui.send_cmd_safe(cmd_list, context="fixed_button")


def _parse_robot_id_text(text):
    raw = (text or "").replace("，", ",").replace(" ", ",")
    tokens = [tok.strip() for tok in raw.split(",") if tok.strip()]
    if not tokens:
        raise ValueError("每一台 Robot 至少要有 1 個 ArUco ID")
    ids = []
    for tok in tokens:
        try:
            value = int(tok)
        except Exception as err:
            raise ValueError(f"無效 ID: {tok}") from err
        if value < 0:
            raise ValueError(f"ID 不可為負值: {value}")
        ids.append(value)
    return ids


def open_robot_id_settings():
    """Open runtime robot-id/filter settings for imageprocess."""
    if not hasattr(imageprocess, "get_robot_tracking_options") or not hasattr(imageprocess, "set_robot_tracking_options"):
        messagebox.showerror("Robot ID 設定", "目前 imageprocess 版本不支援此功能")
        return

    try:
        cfg = imageprocess.get_robot_tracking_options()
    except Exception as err:
        messagebox.showerror("Robot ID 設定", f"讀取設定失敗: {err}")
        return

    id_groups = cfg.get("robot_id_groups", [[0], [10], [20]])
    while len(id_groups) < 3:
        id_groups.append([len(id_groups)])
    filter_default = bool(cfg.get("filter_non_robot_aruco", True))

    popup = tk.Toplevel(window)
    popup.title("Robot ID 設定")
    popup.geometry("420x280")

    tk.Label(
        popup,
        text="每台 Robot 的 ArUco ID（以逗號分隔）",
        anchor="w",
        justify="left",
    ).pack(fill="x", padx=12, pady=(10, 6))

    entry_vars = []
    for rid in range(3):
        row = tk.Frame(popup)
        row.pack(fill="x", padx=12, pady=4)
        tk.Label(row, text=f"Robot{rid}", width=8, anchor="w").pack(side=tk.LEFT)
        var = tk.StringVar(value=",".join(str(v) for v in id_groups[rid]))
        entry_vars.append(var)
        tk.Entry(row, textvariable=var).pack(side=tk.LEFT, fill="x", expand=True)

    filter_var = tk.BooleanVar(value=filter_default)
    tk.Checkbutton(
        popup,
        text="在 imageprocess 過濾非 Robot ID 的 ArUco（建議開啟）",
        variable=filter_var,
        onvalue=True,
        offvalue=False,
        anchor="w",
        justify="left",
    ).pack(fill="x", padx=12, pady=(10, 6))

    hint = tk.Label(
        popup,
        text="套用後若影像辨識正在執行，會自動重啟並立即生效。",
        anchor="w",
        fg="gray30",
    )
    hint.pack(fill="x", padx=12, pady=(0, 8))

    def apply_settings(close_after=False):
        try:
            groups = [_parse_robot_id_text(v.get()) for v in entry_vars]
            imageprocess.set_robot_tracking_options(
                robot_id_groups=groups,
                filter_non_robot_aruco=filter_var.get(),
            )
            restarted = False
            if hasattr(imageprocess, "get_runtime_state") and hasattr(imageprocess, "start_image_thread"):
                state = imageprocess.get_runtime_state()
                if state.get("running"):
                    imageprocess.start_image_thread(show_windows=bool(state.get("show_windows", False)))
                    restarted = True
        except Exception as err:
            messagebox.showerror("Robot ID 設定", f"套用失敗: {err}")
            return

        if restarted:
            messagebox.showinfo("Robot ID 設定", "設定已套用，已重新啟動影像辨識")
        else:
            messagebox.showinfo("Robot ID 設定", "設定已套用")
        if close_after:
            popup.destroy()

    btns = tk.Frame(popup)
    btns.pack(pady=8)
    tk.Button(btns, text="套用", width=10, command=lambda: apply_settings(False)).pack(side=tk.LEFT, padx=5)
    tk.Button(btns, text="套用並關閉", width=12, command=lambda: apply_settings(True)).pack(side=tk.LEFT, padx=5)
    tk.Button(btns, text="取消", width=10, command=popup.destroy).pack(side=tk.LEFT, padx=5)



window = tk.Tk()  # 建立視窗window
window.title('main UI')  # 給視窗的視覺化起名字
vision_ui.init(imageprocess, window)
vision_ui.load_hsv_tables()
challenge_ui.init(
    parent_window=window,
    imageprocess=imageprocess,
    field=field,
    center=CENTER,
    src_dir=SRC_DIR,
    nrf_ui=nrf_ui,
    strategy_ui=strategy_ui,
    attacker_mod=cha_3_1,
    keeper_mod=cha_3_2,
)

ch3_recorder = recording_ui.build_ch3_recorder(
    parent_window=window,
    data_dir=DATA_DIR,
    imageprocess=imageprocess,
    get_attacker=challenge_ui.get_attacker,
    get_cmd=challenge_ui.get_latest_cmd,
    log_func=challenge_ui.append_log,
)
challenge_ui.set_recorder(ch3_recorder)

# 版面配置
challenge_frame = tk.Frame(window)
challenge_frame.pack(pady=8)
control_frame = tk.Frame(window)
control_frame.pack(pady=6)
nrf_frame = tk.Frame(window)
nrf_frame.pack(pady=6)

# 只保留：挑戰、影像偵測、NRF狀態/搜尋/送指令
button3 = tk.Button(challenge_frame, text='挑戰', fg='purple', command=challenge_ui.challenge3_select_and_start)
button4 = tk.Button(control_frame, text='影像偵測', fg='blue', command=vision_ui.imagedetection)
button_robot_ids = tk.Button(control_frame, text='Robot ID設定', fg='black', command=open_robot_id_settings)
# Monitor 開啟簡化版 Simulator 視窗（無 dist/ang/向量箭頭）
button_sim = tk.Button(control_frame, text='Monitor', fg='black', command=open_monitor)
# Simulator 開啟完整即時模擬畫面（含 overlay）
button_sim2 = tk.Button(control_frame, text='Simulator', fg='black', command=open_simulator)

nrf_status_label, button23, button24 = nrf_ui.build_nrf_ui(nrf_frame, parent_window=window)

# 排版
button3.pack(side=tk.LEFT, padx=5)
button4.pack(side=tk.LEFT, padx=5)
button_robot_ids.pack(side=tk.LEFT, padx=5)
button_sim.pack(side=tk.LEFT, padx=5)
button_sim2.pack(side=tk.LEFT, padx=5)
nrf_status_label.pack(side=tk.LEFT, padx=5)
button23.pack(side=tk.LEFT, padx=5)
button24.pack(side=tk.LEFT, padx=5)

# 依內容自動貼齊視窗大小，避免固定高度造成大片留白
window.update_idletasks()
fit_w = max(520, window.winfo_reqwidth() + 20)
fit_h = max(170, window.winfo_reqheight() + 16)
window.geometry(f"{fit_w}x{fit_h}")
window.minsize(fit_w, fit_h)

window.mainloop()  # 主視窗迴圈顯示
