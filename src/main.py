import threading
import tkinter as tk
from tkinter import messagebox
import os


from vision import imageprocess as imageprocess
from simulator import simulator as sim_view
from recording import recording_ui
from strategies.ui import strategy_ui
from strategies.ui import challenge_ui
from strategies import challenge3_forward as cha_3_1
from strategies import challenge3_keeper_final_v2 as cha_3_2
from config.constants import FIELD, CENTER, DATA_DIR
from nrf import nrf_ui
from vision import vision_ui

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



window = tk.Tk()  # 建立視窗window
window.title('main UI')  # 給視窗的視覺化起名字
window.geometry("420x240")  # 精簡介面
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
# Monitor 開啟簡化版 Simulator 視窗（無 dist/ang/向量箭頭）
button_sim = tk.Button(control_frame, text='Monitor', fg='black', command=open_monitor)
# Simulator 開啟完整即時模擬畫面（含 overlay）
button_sim2 = tk.Button(control_frame, text='Simulator', fg='black', command=open_simulator)

nrf_status_label, button23, button24 = nrf_ui.build_nrf_ui(nrf_frame, parent_window=window)

# 排版
button3.pack(side=tk.LEFT, padx=5)
button4.pack(side=tk.LEFT, padx=5)
button_sim.pack(side=tk.LEFT, padx=5)
button_sim2.pack(side=tk.LEFT, padx=5)
nrf_status_label.pack(side=tk.LEFT, padx=5)
button23.pack(side=tk.LEFT, padx=5)
button24.pack(side=tk.LEFT, padx=5)

window.mainloop()  # 主視窗迴圈顯示
