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
_ch3_ball_var = None
_ch3_cmd_var = None
_ch3_log_text = None
_ch3_status_label = None
_ch3_btn_start = None
_ch3_btn_pause = None
_ch3_btn_reselect = None
_ch3_map_canvas = None
_ch3_robot_pos_vars = []
_ch3_robot_ang_vars = []
_ch3_robot_vec_vars = []
_ch3_robot_cmd_vars = []
_ch3_thread = None
_ch3_running = False
_ch3_thread_id = None
_ch3_latest_ball = None
_ch3_latest_cmd = None
_ch3_latest_robot_data = []
_ch3_latest_team = []
_ch3_latest_dirs = []
_ch3_latest_oppo = []
_ch3_latest_ball_xy = []
_ch3_robot_last_seen_ts = [0.0, 0.0, 0.0]
_ch3_log_queue = []
_ch3_ui_tick_started = False

_CH3_ABSENCE_TIMEOUT_SEC = 2.5

_ch3_attacker_mod = None
_ch3_keeper_mod = None


def _show_error_safe(title: str, text: str):
    def _do():
        try:
            messagebox.showerror(title, text)
        except Exception as err:
            print(f"[challenge3][popup] {title}: {text} | err={err}")

    target = _window if _window is not None else tk._default_root
    if threading.current_thread() is threading.main_thread():
        _do()
        return
    if target is not None:
        try:
            target.after(0, _do)
            return
        except Exception:
            pass
    _do()


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


def _coerce_xy(value):
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return float(value[0]), float(value[1])
        except Exception:
            return None
    return None


def _coerce_vec(value):
    xy = _coerce_xy(value)
    if xy is None:
        return 0.0, 0.0
    return xy


def _status_style(enabled):
    if enabled:
        return "狀態: 傳送中", "#0b6b3a", "#d9f7e8"
    return "狀態: 暫停 (不送出)", "#7a1e1e", "#fde8e8"


def _update_status_badge():
    if _ch3_status_var is None or _ch3_status_label is None:
        return
    text, fg, bg = _status_style(_ch3_send_enabled)
    _ch3_status_var.set(text)
    try:
        _ch3_status_label.configure(fg=fg, bg=bg)
    except Exception:
        pass


def _draw_ch3_map():
    if _ch3_map_canvas is None:
        return
    try:
        canvas = _ch3_map_canvas
        canvas.delete("all")
        w = int(canvas.winfo_width()) or 420
        h = int(canvas.winfo_height()) or 280
        margin = 18

        field_pts = _field if isinstance(_field, list) and len(_field) >= 4 else [[0, 0], [360, 0], [360, 260], [0, 260]]
        xs = [float(p[0]) for p in field_pts]
        ys = [float(p[1]) for p in field_pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)

        # Reserve a left-side "bench" zone for robots that are not currently detected.
        bench_w = max(100.0, min(170.0, w * 0.2))
        field_left = margin + bench_w + 10.0
        draw_w = max(1.0, w - field_left - margin)
        draw_h = max(1.0, h - margin * 2)
        scale = min(draw_w / span_x, draw_h / span_y)
        # Keep field centered in the preview area.
        offset_x = field_left + (draw_w - span_x * scale) / 2.0
        offset_y = (h - span_y * scale) / 2.0

        def map_xy(x, y):
            px = offset_x + (x - min_x) * scale
            py = offset_y + (y - min_y) * scale
            return px, py

        field_top_left = map_xy(min_x, min_y)
        field_bottom_right = map_xy(max_x, max_y)

        # Light board area to improve contrast.
        canvas.create_rectangle(
            margin / 2,
            margin / 2,
            w - margin / 2,
            h - margin / 2,
            fill="#eef2f7",
            outline="",
        )

        bench_x0 = margin / 2
        bench_y0 = margin / 2
        bench_x1 = field_left - 6.0
        bench_y1 = h - margin / 2
        canvas.create_rectangle(bench_x0, bench_y0, bench_x1, bench_y1, fill="#e2e8f0", outline="#94a3b8", width=1)
        canvas.create_text((bench_x0 + bench_x1) / 2, bench_y0 + 18, text="未上場", fill="#334155", font=("Microsoft JhengHei UI", 10, "bold"))

        poly = []
        for p in field_pts:
            mp = map_xy(float(p[0]), float(p[1]))
            poly.extend([mp[0], mp[1]])
        canvas.create_polygon(poly, fill="#ecfff1", outline="#2d8a52", width=2)

        if isinstance(_center, (list, tuple)) and len(_center) >= 2:
            cx, cy = map_xy(float(_center[0]), float(_center[1]))
            canvas.create_line(cx, field_top_left[1], cx, field_bottom_right[1], fill="#8eb89f", dash=(4, 3))
            circle_r = max(10.0, 16.0 * scale / 2.0)
            canvas.create_oval(cx - circle_r, cy - circle_r, cx + circle_r, cy + circle_r, outline="#8eb89f")

        def _draw_id_badge(x, y, text, accent):
            text_id = canvas.create_text(
                x,
                y,
                text=text,
                anchor="sw",
                fill="#0f172a",
                font=("Consolas", 11, "bold"),
            )
            bbox = canvas.bbox(text_id)
            if not bbox:
                return
            x1, y1, x2, y2 = bbox
            pad = 2
            rect_id = canvas.create_rectangle(
                x1 - pad,
                y1 - pad,
                x2 + pad,
                y2 + pad,
                fill="#ffffff",
                outline=accent,
                width=1,
            )
            canvas.tag_raise(text_id, rect_id)

        def _draw_robot_marker(pos_xy, dir_vec, fill_color, outline_color, label, label_color):
            px, py = map_xy(pos_xy[0], pos_xy[1])
            vx, vy = _coerce_vec(dir_vec)
            mag = math.hypot(vx, vy)
            if mag < 1e-6:
                # Fallback to old point style when direction is unavailable.
                canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill=fill_color, outline=outline_color, width=2)
                _draw_id_badge(px + 12, py - 8, label, label_color)
                return

            ux, uy = vx / mag, vy / mag
            rx, ry = uy, -ux  # right-hand unit vector

            # Use monitor-like robot body proportions (depth x width) with visible minimum size.
            half_depth = max(5.0, (17.5 * scale) / 2.0)
            half_width = max(7.0, (29.0 * scale) / 2.0)

            fl = (px + ux * half_depth + rx * half_width, py + uy * half_depth + ry * half_width)
            fr = (px + ux * half_depth - rx * half_width, py + uy * half_depth - ry * half_width)
            rr = (px - ux * half_depth - rx * half_width, py - uy * half_depth - ry * half_width)
            rl = (px - ux * half_depth + rx * half_width, py - uy * half_depth + ry * half_width)

            canvas.create_polygon(
                [fl[0], fl[1], fr[0], fr[1], rr[0], rr[1], rl[0], rl[1]],
                fill=fill_color,
                outline=outline_color,
                width=2,
            )

            # Front edge highlight and heading arrow.
            canvas.create_line(fl[0], fl[1], fr[0], fr[1], fill="#f8fafc", width=2)
            arrow_len = max(12.0, 18.0 * scale)
            canvas.create_line(
                px,
                py,
                px + ux * arrow_len,
                py + uy * arrow_len,
                fill=outline_color,
                width=2,
                arrow=tk.LAST,
                arrowshape=(10, 12, 4),
            )
            xs = [fl[0], fr[0], rr[0], rl[0]]
            ys = [fl[1], fr[1], rr[1], rl[1]]
            label_x = max(xs) + 8
            label_y = min(ys) - 4
            _draw_id_badge(label_x, label_y, label, label_color)

        # Opponents
        for idx, opp in enumerate(_ch3_latest_oppo[:3]):
            pos = _coerce_xy(opp)
            if pos is None:
                continue
            _draw_robot_marker(pos, [0.0, 0.0], "#f59e0b", "#92400e", f"O{idx}", "#92400e")

        # Team robots with heading
        inactive_indices = []
        for idx in range(3):
            data = _ch3_latest_robot_data[idx] if idx < len(_ch3_latest_robot_data) else {}
            if not bool(data.get("active", False)):
                inactive_indices.append(idx)

        for order, idx in enumerate(inactive_indices):
            bench_cx = (bench_x0 + bench_x1) / 2
            bench_cy = bench_y0 + 48 + order * 62
            if bench_cy > bench_y1 - 20:
                bench_cy = bench_y1 - 20
            # Use field inverse mapping so marker helper can still be reused.
            fake_x = ((bench_cx - offset_x) / scale) + min_x
            fake_y = ((bench_cy - offset_y) / scale) + min_y
            _draw_robot_marker([fake_x, fake_y], [0.0, 0.0], "#93c5fd", "#1e3a8a", f"R{idx}", "#1e3a8a")

        for idx in range(min(3, len(_ch3_latest_team))):
            data = _ch3_latest_robot_data[idx] if idx < len(_ch3_latest_robot_data) else {}
            if not bool(data.get("active", False)):
                continue
            pos = _coerce_xy(_ch3_latest_team[idx])
            if pos is None:
                continue
            vec = _coerce_vec(_ch3_latest_dirs[idx] if idx < len(_ch3_latest_dirs) else [0, 0])
            _draw_robot_marker(pos, vec, "#60a5fa", "#1e3a8a", f"R{idx}", "#1e3a8a")

        # Ball
        bpos = _coerce_xy(_ch3_latest_ball_xy)
        if bpos is not None:
            bx, by = map_xy(bpos[0], bpos[1])
            ball_r = max(4.0, 4.0 * scale)
            canvas.create_oval(bx - ball_r, by - ball_r, bx + ball_r, by + ball_r, fill="#e11d48", outline="#9f1239", width=2)
            canvas.create_text(bx + 12, by + 10, text="Ball", anchor="w", fill="#9f1239", font=("Consolas", 10, "bold"))
    except Exception:
        pass


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
    global _ch3_monitor, _ch3_status_var, _ch3_ball_var, _ch3_cmd_var, _ch3_log_text
    global _ch3_status_label, _ch3_btn_start, _ch3_btn_pause, _ch3_btn_reselect, _ch3_map_canvas
    global _ch3_robot_pos_vars, _ch3_robot_ang_vars, _ch3_robot_vec_vars, _ch3_robot_cmd_vars
    if _ch3_monitor and _ch3_monitor.winfo_exists():
        _start_ch3_ui_tick()
        return
    parent = _window if _window is not None else None
    _ch3_monitor = tk.Toplevel(parent)
    _ch3_monitor.title("Challenge3 Monitor")
    _ch3_monitor.geometry("920x760")
    _ch3_monitor.minsize(860, 700)
    _ch3_monitor.configure(bg="#f3f7fb")
    _ch3_status_var = tk.StringVar(value="狀態: 暫停 (不送出)")
    _ch3_ball_var = tk.StringVar(value="Ball: X:-  Y:-")
    _ch3_cmd_var = tk.StringVar(value="Cmd: N1 N1 N1")

    top = tk.Frame(_ch3_monitor, bg="#f3f7fb")
    top.pack(fill="x", padx=14, pady=(12, 8))

    _ch3_status_label = tk.Label(
        top,
        textvariable=_ch3_status_var,
        anchor="w",
        font=("Microsoft JhengHei UI", 12, "bold"),
        padx=10,
        pady=8,
    )
    _ch3_status_label.pack(fill="x", side="left", expand=True)

    ctrl = tk.Frame(top, bg="#f3f7fb")
    ctrl.pack(side="right")
    _ch3_btn_start = tk.Button(
        ctrl,
        text="開始",
        width=8,
        command=lambda: _ch3_set_send(True),
        bg="#0f766e",
        fg="white",
        activebackground="#115e59",
        activeforeground="white",
        relief="flat",
        font=("Microsoft JhengHei UI", 10, "bold"),
    )
    _ch3_btn_start.pack(side="left", padx=(0, 8))
    _ch3_btn_pause = tk.Button(
        ctrl,
        text="暫停",
        width=8,
        command=lambda: _ch3_set_send(False),
        bg="#b45309",
        fg="white",
        activebackground="#92400e",
        activeforeground="white",
        relief="flat",
        font=("Microsoft JhengHei UI", 10, "bold"),
    )
    _ch3_btn_pause.pack(side="left")
    _ch3_btn_reselect = tk.Button(
        ctrl,
        text="重選策略",
        width=10,
        command=_reselect_ch3_strategies_from_monitor,
        bg="#334155",
        fg="white",
        activebackground="#1e293b",
        activeforeground="white",
        relief="flat",
        font=("Microsoft JhengHei UI", 10, "bold"),
    )
    _ch3_btn_reselect.pack(side="left", padx=(8, 0))

    info_row = tk.Frame(_ch3_monitor, bg="#f3f7fb")
    info_row.pack(fill="x", padx=14, pady=(0, 8))

    ball_card = tk.Frame(info_row, bg="#e0f2fe", bd=1, relief="solid")
    ball_card.pack(side="left", fill="x", expand=True, padx=(0, 6))
    tk.Label(ball_card, text="Ball", bg="#e0f2fe", fg="#0c4a6e", font=("Consolas", 10, "bold")).pack(anchor="w", padx=10, pady=(6, 0))
    tk.Label(ball_card, textvariable=_ch3_ball_var, bg="#e0f2fe", fg="#0f172a", font=("Consolas", 11)).pack(anchor="w", padx=10, pady=(0, 6))

    cmd_card = tk.Frame(info_row, bg="#ede9fe", bd=1, relief="solid")
    cmd_card.pack(side="left", fill="x", expand=True, padx=(6, 0))
    tk.Label(cmd_card, text="Command", bg="#ede9fe", fg="#4c1d95", font=("Consolas", 10, "bold")).pack(anchor="w", padx=10, pady=(6, 0))
    tk.Label(cmd_card, textvariable=_ch3_cmd_var, bg="#ede9fe", fg="#0f172a", font=("Consolas", 11)).pack(anchor="w", padx=10, pady=(0, 6))

    record_row = tk.Frame(_ch3_monitor, bg="#f3f7fb")
    record_row.pack(fill="x", padx=14, pady=(0, 8))
    if _ch3_recorder is not None:
        _ch3_recorder.add_buttons(record_row, _ch3_monitor)

    mid = tk.Frame(_ch3_monitor, bg="#f3f7fb")
    mid.pack(fill="both", expand=True, padx=14, pady=(2, 8))

    map_panel = tk.Frame(mid, bg="white", bd=1, relief="solid")
    map_panel.pack(side="left", fill="both", expand=True, padx=(0, 8))
    tk.Label(map_panel, text="即時場地視圖", anchor="w", bg="white", fg="#334155", font=("Microsoft JhengHei UI", 10, "bold")).pack(fill="x", padx=10, pady=(8, 4))
    _ch3_map_canvas = tk.Canvas(map_panel, bg="#f8fafc", highlightthickness=0)
    _ch3_map_canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    robot_panel = tk.Frame(mid, bg="#f3f7fb")
    robot_panel.pack(side="right", fill="y")
    tk.Label(robot_panel, text="機器人狀態", anchor="w", bg="#f3f7fb", fg="#334155", font=("Microsoft JhengHei UI", 10, "bold")).pack(fill="x", pady=(0, 4))

    _ch3_robot_pos_vars = []
    _ch3_robot_ang_vars = []
    _ch3_robot_vec_vars = []
    _ch3_robot_cmd_vars = []
    for idx in range(3):
        card = tk.Frame(robot_panel, bg="#ffffff", bd=1, relief="solid")
        card.pack(fill="x", pady=4)
        tk.Label(card, text=f"Robot{idx}", bg="#ffffff", fg="#1e3a8a", font=("Consolas", 10, "bold")).pack(anchor="w", padx=8, pady=(6, 2))
        pos_var = tk.StringVar(value="X:-  Y:-")
        ang_var = tk.StringVar(value="Ang:-")
        vec_var = tk.StringVar(value="Vec:(-, -)")
        cmd_var = tk.StringVar(value="Cmd:-")
        _ch3_robot_pos_vars.append(pos_var)
        _ch3_robot_ang_vars.append(ang_var)
        _ch3_robot_vec_vars.append(vec_var)
        _ch3_robot_cmd_vars.append(cmd_var)
        tk.Label(card, textvariable=pos_var, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8)
        tk.Label(card, textvariable=ang_var, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8)
        tk.Label(card, textvariable=vec_var, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8)
        tk.Label(card, textvariable=cmd_var, bg="#ffffff", fg="#0f172a", font=("Consolas", 10)).pack(anchor="w", padx=8, pady=(0, 6))

    log_frame = tk.Frame(_ch3_monitor, bg="#f3f7fb")
    log_frame.pack(fill="both", expand=True, padx=14, pady=(0, 12))
    tk.Label(log_frame, text="Log", anchor="w", bg="#f3f7fb", fg="#334155", font=("Microsoft JhengHei UI", 10, "bold")).pack(fill="x")
    _ch3_log_text = tk.Text(
        log_frame,
        height=10,
        wrap="word",
        bg="#0b1220",
        fg="#e2e8f0",
        insertbackground="#e2e8f0",
        font=("Consolas", 10),
        relief="flat",
    )
    _ch3_log_text.pack(side="left", fill="both", expand=True)
    log_scroll = tk.Scrollbar(log_frame, command=_ch3_log_text.yview)
    log_scroll.pack(side="right", fill="y")
    _ch3_log_text.configure(yscrollcommand=log_scroll.set, state="disabled")
    _update_status_badge()

    def on_close():
        _ch3_set_send(False)
        try:
            _ch3_monitor.destroy()
        except Exception:
            pass
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
    cmd_tokens = []
    if _ch3_latest_cmd:
        cmd_text = _ch3_latest_cmd.replace("Cmd:", "", 1).strip()
        cmd_tokens = [token for token in cmd_text.split() if token]
    if _ch3_latest_robot_data:
        for idx in range(min(3, len(_ch3_latest_robot_data))):
            data = _ch3_latest_robot_data[idx]
            x = data.get("x")
            y = data.get("y")
            ang = data.get("ang")
            vx = data.get("vx", 0.0)
            vy = data.get("vy", 0.0)
            cmd = cmd_tokens[idx] if idx < len(cmd_tokens) else "-"
            if idx < len(_ch3_robot_pos_vars):
                if x is None or y is None:
                    _ch3_robot_pos_vars[idx].set("X:-  Y:-")
                else:
                    _ch3_robot_pos_vars[idx].set(f"X:{x:.1f}  Y:{y:.1f}")
            if idx < len(_ch3_robot_ang_vars):
                _ch3_robot_ang_vars[idx].set("Ang:-" if ang is None else f"Ang:{ang:.1f}")
            if idx < len(_ch3_robot_vec_vars):
                _ch3_robot_vec_vars[idx].set(f"Vec:({vx:.2f}, {vy:.2f})")
            if idx < len(_ch3_robot_cmd_vars):
                _ch3_robot_cmd_vars[idx].set(f"Cmd:{cmd}")
    else:
        for idx in range(min(3, len(_ch3_robot_pos_vars))):
            _ch3_robot_pos_vars[idx].set("X:-  Y:-")
        for idx in range(min(3, len(_ch3_robot_ang_vars))):
            _ch3_robot_ang_vars[idx].set("Ang:-")
        for idx in range(min(3, len(_ch3_robot_vec_vars))):
            _ch3_robot_vec_vars[idx].set("Vec:(-, -)")
        for idx in range(min(3, len(_ch3_robot_cmd_vars))):
            _ch3_robot_cmd_vars[idx].set("Cmd:-")

    _draw_ch3_map()
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

    def _apply_ui():
        _update_status_badge()
        try:
            if _ch3_btn_start is not None:
                _ch3_btn_start.configure(relief="sunken" if _ch3_send_enabled else "flat")
            if _ch3_btn_pause is not None:
                _ch3_btn_pause.configure(relief="flat" if _ch3_send_enabled else "sunken")
        except Exception:
            pass

    if threading.current_thread() is threading.main_thread():
        _apply_ui()
        return

    try:
        target = _window if _window is not None else _ch3_monitor
        if target is not None:
            target.after(0, _apply_ui)
    except Exception:
        pass


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


def _strategy_mod_name(mod, fallback):
    name = getattr(mod, "__name__", fallback) if mod is not None else fallback
    if not isinstance(name, str) or not name:
        return fallback
    return name.split(".")[-1]


def _select_ch3_strategies():
    global _ch3_attacker_mod, _ch3_keeper_mod
    if _strategy_ui is None or _window is None:
        print("[challenge3] strategy UI not initialized")
        return False

    atk_default = _strategy_mod_name(_ch3_attacker_mod, "challenge3_forward")
    def_default = _strategy_mod_name(_ch3_keeper_mod, "challenge3_keeper_final_v2")
    atk_name = _strategy_ui.choose_strategy(_window, _src_dir, "選擇1/2號(進攻)策略", atk_default)
    def_name = _strategy_ui.choose_strategy(_window, _src_dir, "選擇3號(防守)策略", def_default)
    try:
        _ch3_attacker_mod = _load_strategy_module(atk_name)
        _ch3_keeper_mod = _load_strategy_module(def_name)
        print(f"[challenge3] 使用策略: attacker={atk_name}, keeper={def_name}")
        append_log(f"[challenge3] 使用策略: attacker={atk_name}, keeper={def_name}\n")
        return True
    except Exception as err:
        messagebox.showerror("策略載入失敗", f"{err}")
        return False


def _reselect_ch3_strategies_from_monitor():
    global _ch3_running, _ch3_thread
    was_running = bool(_ch3_thread and _ch3_thread.is_alive())
    was_sending = _ch3_send_enabled
    if not _select_ch3_strategies():
        return

    if not was_running:
        return

    _ch3_set_send(False)
    _ch3_running = False
    worker = _ch3_thread
    if worker is not None and worker.is_alive():
        worker.join(timeout=3.0)
    if worker is not None and worker.is_alive():
        append_log("[challenge3] 尚在停止舊策略執行緒，請稍後再按一次重選策略\n")
        messagebox.showwarning("重選策略", "舊策略執行緒尚未停止，請稍後再試")
        return

    challenge3()
    if was_sending:
        try:
            target = _window if _window is not None else _ch3_monitor
            if target is not None:
                target.after(400, lambda: _ch3_set_send(True))
        except Exception:
            _ch3_set_send(True)


def challenge3_select_and_start():
    if not _select_ch3_strategies():
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
    global _ch3_running, _ch3_thread_id, _ch3_latest_ball, _ch3_latest_cmd, _ch3_latest_robot_data
    global _ch3_latest_team, _ch3_latest_dirs, _ch3_latest_oppo, _ch3_latest_ball_xy, _ch3_robot_last_seen_ts
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
        _show_error_safe("影像偵測", "請先按『影像偵測』完成雙相機四點校正，再啟動挑戰")
        _ch3_running = False
        return

    import builtins
    _orig_print = builtins.print
    _ch3_thread_id = threading.get_ident()
    _ch3_latest_ball = None
    _ch3_latest_cmd = None
    _ch3_latest_robot_data = []
    _ch3_latest_team = []
    _ch3_latest_dirs = []
    _ch3_latest_oppo = []
    _ch3_latest_ball_xy = []
    _ch3_robot_last_seen_ts = [0.0, 0.0, 0.0]

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
                    _ch3_latest_ball_xy = [bx, by]
                else:
                    _ch3_latest_ball = "Ball: X:- Y:-"
                    _ch3_latest_ball_xy = []

                team_state = []
                dir_state = []
                for idx in range(3):
                    loc = location[idx] if idx < len(location) else None
                    ang = angle[idx] if idx < len(angle) else None
                    xy = _coerce_xy(loc)
                    vec_xy = _coerce_xy(ang)
                    team_state.append([xy[0], xy[1]] if xy is not None else None)
                    dir_state.append([vec_xy[0], vec_xy[1]] if vec_xy is not None else [0.0, 0.0])
                _ch3_latest_team = team_state
                _ch3_latest_dirs = dir_state
                oppo_state = []
                for p in real_oppoP:
                    xy = _coerce_xy(p)
                    if xy is not None:
                        oppo_state.append([xy[0], xy[1]])
                _ch3_latest_oppo = oppo_state

                now_ts = time.time()
                robot_data = []
                for idx in range(3):
                    loc = location[idx] if idx < len(location) else None
                    ang_val = angle[idx] if idx < len(angle) else 0.0
                    vx = vy = 0.0
                    ang_base = None
                    if isinstance(ang_val, (list, tuple)) and len(ang_val) >= 2:
                        vx, vy = ang_val[0], ang_val[1]
                        ang_base = math.degrees(math.atan2(vy, vx)) if (vx or vy) else 0.0
                    else:
                        if isinstance(ang_val, (int, float)):
                            ang_base = float(ang_val)
                            rad = math.radians(ang_base)
                            vx, vy = math.cos(rad), math.sin(rad)
                        else:
                            ang_base = None
                    xy = _coerce_xy(loc)
                    vec_mag = math.hypot(float(vx), float(vy))
                    seen_signal = (xy is not None) and (vec_mag > 0.15)
                    if seen_signal:
                        _ch3_robot_last_seen_ts[idx] = now_ts
                    last_seen = _ch3_robot_last_seen_ts[idx]
                    active = bool(last_seen > 0.0 and (now_ts - last_seen) <= _CH3_ABSENCE_TIMEOUT_SEC)
                    robot_data.append({
                        "x": xy[0] if xy is not None else None,
                        "y": xy[1] if xy is not None else None,
                        "ang": ang_base,
                        "vx": float(vx),
                        "vy": float(vy),
                        "active": active,
                    })
                _ch3_latest_robot_data = robot_data

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
