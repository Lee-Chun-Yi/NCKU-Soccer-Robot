import csv
from datetime import datetime
from pathlib import Path
import math
import time
import tkinter as tk
from tkinter import messagebox

import cv2
import numpy as np

from simulator import vec_cal_func as vec


class Ch3Recorder:
    def __init__(
        self,
        parent_window,
        data_dir,
        imageprocess,
        get_attacker,
        get_cmd,
        log_func=None,
    ):
        self.parent_window = parent_window
        self.data_dir = Path(data_dir)
        self.imageprocess = imageprocess
        self.get_attacker = get_attacker
        self.get_cmd = get_cmd
        self.log_func = log_func

        self.monitor_window = None
        self.recording = False
        self.targets = []
        self.csv_fp = None
        self.csv_writer = None
        self.csv_path = None
        self.record_start_ts = None
        self.timer_id = None
        self.video_writers = {}
        self.video_sizes = {}
        self.video_paths = {}
        self.record_csv = True
        self.record_video = False
        self.btn_start = None
        self.btn_stop = None

    def set_monitor(self, monitor_window):
        self.monitor_window = monitor_window

    def add_buttons(self, parent_frame, monitor_window=None):
        if monitor_window is not None:
            self.monitor_window = monitor_window
        self.btn_start = tk.Button(parent_frame, text="開始錄製", width=10, command=self.start_recording)
        self.btn_start.pack(side="left", padx=6)
        self.btn_stop = tk.Button(parent_frame, text="結束錄製", width=10, command=self.stop_recording)
        self.btn_stop.pack(side="left", padx=6)
        self._set_button_state(self.recording)

    def _log(self, text):
        if not text:
            return
        if self.log_func:
            self.log_func(text)
        else:
            print(text, end="")

    def _normalize_angle_deg(self, angle):
        try:
            return (float(angle) + 180.0) % 360.0 - 180.0
        except Exception:
            return 0.0

    def _as_vec2(self, value):
        if value is None:
            return None
        try:
            if isinstance(value, (list, tuple, np.ndarray)) and len(value) >= 2:
                return [float(value[0]), float(value[1])]
        except Exception:
            return None
        return None

    def _vec_valid(self, vec2):
        if vec2 is None:
            return False
        try:
            return math.hypot(vec2[0], vec2[1]) > 1e-6
        except Exception:
            return False

    def _snapshot_state(self):
        team = []
        dirs = []
        ball = []
        try:
            state_lock = getattr(self.imageprocess, "_state_lock", None)
            if state_lock is not None:
                with state_lock:
                    team = [p[:] for p in getattr(self.imageprocess, "team_pos", [])]
                    dirs = [
                        d[:] if isinstance(d, (list, tuple, np.ndarray)) else d
                        for d in getattr(self.imageprocess, "team_degree", [])
                    ]
                    ball = list(getattr(self.imageprocess, "ball_center", []))
            else:
                team = [p[:] for p in getattr(self.imageprocess, "team_pos", [])]
                dirs = [
                    d[:] if isinstance(d, (list, tuple, np.ndarray)) else d
                    for d in getattr(self.imageprocess, "team_degree", [])
                ]
                ball = list(getattr(self.imageprocess, "ball_center", []))
        except Exception:
            team, dirs, ball = [], [], []

        team_list = []
        for p in team:
            v = self._as_vec2(p)
            if v is not None:
                team_list.append(v)

        dirs_list = []
        for d in dirs:
            v = self._as_vec2(d)
            if v is None and isinstance(d, (int, float)):
                rad = math.radians(float(d))
                v = [math.cos(rad), math.sin(rad)]
            if v is not None:
                dirs_list.append(v)
            else:
                dirs_list.append([None, None])

        ball_list = self._as_vec2(ball)
        return team_list, dirs_list, ball_list

    def _calc_abs_p_dir_tvec(self):
        attacker = self.get_attacker() if self.get_attacker else None
        if attacker is None:
            return None
        try:
            player_dir = self._as_vec2(getattr(attacker, "player_dir", None))
            if not self._vec_valid(player_dir):
                return None
            target_primary = None
            target_secondary = None
            if player_dir[1] < 0:
                target_primary = self._as_vec2(getattr(attacker, "target_vec_270", None))
                target_secondary = self._as_vec2(getattr(attacker, "target_vec_90", None))
            else:
                target_primary = self._as_vec2(getattr(attacker, "target_vec_90", None))
                target_secondary = self._as_vec2(getattr(attacker, "target_vec_270", None))
            if self._vec_valid(target_primary):
                target_vec = target_primary
            elif self._vec_valid(target_secondary):
                target_vec = target_secondary
            else:
                return None
            _, ang = vec.vector_angle(player_dir, target_vec)
            ang = self._normalize_angle_deg(ang)
            return abs(ang)
        except Exception:
            return None

    def _format_cmd_text(self, cmd_text):
        if not cmd_text:
            return ""
        cmd_text = cmd_text.strip()
        if cmd_text.startswith("Cmd:"):
            return cmd_text.replace("Cmd:", "", 1).strip()
        return cmd_text

    def _format_csv_num(self, value, ndigits=3):
        if value is None:
            return ""
        try:
            return round(float(value), ndigits)
        except Exception:
            return ""

    def _choose_record_targets(self):
        dlg = tk.Toplevel(self.monitor_window or self.parent_window)
        dlg.title("選擇錄製對象")
        tk.Label(dlg, text="請選擇要錄製的機器人").pack(padx=10, pady=8)

        var0 = tk.IntVar(value=1)
        var1 = tk.IntVar(value=1)
        tk.Checkbutton(dlg, text="Robot0", variable=var0).pack(anchor="w", padx=12)
        tk.Checkbutton(dlg, text="Robot1", variable=var1).pack(anchor="w", padx=12)

        tk.Label(dlg, text="錄製內容").pack(padx=10, pady=(6, 4), anchor="w")
        var_csv = tk.IntVar(value=1)
        var_video = tk.IntVar(value=0)
        tk.Checkbutton(dlg, text="CSV", variable=var_csv).pack(anchor="w", padx=12)
        tk.Checkbutton(dlg, text="畫面錄製", variable=var_video).pack(anchor="w", padx=12)

        choice = {"targets": None, "csv": True, "video": False}

        def on_ok():
            targets = []
            if var0.get():
                targets.append(0)
            if var1.get():
                targets.append(1)
            if not targets:
                messagebox.showwarning("錄製", "請至少選擇一個機器人")
                return
            record_csv = bool(var_csv.get())
            record_video = bool(var_video.get())
            if not record_csv and not record_video:
                messagebox.showwarning("錄製", "請至少勾選 CSV 或畫面錄製")
                return
            choice["targets"] = targets
            choice["csv"] = record_csv
            choice["video"] = record_video
            dlg.destroy()

        def on_cancel():
            choice["targets"] = None
            dlg.destroy()

        btn_frame = tk.Frame(dlg)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="確定", width=8, command=on_ok).pack(side="left", padx=6)
        tk.Button(btn_frame, text="取消", width=8, command=on_cancel).pack(side="left", padx=6)
        dlg.protocol("WM_DELETE_WINDOW", on_cancel)
        dlg.grab_set()
        dlg.focus_force()
        self.parent_window.wait_window(dlg)
        return choice["targets"], choice["csv"], choice["video"]

    def _set_button_state(self, recording):
        if self.btn_start is not None:
            self.btn_start.configure(state="disabled" if recording else "normal")
        if self.btn_stop is not None:
            self.btn_stop.configure(state="normal" if recording else "disabled")

    def _ensure_record_dir(self):
        try:
            self.data_dir.mkdir(parents=True, exist_ok=True)
        except Exception as err:
            messagebox.showerror("錄製", f"建立資料夾失敗: {err}")
            return False
        return True

    def start_recording(self):
        if self.recording:
            messagebox.showinfo("錄製", "已在錄製中")
            return
        targets, record_csv, record_video = self._choose_record_targets()
        if not targets:
            return
        if not record_csv and not record_video:
            return
        if not self._ensure_record_dir():
            return

        timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        fp = None
        writer = None
        csv_path = None
        if record_csv:
            csv_path = self.data_dir / f"{timestamp}.csv"
            try:
                fp = open(csv_path, "w", newline="", encoding="utf-8")
            except Exception as err:
                messagebox.showerror("錄製", f"建立 CSV 失敗: {err}")
                return

            writer = csv.writer(fp)
            writer.writerow([
                "timestamp", "elapsed_s", "robot_id",
                "ball_x", "ball_y",
                "robot_x", "robot_y",
                "robot_vx", "robot_vy",
                "robot_ball_dist",
                "abs_p_dir_tvec",
                "cmd",
            ])
            fp.flush()

        self.recording = True
        self.record_start_ts = time.time() if record_csv else None
        self.targets = targets
        self.csv_fp = fp
        self.csv_writer = writer
        self.csv_path = str(csv_path) if csv_path is not None else None
        self.video_writers = {}
        self.video_sizes = {}
        self.video_paths = {}
        self.record_csv = record_csv
        self.record_video = record_video
        if record_video:
            self.video_paths = {
                "cam0": self.data_dir / f"{timestamp}_cam0.avi",
                "cam1": self.data_dir / f"{timestamp}_cam1.avi",
            }

        if self.record_csv:
            self._log(f"[record] start csv={self.csv_path}\n")
        if self.record_video:
            self._log(f"[record] start video={timestamp}\n")
        self._set_button_state(True)
        self._record_tick()

    def _release_video_writers(self):
        for _, writer in list(self.video_writers.items()):
            try:
                writer.release()
            except Exception:
                pass
        self.video_writers = {}

    def stop_recording(self):
        if not self.recording:
            messagebox.showwarning("錄製", "尚未開始錄製")
            return
        self.recording = False
        if self.timer_id is not None:
            try:
                if self.monitor_window is not None:
                    self.monitor_window.after_cancel(self.timer_id)
                else:
                    self.parent_window.after_cancel(self.timer_id)
            except Exception:
                pass
        self.timer_id = None

        if self.csv_fp is not None:
            try:
                self.csv_fp.flush()
                self.csv_fp.close()
            except Exception:
                pass
        self.csv_fp = None
        self.csv_writer = None
        self._release_video_writers()

        self._log("[record] stop\n")
        self._set_button_state(False)
        self.targets = []
        self.csv_path = None
        self.record_start_ts = None
        self.record_csv = True
        self.record_video = False

    def _get_camera_frames(self):
        frame0 = getattr(self.imageprocess, "robot_cam0_preview", None)
        frame1 = getattr(self.imageprocess, "robot_cam2_preview", None)
        if frame0 is not None:
            try:
                frame0 = frame0.copy()
            except Exception:
                frame0 = None
        if frame1 is not None:
            try:
                frame1 = frame1.copy()
            except Exception:
                frame1 = None
        return frame0, frame1

    def _write_video_frame(self, cam_key, frame):
        if frame is None:
            return
        path = self.video_paths.get(cam_key)
        if path is None:
            return
        writer = self.video_writers.get(cam_key)
        if writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            writer = cv2.VideoWriter(str(path), fourcc, 2.0, (w, h))
            if not writer.isOpened():
                self._log(f"[record] video open failed: {path}\n")
                return
            self.video_writers[cam_key] = writer
            self.video_sizes[cam_key] = (w, h)
        else:
            w, h = self.video_sizes.get(cam_key, (frame.shape[1], frame.shape[0]))
            if frame.shape[1] != w or frame.shape[0] != h:
                frame = cv2.resize(frame, (w, h))
        try:
            writer.write(frame)
        except Exception:
            pass

    def _record_tick(self):
        if not self.recording:
            return
        try:
            if self.record_csv:
                team_list, dirs_list, ball_list = self._snapshot_state()
                cmd_text = self._format_cmd_text(self.get_cmd() if self.get_cmd else "")
                abs_p_dir_tvec = self._calc_abs_p_dir_tvec()
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                elapsed_s = None
                if self.record_start_ts is not None:
                    elapsed_s = max(0.0, time.time() - self.record_start_ts)
                for rid in self.targets:
                    robot_pos = team_list[rid] if rid < len(team_list) else None
                    robot_dir = dirs_list[rid] if rid < len(dirs_list) else None
                    dist = None
                    if robot_pos is not None and ball_list is not None:
                        try:
                            dist = math.hypot(ball_list[0] - robot_pos[0], ball_list[1] - robot_pos[1])
                        except Exception:
                            dist = None
                    row = [
                        timestamp,
                        self._format_csv_num(elapsed_s),
                        rid,
                        self._format_csv_num(ball_list[0] if ball_list else None),
                        self._format_csv_num(ball_list[1] if ball_list else None),
                        self._format_csv_num(robot_pos[0] if robot_pos else None),
                        self._format_csv_num(robot_pos[1] if robot_pos else None),
                        self._format_csv_num(robot_dir[0] if robot_dir else None),
                        self._format_csv_num(robot_dir[1] if robot_dir else None),
                        self._format_csv_num(dist),
                        self._format_csv_num(abs_p_dir_tvec),
                        cmd_text,
                    ]
                    if self.csv_writer is not None:
                        self.csv_writer.writerow(row)
                if self.csv_fp is not None:
                    self.csv_fp.flush()

            if self.record_video:
                frame0, frame1 = self._get_camera_frames()
                self._write_video_frame("cam0", frame0)
                self._write_video_frame("cam1", frame1)
        except Exception as err:
            self._log(f"[record][error] {err}\n")

        if self.monitor_window is not None and self.monitor_window.winfo_exists():
            self.timer_id = self.monitor_window.after(500, self._record_tick)
        else:
            self.timer_id = self.parent_window.after(500, self._record_tick)
