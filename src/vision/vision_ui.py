import json
import os
import tkinter as tk

import numpy as np

try:
    from config.constants import ROOT_DIR, CONFIG_DIR, CALIB_DIR
except ModuleNotFoundError:
    import sys
    from pathlib import Path

    ROOT_DIR = Path(__file__).resolve().parent.parent
    root_str = str(ROOT_DIR)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    from config.constants import ROOT_DIR, CONFIG_DIR, CALIB_DIR

_imageprocess = None
_parent_window = None
var = None
camera_setting = None
color_sle_setting = None

color_selection_setting = "default"
camera_selection_setting = "right"
color = 0
fieldchoose = 0


def init(imageprocess, parent_window):
    global _imageprocess, _parent_window, var, camera_setting, color_sle_setting
    _imageprocess = imageprocess
    _parent_window = parent_window
    var = tk.StringVar(master=parent_window)
    camera_setting = tk.StringVar(master=parent_window)
    color_sle_setting = tk.StringVar(master=parent_window)


def _resource_path(filename: str):
    candidates = [
        CONFIG_DIR / filename,
        CALIB_DIR / filename,
        ROOT_DIR / filename,
    ]
    for path in candidates:
        if path.exists():
            return str(path)
    return filename


def load_hsv_tables():
    if _imageprocess is None:
        return
    try:
        with open(_resource_path("HSVmax.txt"), "r") as f_max:
            elements = [line.strip().split() for line in f_max]
        _imageprocess.HSVcolormax = np.array(elements).astype(np.int64)

        with open(_resource_path("HSVmin.txt"), "r") as f_min:
            elements = [line.strip().split() for line in f_min]
        _imageprocess.HSVcolormin = np.array(elements).astype(np.int64)
    except IOError:
        print("沒找到HSV的指定值,將使用預設值")


def imagedetection():
    if _imageprocess is None:
        return

    def start_default(popup_obj=None):
        if popup_obj is not None:
            popup_obj.destroy()
        try:
            default_path = _resource_path("calibration_points.json")
            hsv_path = _resource_path("calibration_hsv.json")

            def _valid(pts):
                return isinstance(pts, list) and len(pts) == 4 and all(isinstance(p, (list, tuple)) and len(p) == 2 for p in pts)

            def _valid_hsv(data):
                return isinstance(data, dict) and "lower" in data and "upper" in data

            saved0 = saved2 = None
            saved_hsv = None
            if os.path.exists(default_path):
                try:
                    with open(default_path, "r", encoding="utf-8") as f:
                        saved = json.load(f)
                        saved0 = saved.get("points_cam0")
                        saved2 = saved.get("points_cam2")
                except Exception as err:
                    print(f"[calibration] load default failed: {err}")
            if os.path.exists(hsv_path):
                try:
                    with open(hsv_path, "r", encoding="utf-8") as f:
                        saved_hsv = json.load(f)
                except Exception as err:
                    print(f"[calibration] load hsv failed: {err}")
            if hasattr(_imageprocess, "points_cam0"):
                _imageprocess.points_cam0.clear()
                if _valid(saved0):
                    _imageprocess.points_cam0.extend([tuple(p) for p in saved0])
                else:
                    _imageprocess.points_cam0.extend([(25, 30), (453, 189), (464, 506), (21, 636)])
            if hasattr(_imageprocess, "points_cam2"):
                _imageprocess.points_cam2.clear()
                if _valid(saved2):
                    _imageprocess.points_cam2.extend([tuple(p) for p in saved2])
                else:
                    _imageprocess.points_cam2.extend([(73, 157), (74, 463), (477, 618), (471, 2)])
            if hasattr(_imageprocess, "start_image_thread"):
                _imageprocess.start_image_thread(show_windows=True)
            else:
                _imageprocess.main()
            try:
                if _valid_hsv(saved_hsv) and hasattr(_imageprocess, "my_ball"):
                    lower = np.array(saved_hsv["lower"], dtype=np.uint8)
                    upper = np.array(saved_hsv["upper"], dtype=np.uint8)
                    _imageprocess.my_ball.hsv_range = (lower, upper)
                    radius_val = saved_hsv.get("manual_radius_px")
                    if radius_val:
                        _imageprocess.my_ball.manual_radius_px = int(radius_val)
                        print(f"[calibration] Applied default HSV mask (radius={radius_val})")
                    else:
                        print("[calibration] Applied default HSV mask")
            except Exception as err:
                print(f"[calibration] apply hsv failed: {err}")
        except AttributeError:
            _imageprocess.imagedetection(fieldchoose)

    def start_manual(popup_obj=None):
        if popup_obj is not None:
            popup_obj.destroy()
        try:
            if hasattr(_imageprocess, "points_cam0"):
                _imageprocess.points_cam0.clear()
            if hasattr(_imageprocess, "points_cam2"):
                _imageprocess.points_cam2.clear()
            if hasattr(_imageprocess, "start_image_thread"):
                _imageprocess.start_image_thread(show_windows=True)
            else:
                _imageprocess.main()
        except AttributeError:
            _imageprocess.imagedetection(fieldchoose)

    def start_background(popup_obj):
        popup_obj.destroy()
        try:
            if hasattr(_imageprocess, "start_image_thread"):
                _imageprocess.start_image_thread(show_windows=False)
            elif hasattr(_imageprocess, "image_result"):
                _imageprocess.image_result()
            else:
                print("[imagedetection] background mode not supported by current backend")
        except Exception as err:
            print(f"[imagedetection] switch to background failed: {err}")

    def restart_detection(popup_obj):
        popup_obj.destroy()
        try:
            if hasattr(_imageprocess, "start_image_thread"):
                _imageprocess.start_image_thread(show_windows=True)
            else:
                _imageprocess.main()
        except Exception as err:
            print(f"[imagedetection] restart failed: {err}")

    def stop_detection(popup_obj):
        popup_obj.destroy()
        try:
            if hasattr(_imageprocess, "stop_image_thread"):
                _imageprocess.stop_image_thread()
            elif hasattr(_imageprocess, "_running"):
                _imageprocess._running = False
        except Exception as err:
            print(f"[imagedetection] stop failed: {err}")

    if getattr(_imageprocess, "_running", False):
        running_popup = tk.Toplevel(_parent_window) if _parent_window is not None else tk.Toplevel()
        running_popup.title("影像辨識執行中")
        tk.Label(running_popup, text="影像辨識已啟動，請選擇操作").pack(pady=8, padx=12)
        btn_frame = tk.Frame(running_popup)
        btn_frame.pack(pady=6)
        tk.Button(
            btn_frame,
            text="背景執行",
            width=10,
            command=lambda: start_background(running_popup),
        ).pack(side="left", padx=4)
        tk.Button(
            btn_frame,
            text="重新啟動",
            width=10,
            command=lambda: restart_detection(running_popup),
        ).pack(side="left", padx=4)
        tk.Button(
            btn_frame,
            text="關閉",
            width=10,
            command=lambda: stop_detection(running_popup),
        ).pack(side="left", padx=4)
        return

    popup = tk.Toplevel(_parent_window) if _parent_window is not None else tk.Toplevel()
    popup.title("影像辨識選擇")
    tk.Label(popup, text="選擇校正方式").pack(pady=8, padx=12)
    btn_frame = tk.Frame(popup)
    btn_frame.pack(pady=6)
    tk.Button(btn_frame, text="預設", width=10, command=lambda: start_default(popup)).pack(side="left", padx=6)
    tk.Button(btn_frame, text="手動校正", width=10, command=lambda: start_manual(popup)).pack(side="left", padx=6)


def HSVdetection():
    if _imageprocess is None:
        return
    popup = tk.Toplevel(_parent_window) if _parent_window is not None else tk.Toplevel()
    popup.title("選擇攝影機方向")
    popup.geometry("250x120")

    tk.Label(popup, text="請選擇攝影機方向:").pack(pady=10)

    def select_camera(camera_selection_setting):
        popup.destroy()
        _imageprocess.HSVdetection(color, camera_selection_setting)

    tk.Button(popup, text="Left", width=10, command=lambda: select_camera("left")).pack(pady=5)
    tk.Button(popup, text="Right", width=10, command=lambda: select_camera("right")).pack()


def ColorMask():
    if _imageprocess is None:
        return
    print("Use ", color_selection_setting, " setting", camera_selection_setting, " camera")
    _imageprocess.ColorMask(color, color_selection_setting=color_selection_setting, camera_selection_setting=camera_selection_setting)


def fieldlocation():
    global fieldchoose, color
    color = 0
    fieldchoose = 0
    if var is not None:
        var.set("邊界12個點")


def fieldcenter():
    global fieldchoose, color
    color = 0
    fieldchoose = 1
    if var is not None:
        var.set("中心點")


def fieldpenalty():
    global fieldchoose, color
    color = 0
    fieldchoose = 2
    if var is not None:
        var.set("罰球點")


def red():
    global color
    color = "red"
    if var is not None:
        var.set("red")


def green():
    global color
    color = "green"
    if var is not None:
        var.set("green")


def blue():
    global color
    color = "blue"
    if var is not None:
        var.set("blue")


def purple():
    global color
    color = "purple"
    if var is not None:
        var.set("purple")


def pink():
    global color
    color = "pink"
    if var is not None:
        var.set("pink")


def orange():
    global color
    color = "orange"
    if var is not None:
        var.set("orange")


def yellow():
    global color
    color = "yellow"
    if var is not None:
        var.set("yellow")


def clear():
    global color
    if _imageprocess is not None:
        _imageprocess.clear(color)
    if var is not None:
        var.set("clear")
    color = ""


def reset():
    global color
    if _imageprocess is not None:
        _imageprocess.reset()
    if var is not None:
        var.set("")
    color = 0


def showall():
    global color
    color = 9
    if var is not None:
        var.set("show all")


def color_setting_default():
    global color_selection_setting
    color_selection_setting = "default"
    if color_sle_setting is not None:
        color_sle_setting.set("default")


def color_setting_custom():
    global color_selection_setting
    color_selection_setting = "custom"
    if color_sle_setting is not None:
        color_sle_setting.set("custom")


def camera_setting_left():
    global camera_selection_setting
    camera_selection_setting = "left"
    if camera_setting is not None:
        camera_setting.set("left")


def camera_setting_right():
    global camera_selection_setting
    camera_selection_setting = "right"
    if camera_setting is not None:
        camera_setting.set("right")
