import tkinter as tk
from tkinter import messagebox
import threading

try:
    from nrf import nrf_controller as controller
    from nrf import nrf_controller as nrf_scan
except ModuleNotFoundError:
    import sys
    from pathlib import Path

    ROOT_DIR = Path(__file__).resolve().parent.parent
    root_str = str(ROOT_DIR)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    from nrf import nrf_controller as controller
    from nrf import nrf_controller as nrf_scan

controller_device = None
controller_port_info = None
controller_status_var = None
_nrf_cfg_loaded = False
_controller_cfg_loaded = False
_popup_parent = None


def _show_popup(kind: str, title: str, text: str):
    def _do_show():
        try:
            fn = getattr(messagebox, kind, None)
            if callable(fn):
                fn(title, text)
            else:
                messagebox.showerror(title, text)
        except Exception as err:
            print(f"[NRF][popup:{kind}] {title}: {text} | err={err}")

    target = _popup_parent if _popup_parent is not None else tk._default_root
    if threading.current_thread() is threading.main_thread():
        _do_show()
        return
    if target is not None:
        try:
            target.after(0, _do_show)
            return
        except Exception:
            pass
    _do_show()


def _set_controller_status(text):
    if controller_status_var is not None:
        controller_status_var.set(text)
    else:
        print(text)


def _ensure_nrf_config():
    global _nrf_cfg_loaded
    if _nrf_cfg_loaded:
        return True
    try:
        nrf_scan.read_config()
        _nrf_cfg_loaded = True
        return True
    except Exception as err:
        try:
            _show_popup("showerror", "NRF 設定錯誤", str(err))
        except Exception:
            print(f"NRF 設定錯誤: {err}")
        return False


def _ensure_controller_config():
    global _controller_cfg_loaded
    if _controller_cfg_loaded:
        return True
    try:
        controller.read_config()
        _controller_cfg_loaded = True
        return True
    except Exception as err:
        try:
            _show_popup("showerror", "NRF 設定錯誤", str(err))
        except Exception:
            print(f"NRF 設定錯誤: {err}")
        return False


def get_controller_device(allow_offline: bool = False):
    """Return existing NRF serial device; auto connect if needed."""
    global controller_device, controller_port_info
    if controller_device is not None:
        try:
            if controller_device.is_open:
                return controller_device
        except AttributeError:
            controller_device = None

    if not _ensure_controller_config():
        return None

    ports = nrf_scan.get_device()
    if not ports:
        _set_controller_status("NRF: 未找到序列埠")
        return None

    for port in ports:
        try:
            state, device, baud = nrf_scan.open_device(port)
            if state and device is not None:
                controller_device = device
                controller_port_info = f"{port} @ {baud}"
                _set_controller_status(f"NRF: {controller_port_info}")
                try:
                    controller.download_cfg(controller_device)
                except Exception as err:
                    print(f"[NRF] download cfg failed: {err}")
                return controller_device
        except Exception as err:
            print(f"[NRF] auto-open {port} failed: {err}")
            continue

    _set_controller_status("NRF: 未連線")
    if allow_offline:
        return None
    return None


def scan_serial_ports(parent_window=None):
    """Search available ports via nrf_controller and allow manual selection."""
    global controller_device, controller_port_info
    if not _ensure_nrf_config():
        return

    ports = nrf_scan.get_device()
    if not ports:
        _show_popup("showwarning", "NRF", "找不到任何序列埠，請確認設備已連接")
        return

    usable = []
    for port in ports:
        try:
            state, device, baud = nrf_scan.open_device(port)
        except Exception as err:
            print(f"[NRF] 開啟 {port} 失敗: {err}")
            continue
        if state:
            usable.append((port, baud, device))
        else:
            if device is not None and device.is_open:
                device.close()

    if not usable:
        _show_popup("showwarning", "NRF", "沒有可使用的序列埠")
        return

    selector = tk.Toplevel(parent_window) if parent_window is not None else tk.Toplevel()
    selector.title("選擇 NRF 序列埠")
    selector.geometry("320x200")
    tk.Label(selector, text="請選擇要使用的序列埠:").pack(pady=5)

    def choose(idx):
        global controller_device, controller_port_info
        chosen_port, chosen_baud, chosen_device = usable[idx]

        if controller_device is not None and controller_device is not chosen_device:
            try:
                if controller_device.is_open:
                    controller_device.close()
            except Exception:
                pass

        for j, (_, _, dev) in enumerate(usable):
            if j != idx and dev is not None and dev.is_open:
                dev.close()

        controller_device = chosen_device
        controller_port_info = f"{chosen_port} @ {chosen_baud}"
        _set_controller_status(f"NRF: {controller_port_info}")

        try:
            if _ensure_controller_config():
                controller.download_cfg(controller_device)
        except Exception as err:
            _show_popup("showwarning", "NRF", f"下載設定失敗: {err}")

        selector.destroy()

    for i, (port, baud, _) in enumerate(usable):
        btn = tk.Button(selector, text=f"{port} @ {baud}", command=lambda idx=i: choose(idx))
        btn.pack(fill="x", padx=10, pady=2)

    def on_close():
        for _, _, dev in usable:
            if dev is not None and dev.is_open and dev is not controller_device:
                dev.close()
        selector.destroy()

    selector.protocol("WM_DELETE_WINDOW", on_close)


def send_cmd_safe(move_data, device=None, context="", allow_popup=True):
    """Send NRF command with validation and error message."""
    dev = device or get_controller_device()
    if dev is None:
        if allow_popup:
            _show_popup("showerror", "NRF", "無法連線到 NRF 裝置，請先執行搜尋序列埠")
        print(f"[NRF][{context}] no device")
        return False
    if not isinstance(move_data, (list, tuple)) or len(move_data) < 3:
        _show_popup("showerror", "NRF", f"{context or '指令'} 格式錯誤，需三個字串")
        print(f"[NRF][{context}] invalid move_data: {move_data}")
        return False
    try:
        print(f"[NRF][{context}] sending {move_data}")
        controller.main_procedure(dev, move_data)
        return True
    except Exception as err:
        _show_popup("showerror", "NRF", f"{context or '送出指令'}失敗: {err}")
        print(f"[NRF][{context}] send error: {err}")
        return False


def build_nrf_ui(frame, parent_window=None):
    global controller_status_var, _popup_parent
    _popup_parent = parent_window
    controller_status_var = tk.StringVar(value="NRF: 未連線")
    nrf_status_label = tk.Label(
        frame,
        textvariable=controller_status_var,
        bg="black",
        fg="white",
        font=("Arial", 12),
        width=20,
        height=2,
    )
    button_scan = tk.Button(frame, text="搜尋序列埠", fg="blue", command=lambda: scan_serial_ports(parent_window))
    button_send = tk.Button(
        frame,
        text="送w1指令",
        fg="black",
        command=lambda: send_cmd_safe(["w1", "w2", "w9"], context="fixed_button"),
    )
    return nrf_status_label, button_scan, button_send
