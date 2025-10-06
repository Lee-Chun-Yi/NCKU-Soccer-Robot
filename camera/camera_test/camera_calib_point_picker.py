
#如果報錯說 相機不能用 且 確定相機有插 ＝>記得把main_v5_nrfcontrol_mod2024的視窗關起來
import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox

clicked_points = []
frame_display = None
cap = None
camera_matrix = None

# 點擊事件，擷取四個點
def click_event(event, x, y, flags, param):
    global clicked_points, frame_display
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append([x, y])
        print(f"Point {len(clicked_points)}: ({x}, {y})")
        if frame_display is not None:
            cv2.circle(frame_display, (x, y), 6, (0, 255, 0), -1)
            cv2.putText(frame_display, f"{len(clicked_points)}", (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

# 開始擷取按鈕功能
def start_capture():
    global clicked_points, frame_display, cap, camera_matrix
    clicked_points = []

    cam_index = int(cam_var.get())
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        messagebox.showerror("Error", f"無法打開相機 index {cam_index}")
        return

    # 讀取對應相機的 calibration matrix（純矩陣格式）
    if cam_index == 2:
        camera_matrix = np.load("calibration_matrix_left.npy")
    elif cam_index == 0:
        camera_matrix = np.load("calibration_matrix_right.npy")
    else:
        messagebox.showerror("Error", "不支援的相機 index")
        cap.release()
        return

    # 嘗試設定解析度為 1920x1080
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # 嘗試讀第一幀確認解析度
    ret, frame = cap.read()
    if not ret:
        messagebox.showerror("Error", "無法讀取影像")
        cap.release()
        return

    print("實際解析度:", frame.shape[1], "x", frame.shape[0])

    # 建立視窗
    cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera View", 1920, 1080)
    cv2.setMouseCallback("Camera View", click_event)

    while True:
        ret, frame = cap.read()
        if not ret:
            messagebox.showerror("Error", "無法讀取影像")
            break
        cam_index = int(cam_var.get())

        frame_display = frame.copy()

        for i, pt in enumerate(clicked_points):
            cv2.circle(frame_display, tuple(pt), 6, (0, 255, 0), -1)
            cv2.putText(frame_display, f"{i+1}", (pt[0] + 10, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(frame_display, "ESC:退出 | s:儲存 | r:重設", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Camera View", frame_display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('r'):
            clicked_points = []
        elif key == ord('s') and len(clicked_points) == 4:
            filename = f"points_{cam_label_map[cam_index]}.npy"
            np.save(filename, np.array(clicked_points, dtype=np.float32))
            messagebox.showinfo("Saved", f"成功儲存為 {filename}")
            break

    cap.release()
    cv2.destroyAllWindows()

# tkinter UI
root = tk.Tk()
root.title("相機標定 GUI 工具")
root.geometry("300x150")

cam_label_map = {2: "left", 0: "right"}

label = tk.Label(root, text="選擇相機 index:")
label.pack(pady=10)

cam_var = tk.StringVar(value="2")
cam_dropdown = ttk.Combobox(root, textvariable=cam_var, state="readonly")
cam_dropdown['values'] = list(cam_label_map.keys())
cam_dropdown.pack()

start_button = tk.Button(root, text="開始擷取四點", command=start_capture)
start_button.pack(pady=20)

root.mainloop()
