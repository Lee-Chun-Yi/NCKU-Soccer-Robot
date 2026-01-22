import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import sys

clicked_points = []
frame_display = None
cap = None
camera_matrix = None
dist_coeffs = None

'''
    使用說明：
    先挑選相機，放校正板，按下按鈕，如果有偵測到就會彈出影像
    按 y ： 保存這張照片
    按 n ： 捨棄這張照片
    按 q ： 離開
'''

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
    global clicked_points, frame_display, cap, camera_matrix, dist_coeffs
    clicked_points = []

    cam_index = int(cam_var.get())
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        messagebox.showerror("Error", f"無法打開相機 index {cam_index}")
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

    # 改為手動選擇保存的設定
    chessboard_size = (8, 6)
    captured_images = 0
    max_images = 25 #設定要拍幾張
    objpoints = []
    imgpoints = []

    square_size = 0.04  # 每個格子 4 cm -> 0.04 m
    objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp[:, :2] *= square_size


    # while captured_images < max_images:
    #     ret, frame = cap.read()
    #     if not ret:
    #         messagebox.showerror("Error", "無法讀取影像")
    #         break

    #     frame_display = frame.copy()
    #     gray = cv2.cvtColor(frame_display, cv2.COLOR_BGR2GRAY)
    #     ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    #     if ret_corners:
    #         cv2.drawChessboardCorners(frame_display, chessboard_size, corners, ret_corners)
    #         cv2.imshow("Detected Chessboard - Press y to save, n to skip", frame_display)
    #         while True:
    #             key = cv2.waitKey(0) & 0xFF
    #             if key == ord('y'):
    #                 objpoints.append(objp)
    #                 imgpoints.append(corners)
    #                 captured_images += 1
    #                 print(f"✔️ 儲存第 {captured_images} 張棋盤格圖")
    #                 break
    #             elif key == ord('n'):
    #                 print("跳過此張影像")
    #                 break
    #             elif key == 27 or key == ord('q'):
    #                 cap.release()
    #                 cv2.destroyAllWindows()
    #                 sys.exit()
    #         cv2.destroyWindow("Detected Chessboard - Press y to save, n to skip")

    # cap.release()
    # cv2.destroyAllWindows()

    # 開啟預覽視窗
    cv2.namedWindow("Camera Preview", cv2.WINDOW_NORMAL)  # 可縮放視窗
    cv2.resizeWindow("Camera Preview", 960, 540)           # 設定視窗大小為 960x540
    cv2.namedWindow("Camera Preview")
    while captured_images < max_images:
        ret, frame = cap.read()
        if not ret:
            messagebox.showerror("Error", "無法讀取影像")
            break

        frame_display = frame.copy()
        gray = cv2.cvtColor(frame_display, cv2.COLOR_BGR2GRAY)
        ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret_corners:
            cv2.drawChessboardCorners(frame_display, chessboard_size, corners, ret_corners)

        cv2.imshow("Camera Preview", frame_display)
        key = cv2.waitKey(1) & 0xFF

        # 按鍵操作只在偵測到角點時有效
        if ret_corners:
            if key == ord('y'):
                objpoints.append(objp)
                imgpoints.append(corners)
                captured_images += 1
                print(f"✔️ 儲存第 {captured_images} 張棋盤格圖")
            elif key == ord('n'):
                print("跳過此張影像")
            elif key == ord('q') or key == 27:
                break
        else:
            # 即使沒抓到，也允許用 q 退出
            if key == ord('q') or key == 27:
                break

    cap.release()
    cv2.destroyAllWindows()


    # 執行校正
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("\n✅ 校正完成")
    print("內參矩陣 (mtx):\n", mtx)
    print("畸變參數 (dist):\n", dist)

    # 儲存內參矩陣（純矩陣格式）
    if cam_index == 2:
        np.save('calibration_matrix_left.npy', mtx)
        np.save('distortion_coefficients_left.npy', dist)
    else:
        np.save('calibration_matrix_right.npy', mtx)
        np.save('distortion_coefficients_right.npy', dist)

    print("\n📁 內參矩陣已儲存：", 'calibration_matrix_left.npy' if cam_index == 2 else 'calibration_matrix_right.npy')

    # 顯示 undistort 對比圖
    cap = cv2.VideoCapture(cam_index)
    ret, sample_frame = cap.read()
    if ret:
        undistorted_sample = cv2.undistort(sample_frame, mtx, dist)
        comparison = np.hstack((sample_frame, undistorted_sample))
        cv2.imshow("Undistort Comparison", comparison)
        key = cv2.waitKey(0) & 0xFF
        if key == 27 or key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()
    cap.release()

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

start_button = tk.Button(root, text="開始手動擷取並校正", command=start_capture)
start_button.pack(pady=20)

root.mainloop()
