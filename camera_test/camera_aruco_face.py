import cv2
import numpy as np
import math
from typing import Tuple, Optional

# ==========================================
# [translate:參數設定]
# ==========================================
CAM1_ID = 0
CAM2_ID = 2
CAM1_CALIB_FILE = 'calib_cam0.npz'
CAM2_CALIB_FILE = 'calib_cam2.npz'

def get_camera_parameters(filename: str) -> Tuple[np.ndarray, np.ndarray]:
    try:
        data = np.load(filename)
        return data['mtx'], data['dist']
    except:
        print(f"[translate:警告] 載入參數失敗: {filename}，使用預設值")
        return np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32), np.zeros((5,), dtype=np.float32)

def create_aruco_detector() -> cv2.aruco.ArucoDetector:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(aruco_dict, parameters)

def get_object_points(marker_length=0.05) -> np.ndarray:
    half_len = marker_length / 2
    return np.array([
        [-half_len,  half_len, 0],
        [ half_len,  half_len, 0],
        [ half_len, -half_len, 0],
        [-half_len, -half_len, 0]
    ], dtype=np.float32)

def estimate_pose(corners, obj_points, camera_matrix, dist_coeffs):
    img_points = corners.reshape(4, 2).astype(np.float32)
    success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
    if success:
        R, _ = cv2.Rodrigues(rvec)
        return rvec, tvec, R
    else:
        return None

def calculate_facing_direction(tvec, R):
    """
    [translate:計算方向] (全域旋轉 180 度版)
    不再需要 cam_name 參數，因為所有相機都要轉
    """
    y_axis = R[:, 1]
    x_axis = R[:, 0]
    tvec_norm = tvec.flatten() / np.linalg.norm(tvec)
    
    dot_y = np.dot(tvec_norm, y_axis)
    dot_x = np.dot(tvec_norm, x_axis)
    
    angle = math.degrees(math.atan2(dot_x, dot_y))
    
    if angle < 0:
        angle += 360

    # [關鍵修改] 無條件旋轉 180 度 (適用於兩台相機)
    angle = (angle + 180) % 360

    direction_sectors = {
        "front": 0, "front-left": 45, "left": 90, "back-left": 135,
        "back": 180, "back-right": 225, "right": 270, "front-right": 315
    }

    min_diff = 360
    best_direction = None
    for direction, d_angle in direction_sectors.items():
        diff = abs(angle - d_angle)
        diff = min(diff, 360 - diff)
        if diff < min_diff:
            min_diff = diff
            best_direction = direction

    return angle, best_direction

def process_frame(frame, detector, mtx, dist, obj_points, selected_ids, cam_name):
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        for i, id_ in enumerate(ids):
            if id_[0] in selected_ids:
                corner = corners[i]
                pose = estimate_pose(corner, obj_points, mtx, dist)
                
                if pose is not None:
                    rvec, tvec, R = pose
                    
                    # 計算方向 (已內建 +180 度)
                    angle, direction = calculate_facing_direction(tvec, R)
                    
                    rad = math.radians(angle)
                    vec_x = math.cos(rad) * (-1)
                    vec_y = math.sin(rad)
                    
                    c = tuple(corner[0][0].astype(int))
                    
                    cv2.putText(frame, f"[{cam_name}]", (c[0], c[1] - 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(frame, f"ID:{id_[0]} Ang:{angle:.1f}", (c[0], c[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"Dir:{direction}", (c[0], c[1] - 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Vec:({vec_x:.2f},{vec_y:.2f})", (c[0], c[1] - 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.03)
                    cv2.aruco.drawDetectedMarkers(frame, [corner])
    return frame

def main():
    input_ids = input("[translate:輸入 ID (逗號分隔)]: ")
    target_ids = [int(x) for x in input_ids.split(',') if x.strip().isdigit()]

    mtx1, dist1 = get_camera_parameters(CAM1_CALIB_FILE)
    mtx2, dist2 = get_camera_parameters(CAM2_CALIB_FILE)

    cap1 = cv2.VideoCapture(CAM1_ID)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    cap2 = cv2.VideoCapture(CAM2_ID)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap1.isOpened() or not cap2.isOpened():
        print("[translate:錯誤] 無法開啟相機")
        return

    detector = create_aruco_detector()
    obj_points = get_object_points()

    print("[translate:系統啟動] 所有相機角度皆已設為 +180 度修正")

    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        
        if not ret1 or not ret2: break

        res1 = process_frame(frame1, detector, mtx1, dist1, obj_points, target_ids, "Camera 1")
        res2 = process_frame(frame2, detector, mtx2, dist2, obj_points, target_ids, "Camera 2")

        cv2.imshow("Camera 1 (Left - Rotated 180)", res1)
        cv2.imshow("Camera 2 (Right - Rotated 180)", res2)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
