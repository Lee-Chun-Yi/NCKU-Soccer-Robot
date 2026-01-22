import cv2
import numpy as np
import math
import threading
import time


# ==========================================
# 全域設定
# ==========================================
CAM1_ID = 0
CAM2_ID = 2
MARKER_SIZE = 0.08  # 標記大小通常是一樣的


# ==========================================
# 相機 1 專屬設定與參數
# ==========================================
CONFIG_CAM1 = {
    'calib_file': 'calib_cam0.npz',
    'camera_height': 2.2,  # 相機 1 的架設高度
    'robot_height': 0.27,  # 被測物高度
    # 相機 1 的 ID 補償表
    'offset_map': {
        10: {'x_shift': -46, 'y_shift': 0},  # ID 10 不做額外修正
        20: {'x_shift': 0, 'y_shift': 0},    # ID 12: X 減 2.5cm, Y 加 1.0cm
        0: {'x_shift': 0, 'y_shift': 0},
    }
}


# ==========================================
# 相機 2 專屬設定與參數
# ==========================================
CONFIG_CAM2 = {
    'calib_file': 'calib_cam2.npz',
    'camera_height': 2.2,  # [注意] 相機 2 高度可能不同，請修改
    'robot_height': 0.27,
    # 相機 2 的 ID 補償表 (通常與 Cam 1 不同)
    'offset_map': {
        10: {'x_shift': 0.0, 'y_shift': 0.0},  # 範例: Cam 2 不需要補償
        20: {'x_shift': 0.0, 'y_shift': 0.0},
        0: {'x_shift': 0.0, 'y_shift': 0.0},
        # 請自行新增...
    }
}


# ==========================================
# 功能區塊 1: 相機串流
# ==========================================
class CameraStream:
    def __init__(self, src=0, name="Camera"):
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if not self.stream.isOpened():
            self.stream = cv2.VideoCapture(src)

        # 頻寬優化
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.stream.set(cv2.CAP_PROP_FOURCC, fourcc)
        
        self.name = name
        self.stopped = not self.stream.isOpened()
        self.frame = None
        if not self.stopped:
            print(f"成功開啟 {name} (ID: {src})")
        else:
            print(f"錯誤: 無法開啟 {name}")

    def start(self):
        t = threading.Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            grabbed, frame = self.stream.read()
            if grabbed:
                self.frame = frame
            else:
                self.stopped = True

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()


# ==========================================
# 功能區塊 2: 基礎工具
# ==========================================
def load_calibration(calib_file):
    """載入相機校正參數"""
    try:
        data = np.load(calib_file)
        return data['mtx'], data['dist']
    except FileNotFoundError:
        print(f"警告: 找不到 {calib_file}，使用預設參數")
        default_mtx = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        default_dist = np.zeros((5,), dtype=np.float32)
        return default_mtx, default_dist


def initialize_detector():
    """初始化 ArUco 偵測器"""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    return cv2.aruco.ArucoDetector(d, p)


def create_obj_points(length):
    """建立 ArUco 標記的 3D 物件點"""
    h = length / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


# ==========================================
# 功能區塊 3: 核心運算邏輯 (完全分開)
# ==========================================
def calculate_cam1_physics(tvec, rvec, marker_id):
    """
    相機 1 的專屬物理運算
    輸入: tvec, rvec (相機 1 的原始 Pose)
    輸出: X, Y
    """
    # 1. 從 tvec 提取原始座標 (Cam 1 座標系)
    z_cam = tvec[2][0]
    x_cam = tvec[1][0]
    y_cam = tvec[0][0]
    
    # 2. 幾何計算 (使用 Cam 1 的高度設定)
    h = CONFIG_CAM1['camera_height']
    h_robot = CONFIG_CAM1['robot_height']
    
    val = (x_cam**2 + z_cam**2) - (h - h_robot)**2
    if val < 0:
        return None, None  # 數學錯誤
    
    X_raw = math.sqrt(val)
    Y_raw = y_cam  # Cam 1 特有的位移補償
    
    # 3. 多項式修正 (Cam 1 專用公式)
    Xa = (X_raw - 1.3) * 100
    Ya = (Y_raw + 1.4) * 100
    
    # 4. ID 補償 (Cam 1)
    off = CONFIG_CAM1['offset_map'].get(marker_id, {'x_shift': 0.0, 'y_shift': 0.0})
    final_X = Xa + off['x_shift']
    final_Y = Ya + off['y_shift']
    
    return final_X, final_Y


def calculate_cam2_physics(tvec, rvec, marker_id):
    """
    相機 2 的專屬物理運算
    [重點] 請在此處填寫相機 2 的邏輯
    """
    # 1. 從 tvec 提取原始座標 (Cam 2 座標系 - 這是獨立的!)
    z_cam = tvec[2][0]
    x_cam = tvec[1][0]
    y_cam = tvec[0][0]  # [注意] 如果相機 2 倒掛或旋轉，這裡的 y_cam 定義可能不同
    
    # 2. 幾何計算 (使用 Cam 2 的高度設定)
    h = CONFIG_CAM2['camera_height']
    h_robot = CONFIG_CAM2['robot_height']
    
    # [可修改] 相機 2 的幾何模型可能不同
    val = (x_cam**2 + z_cam**2) - (h - h_robot)**2
    if val < 0:
        return None, None
    
    X_raw = math.sqrt(val)
    Y_raw = y_cam  # [假設] Cam 2 不需要那個 +1.3 的位移
    
    # 3. 多項式修正 (Cam 2 專用公式 - 請填寫)
    # 範例：如果相機 2 很準，可能不需要複雜多項式
    Xa = 360 - (X_raw - 1) * 100.0   # 轉成 cm
    Ya = (1.42 + Y_raw) * 100.0  # 轉成 cm
    
    # 4. ID 補償 (Cam 2)
    off = CONFIG_CAM2['offset_map'].get(marker_id, {'x_shift': 0.0, 'y_shift': 0.0})
    final_X = Xa + off['x_shift']
    final_Y = Ya + off['y_shift']
    
    return final_X, final_Y


def process_pipeline(frame, detector, mtx, dist, target_ids, cam_role):
    """通用處理流程，但在計算階段會分流"""
    if frame is None:
        return None
    
    frame_undist = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    obj_pts = create_obj_points(MARKER_SIZE)
    
    if ids is not None:
        for i, mid in enumerate(ids.flatten()):
            if mid not in target_ids:
                continue
            
            # 算出 Pose (這是該相機座標系下的 tvec/rvec，兩台相機絕對不同)
            success, rvec, tvec = cv2.solvePnP(
                obj_pts, corners[i], mtx, dist, 
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if success:
                # [關鍵分流] 將 raw tvec/rvec 傳給各自的物理引擎
                if cam_role == "Cam1":
                    fX, fY = calculate_cam1_physics(tvec, rvec, mid)
                else:
                    fX, fY = calculate_cam2_physics(tvec, rvec, mid)
                
                if fX is not None:
                    # 繪圖
                    cv2.aruco.drawDetectedMarkers(frame_undist, [corners[i]])
                    
                    # 顯示座標
                    c = tuple(corners[i][0][0].astype(int))
                    cv2.putText(frame_undist, f"ID:{mid}", 
                              (c[0], c[1]-30), 0, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame_undist, f"X:{fX:.1f}", 
                              (c[0], c[1]-10), 0, 0.5, (0, 255, 255), 2)
                    cv2.putText(frame_undist, f"Y:{fY:.1f}", 
                              (c[0], c[1]+10), 0, 0.5, (0, 255, 255), 2)
                    
                    print(f"[{cam_role}] ID:{mid} | X={fX:.1f}, Y={fY:.1f}")
    
    return frame_undist


# ==========================================
# 主程式
# ==========================================
def main():
    # 載入各自的參數
    mtx1, dist1 = load_calibration(CONFIG_CAM1['calib_file'])
    mtx2, dist2 = load_calibration(CONFIG_CAM2['calib_file'])
    detector = initialize_detector()
    
    cam1 = CameraStream(CAM1_ID, "Cam 1").start()
    cam2 = CameraStream(CAM2_ID, "Cam 2").start()
    time.sleep(2.0)
    
    if cam1.stopped and cam2.stopped:
        print("錯誤: 無法開啟任何相機")
        return
    
    input_ids = input("輸入 ID (逗號分隔): ")
    target_ids = [int(x) for x in input_ids.split(',') if x.strip().isdigit()]
    
    try:
        while True:
            f1 = cam1.read()
            f2 = cam2.read()
            
            if f1 is not None:
                res1 = process_pipeline(f1, detector, mtx1, dist1, target_ids, "Cam1")
                if res1 is not None:
                    cv2.imshow("Camera 1", res1)
            
            if f2 is not None:
                res2 = process_pipeline(f2, detector, mtx2, dist2, target_ids, "Cam2")
                if res2 is not None:
                    cv2.imshow("Camera 2", res2)
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
                
    except KeyboardInterrupt:
        print("\n程式中斷")
    finally:
        cam1.stop()
        cam2.stop()
        cv2.destroyAllWindows()
        print("程式結束")


if __name__ == "__main__":
    main()
