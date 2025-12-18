import cv2
import numpy as np
import math
import threading
import time
from typing import Tuple, Optional
import sys


# ==========================================
# [全域常數] 場地與視窗設定
# ==========================================
REAL_WIDTH = 360.0      # 場地實際寬度 (cm)
REAL_HEIGHT = 260.0     # 場地實際高度 (cm)
WINDOW_WIDTH = 720      # 視窗寬度 (pixel)
WINDOW_HEIGHT = 520     # 視窗高度 (pixel)


# ==========================================
# [類別定義] Ball - 球體追蹤系統
# ==========================================
class Ball():    
    def __init__(self, diameter_cm=14.0, size_tolerance=0.2):
        """
        初始化球體物件
        :param diameter_cm: 球體實際直徑 (cm)
        :param size_tolerance: 半徑容許誤差 (0.5 代表 ±50%)
        """
        self.diameter_cm = diameter_cm
        self.size_tolerance = size_tolerance
        self.hsv_range = None  # 儲存 (lower, upper)
        
        # 目前的狀態
        self.center_px = None  # (x, y) 像素座標
        self.center_real = None # (x, y) 實際 cm 座標
        self.is_tracked = False
        self.radius_px = 0
        
        # 計算期望的像素半徑
        self.expected_radius_px = self._calculate_expected_pixel_radius()
        
        # 顯示初始化資訊
        print(f"[Ball] 初始化: 直徑={self.diameter_cm}cm, 期望半徑={self.expected_radius_px:.1f}px")
        
    def _calculate_expected_pixel_radius(self):
        """內部方法:將實際 cm 換算成 pixel 半徑"""
        scale_x = WINDOW_WIDTH / REAL_WIDTH
        scale_y = WINDOW_HEIGHT / REAL_HEIGHT
        px_per_cm = (scale_x + scale_y) / 2.0
        return (self.diameter_cm / 2.0) * px_per_cm


    def set_hsv_range_from_selection(self, image, x, y, range_h=5):
        """根據滑鼠點擊位置自動計算 HSV 範圍"""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        x_start = max(x - range_h, 0)
        x_end = min(x + range_h + 1, hsv_image.shape[1])
        y_start = max(y - range_h, 0)
        y_end = min(y + range_h + 1, hsv_image.shape[0])
        
        region = hsv_image[y_start:y_end, x_start:x_end]
        if region.size == 0: return


        avg_hsv = np.mean(region.reshape(-1, 3), axis=0).astype(int)
        
        lower = np.array([max(avg_hsv[0]-40,0), max(avg_hsv[1]-100,0), max(avg_hsv[2]-100,0)])
        upper = np.array([min(avg_hsv[0]+40,179), min(avg_hsv[1]+100,255), min(avg_hsv[2]+100,255)])
        
        self.hsv_range = (lower, upper)
        print(f"[Ball] HSV 設定更新: {self.hsv_range}")


    def track(self, image):
        """
        在輸入影像中追蹤球體
        :param image: 輸入影像 (通常是鳥瞰圖)
        :return: 處理後的影像 (畫上追蹤結果), mask, 是否追蹤成功
        """
        if self.hsv_range is None:
            self.is_tracked = False
            return image, None, False


        # 影像前處理
        enhanced = self._apply_clahe(image)
        hsv_img = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        
        # 產生 Mask
        mask = cv2.inRange(hsv_img, self.hsv_range[0], self.hsv_range[1])
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        blurred = cv2.GaussianBlur(mask, (9,9), 2)


        # Hough Circle 偵測
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
            param1=50, param2=15,
            minRadius=1, maxRadius=100  # 設寬一點,由後續邏輯過濾
        )


        result_img = image.copy()
        self.is_tracked = False
        self.center_px = None
        self.center_real = None


        if circles is not None:
            circles = np.uint16(np.around(circles))
            for (x, y, r) in circles[0, :]:
                # 檢查尺寸是否符合預期
                if self._is_size_valid(r):
                    self.center_px = (int(x), int(y))
                    self.radius_px = int(r)
                    self.is_tracked = True
                    
                    # 計算實際座標
                    self._calculate_real_coordinates()
                    
                    # 繪圖
                    self._draw_info(result_img)
                    break # 找到一個符合的就跳出


        return result_img, mask, self.is_tracked


    def _is_size_valid(self, radius):
        """檢查半徑是否在容許誤差範圍內"""
        r_min = self.expected_radius_px * (1.0 - self.size_tolerance)
        r_max = self.expected_radius_px * (1.0 + self.size_tolerance)
        # 確保至少大於 1px
        if r_min < 1: r_min = 1
        return r_min <= radius <= r_max


    def _calculate_real_coordinates(self):
        """像素座標轉實際座標"""
        if self.center_px:
            px, py = self.center_px
            real_x = px * (REAL_WIDTH / WINDOW_WIDTH)
            real_y = py * (REAL_HEIGHT / WINDOW_HEIGHT)
            self.center_real = (real_x, real_y)


    def _draw_info(self, image):
        """在影像上繪製追蹤資訊"""
        if not self.is_tracked: return
        
        cx, cy = self.center_px
        rx, ry = self.center_real
        
        # 畫圓與中心點
        cv2.circle(image, (cx, cy), self.radius_px, (0, 255, 0), 2)
        cv2.circle(image, (cx, cy), 2, (0, 0, 255), 3)
        
        # 顯示資訊文字
        info_text = f"Ball: ({rx:.1f}, {ry:.1f}) cm"
        radius_text = f"r={self.radius_px}px"
        
        # 左上角顯示座標
        cv2.putText(image, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # 球體旁顯示半徑
        cv2.putText(image, radius_text, (cx + 15, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)


    def _apply_clahe(self, image):
        """內部工具:CLAHE 增強"""
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        merged = cv2.merge((cl, a, b))
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)


# ==========================================
# [類別定義] Robot - 機器人追蹤系統
# ==========================================
class Robot():
    def __init__(self, id, aruco_id_list, aruco_x_list, aruco_y_list, aruco_degree_list, camera_setting="right"):
        # 原有的 Robot 基本屬性
        self.id = id 
        self.aruco_id_list = aruco_id_list
        self.aruco_x_list = aruco_x_list
        self.aruco_y_list = aruco_y_list
        self.aruco_degree_list = aruco_degree_list
        self.x = 0
        self.y = 0
        self.degree = 0
        self.x_left, self.x_right, self.y_left, self.y_right, self.degree_left, self.degree_right = 0, 0, 0, 0, 0, 0
        
        # 新增: 向量資訊
        self.vec_x = 0.0
        self.vec_y = 0.0
        self.vec_x_left = 0.0
        self.vec_x_right = 0.0
        self.vec_y_left = 0.0
        self.vec_y_right = 0.0
        
        self.camera_setting = camera_setting
        self.amount_choose_left = 0
        self.amount_choose_right = 0 
        self.is_enemy = False
        
        # ==========================================
        # 雙相機追蹤系統 - 全域設定
        # ==========================================
        self.CAM1_ID = 0
        self.CAM2_ID = 0
        self.MARKER_SIZE = 0.08  # 標記大小通常是一樣的
        
        # ==========================================
        # 相機 1 專屬設定與參數
        # ==========================================
        self.CONFIG_CAM1 = {
            'calib_file': 'calib_cam0.npz',
            'camera_height': 2.2,
            'robot_height': 0.27,
            'offset_map': {
                10: {'x_shift': -46, 'y_shift': 0},
                20: {'x_shift': 0, 'y_shift': 0},
                0: {'x_shift': 0, 'y_shift': 0},
            }
        }
        
        # ==========================================
        # 相機 2 專屬設定與參數
        # ==========================================
        self.CONFIG_CAM2 = {
            'calib_file': 'calib_cam2.npz',
            'camera_height': 2.2,
            'robot_height': 0.27,
            'offset_map': {
                10: {'x_shift': 0.0, 'y_shift': 0.0},
                20: {'x_shift': 0.0, 'y_shift': 0.0},
                0: {'x_shift': 0.0, 'y_shift': 0.0},
            }
        }
        
        # ==========================================
        # 內部類別定義 - CameraStream
        # ==========================================
        class CameraStream:
            def __init__(self, src=0, name="Camera"):
                self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
                if not self.stream.isOpened():
                    self.stream = cv2.VideoCapture(src)
                
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
                    print(f"錯誤 無法開啟 {name}")
            
            def start(self):
                t = threading.Thread(target=self.update, args=())
                t.daemon = True
                t.start()
                return self
            
            def update(self):
                while True:
                    if self.stopped: return
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
        
        self.CameraStream = CameraStream
        
        # ==========================================
        # 工具函數定義
        # ==========================================
        def load_calibration(calib_file):
            try:
                data = np.load(calib_file)
                return data['mtx'], data['dist']
            except FileNotFoundError:
                print(f"警告 找不到 {calib_file},使用預設參數")
                return np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32), np.zeros((5,), dtype=np.float32)
        
        def initialize_detector():
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
            p = cv2.aruco.DetectorParameters()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 
            p.cornerRefinementWinSize = 5
            return cv2.aruco.ArucoDetector(d, p)
        
        self.load_calibration = load_calibration
        self.initialize_detector = initialize_detector
        
        self.mtx1, self.dist1 = load_calibration(self.CONFIG_CAM1['calib_file'])
        self.mtx2, self.dist2 = load_calibration(self.CONFIG_CAM2['calib_file'])
        self.detector = initialize_detector()


# ==========================================
# [獨立工具函數] - 機器人追蹤處理管線
# ==========================================
def process_robot_pipeline(frame, detector, mtx, dist, target_ids, cam_role, robots_dict, config_cam1, config_cam2, marker_size):
    """
    處理機器人追蹤的完整流程
    """
    if frame is None: return None
    
    # 只去畸變,不旋轉
    frame_undist = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    
    # 建立物件點
    h = marker_size / 2.0
    obj_pts = np.array([[-h,h,0], [h,h,0], [h,-h,0], [-h,-h,0]], dtype=np.float32)
    
    if ids is not None:
        for i, mid in enumerate(ids.flatten()):
            if mid not in target_ids: continue
            
            success, rvec, tvec = cv2.solvePnP(obj_pts, corners[i], mtx, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            
            if success:
                R, _ = cv2.Rodrigues(rvec)
                
                # 計算物理座標
                fX, fY = None, None
                if cam_role == "Cam1":
                    z_cam = tvec[2][0]
                    x_cam = tvec[1][0]
                    y_cam = tvec[0][0]
                    
                    h = config_cam1['camera_height']
                    h_robot = config_cam1['robot_height']
                    
                    val = (x_cam**2 + z_cam**2) - (h - h_robot)**2
                    if val >= 0:
                        X_raw = math.sqrt(val)
                        Y_raw = y_cam
                        
                        Xa = (X_raw - 1.3) * 100 
                        Ya = (Y_raw + 1.4) * 100
                        
                        off = config_cam1['offset_map'].get(mid, {'x_shift': 0.0, 'y_shift': 0.0})
                        fX = Xa + off['x_shift']
                        fY = Ya + off['y_shift']
                else:  # Cam2
                    z_cam = tvec[2][0]
                    x_cam = tvec[1][0]
                    y_cam = tvec[0][0]
                    
                    h = config_cam2['camera_height']
                    h_robot = config_cam2['robot_height']
                    
                    val = (x_cam**2 + z_cam**2) - (h - h_robot)**2
                    if val >= 0:
                        X_raw = math.sqrt(val)
                        Y_raw = y_cam
                        
                        Xa = 360 - (X_raw - 1) * 100.0
                        Ya = (1.2 - Y_raw) * 100.0
                        
                        off = config_cam2['offset_map'].get(mid, {'x_shift': 0.0, 'y_shift': 0.0})
                        fX = Xa + off['x_shift']
                        fY = Ya + off['y_shift']
                
                # 計算朝向角度
                y_axis = R[:, 1]
                x_axis = R[:, 0]
                tvec_norm = tvec.flatten() / np.linalg.norm(tvec)
                
                dot_y = np.dot(tvec_norm, y_axis)
                dot_x = np.dot(tvec_norm, x_axis)
                
                angle = math.degrees(math.atan2(dot_x, dot_y))
                
                if angle < 0:
                    angle += 360
                
                angle = (angle + 180) % 360
                
                rad = math.radians(angle)
                vec_x = math.cos(rad) * (-1)
                vec_y = math.sin(rad)
                
                # 針對 Cam2 的向量加上負號
                if cam_role == "Cam2":
                    vec_x = -vec_x
                    vec_y = -vec_y
                
                # 更新對應的 Robot 物件 (不管 fX 是否為 None 都更新向量和角度)
                if mid in robots_dict:
                    robot = robots_dict[mid]
                    if cam_role == "Cam1":
                        if fX is not None and fY is not None:
                            robot.x_left = fX
                            robot.y_left = fY
                        robot.degree_left = angle
                        robot.vec_x_left = vec_x
                        robot.vec_y_left = vec_y
                        robot.amount_choose_left += 1
                    else:  # Cam2
                        if fX is not None and fY is not None:
                            robot.x_right = fX
                            robot.y_right = fY
                        robot.degree_right = angle
                        robot.vec_x_right = vec_x
                        robot.vec_y_right = vec_y
                        robot.amount_choose_right += 1
                    
                    # 根據 camera_setting 更新主要數據 (改進版,支持 fallback)
                    if robot.camera_setting == "left":
                        if robot.amount_choose_left > 0:
                            robot.x = robot.x_left
                            robot.y = robot.y_left
                            robot.degree = robot.degree_left
                            robot.vec_x = robot.vec_x_left
                            robot.vec_y = robot.vec_y_left
                        elif robot.amount_choose_right > 0:  # fallback to right
                            robot.x = robot.x_right
                            robot.y = robot.y_right
                            robot.degree = robot.degree_right
                            robot.vec_x = robot.vec_x_right
                            robot.vec_y = robot.vec_y_right
                    elif robot.camera_setting == "right":
                        if robot.amount_choose_right > 0:
                            robot.x = robot.x_right
                            robot.y = robot.y_right
                            robot.degree = robot.degree_right
                            robot.vec_x = robot.vec_x_right
                            robot.vec_y = robot.vec_y_right
                        elif robot.amount_choose_left > 0:  # fallback to left
                            robot.x = robot.x_left
                            robot.y = robot.y_left
                            robot.degree = robot.degree_left
                            robot.vec_x = robot.vec_x_left
                            robot.vec_y = robot.vec_y_left
                    else:  # 平均或其他設定
                        if robot.amount_choose_left > 0 and robot.amount_choose_right > 0:
                            robot.x = (robot.x_left + robot.x_right) / 2.0
                            robot.y = (robot.y_left + robot.y_right) / 2.0
                            robot.degree = (robot.degree_left + robot.degree_right) / 2.0
                            robot.vec_x = (robot.vec_x_left + robot.vec_x_right) / 2.0
                            robot.vec_y = (robot.vec_y_left + robot.vec_y_right) / 2.0
                        elif robot.amount_choose_left > 0:
                            robot.x = robot.x_left
                            robot.y = robot.y_left
                            robot.degree = robot.degree_left
                            robot.vec_x = robot.vec_x_left
                            robot.vec_y = robot.vec_y_left
                        elif robot.amount_choose_right > 0:
                            robot.x = robot.x_right
                            robot.y = robot.y_right
                            robot.degree = robot.degree_right
                            robot.vec_x = robot.vec_x_right
                            robot.vec_y = robot.vec_y_right
                
                # 繪圖顯示
                cv2.aruco.drawDetectedMarkers(frame_undist, [corners[i]])
                c = tuple(corners[i][0][0].astype(int))
                
                cv2.putText(frame_undist, f"[{cam_role}]", (c[0], c[1]-100), 0, 0.6, (0,255,255), 2)
                cv2.putText(frame_undist, f"ID:{mid}", (c[0],c[1]-30), 0, 0.5, (0,255,0), 2)
                
                if fX is not None and fY is not None:
                    cv2.putText(frame_undist, f"X:{fX:.1f}", (c[0],c[1]-10), 0, 0.5, (0,255,255), 2)
                    cv2.putText(frame_undist, f"Y:{fY:.1f}", (c[0],c[1]+10), 0, 0.5, (0,255,255), 2)
                else:
                    cv2.putText(frame_undist, f"X:N/A", (c[0],c[1]-10), 0, 0.5, (0,0,255), 2)
                    cv2.putText(frame_undist, f"Y:N/A", (c[0],c[1]+10), 0, 0.5, (0,0,255), 2)
                    
                cv2.putText(frame_undist, f"Ang:{angle:.1f}", (c[0], c[1]-55), 0, 0.5, (0,255,0), 2)
                cv2.putText(frame_undist, f"Vec:({vec_x:.2f},{vec_y:.2f})", (c[0], c[1]-70), 0, 0.5, (255,0,0), 2)
                
                cv2.drawFrameAxes(frame_undist, mtx, dist, rvec, tvec, 0.03)
                
                # Console 輸出
                if mid in robots_dict:
                    robot_id = robots_dict[mid].id
                    if fX is not None and fY is not None:
                        print(f"[{cam_role}] Robot{robot_id} ID:{mid} | X={fX:.1f}, Y={fY:.1f} | Angle={angle:.1f}° | Vec:({vec_x:.2f},{vec_y:.2f})")
                    else:
                        print(f"[{cam_role}] Robot{robot_id} ID:{mid} | 座標計算失敗 | Angle={angle:.1f}° | Vec:({vec_x:.2f},{vec_y:.2f})")
    
    return frame_undist


# ==========================================
# [相機管理工具函數]
# ==========================================
def load_calibration_params(filename):
    """載入相機校正參數"""
    try:
        data = np.load(filename)
        return data['mtx'], data['dist']
    except:
        return np.eye(3), np.zeros(5)


def undistort_image(frame, mtx, dist, rotate_direction=None):
    """去除影像畸變並可選旋轉"""
    h, w = frame.shape[:2]
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)
    if rotate_direction == 'clockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_CLOCKWISE)
    elif rotate_direction == 'counterclockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return undistorted


def sort_points(pts):
    """排序四個角點為標準順序"""
    pts = np.array(pts)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    return np.array([pts[np.argmin(s)], pts[np.argmin(diff)], 
                     pts[np.argmax(s)], pts[np.argmax(diff)]], dtype=np.float32)


def get_perspective_warp(image, pts, width, height):
    """透視變換為鳥瞰圖"""
    pts_sorted = sort_points(pts)
    dst = np.array([[0,0], [width-1,0], [width-1,height-1], [0,height-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts_sorted, dst)
    return cv2.warpPerspective(image, M, (width, height))


def draw_points(image, points):
    """在影像上標記選定的點"""
    for pt in points:
        cv2.circle(image, pt, 5, (0,0,255), -1)


# ==========================================
# [全域變數]
# ==========================================
points_cam0 = []
points_cam2 = []
my_ball = Ball(diameter_cm=14.0, size_tolerance=0.2)


# ==========================================
# [滑鼠回調函式]
# ==========================================
def mouse_callback_fused(event, x, y, flags, param):
    """融合視窗中點擊設定球體顏色"""
    if event == cv2.EVENT_LBUTTONDOWN:
        fused_image = param
        my_ball.set_hsv_range_from_selection(fused_image, x, y)


def mouse_callback_cam0(event, x, y, flags, param):
    """相機0選點回調"""
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam0) < 4:
        points_cam0.append((x,y))
        print(f"Camera 0 選點: ({x},{y})")


def mouse_callback_cam2(event, x, y, flags, param):
    """相機2選點回調"""
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam2) < 4:
        points_cam2.append((x,y))
        print(f"Camera 2 選點: ({x},{y})")


# ==========================================
# [主程式]
# ==========================================
def _open_camera(cam_id):
    """Open camera with a sensible backend per OS, fallback to default."""
    backends = []
    if sys.platform.startswith("win"):
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, 0]
    elif sys.platform.startswith("linux"):
        backends = [cv2.CAP_V4L2, 0]
    else:
        backends = [0]

    cap = None
    for backend in backends:
        cap = cv2.VideoCapture(cam_id, backend) if backend != 0 else cv2.VideoCapture(cam_id)
        if cap.isOpened():
            break
        cap.release()
        cap = None
    return cap


def main():
    print("=" * 60)
    print("整合系統啟動: 球體追蹤 + 機器人追蹤")
    print("=" * 60)
    
    # ==========================================
    # 1. 建立機器人實例
    # ==========================================
    robot_0 = Robot(0, [0], [0.0], [0.0], [0.0], "right")
    robot_1 = Robot(1, [10], [0.0], [0.0], [0.0], "right")
    robot_2 = Robot(2, [20], [0.0], [0.0], [0.0], "right")
    
    robots = [robot_0, robot_1, robot_2]
    
    print("機器人追蹤系統:")
    for robot in robots:
        print(f"  機器人 {robot.id}: ArUco ID={robot.aruco_id_list}, camera_setting={robot.camera_setting}")
    
    # 收集所有要追蹤的 ArUco ID
    target_ids = []
    for robot in robots:
        target_ids.extend(robot.aruco_id_list)
    
    # 建立 ArUco ID 到 Robot 的映射字典
    robots_dict = {}
    for robot in robots:
        for aruco_id in robot.aruco_id_list:
            robots_dict[aruco_id] = robot
    
    print(f"追蹤目標 ID: {target_ids}")
    
    # ==========================================
    # 2. 載入相機校正參數 (共用)
    # ==========================================
    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')
    
    # 使用第一個機器人的 ArUco 偵測器
    detector = robot_0.detector
    
    # ==========================================
    # 3. 開啟相機
    # ==========================================
    # Use DirectShow on Windows to avoid MSMF grab errors
    cap0 = _open_camera(robot_0.CAM1_ID)
    cap2 = _open_camera(robot_0.CAM2_ID)
    
    # 設定相機解析度
    for cap in [cap0, cap2]:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


    if not cap0.isOpened() or not cap2.isOpened():
        print("錯誤: 無法開啟相機")
        return


    # 權重初始值
    weight_cam0 = 0.5
    weight_cam2 = 0.5


    # ==========================================
    # 4. 建立視窗並設定回調
    # ==========================================
    cv2.namedWindow("Camera 0 - Perspective")
    cv2.setMouseCallback("Camera 0 - Perspective", mouse_callback_cam0)
    cv2.namedWindow("Camera 2 - Perspective")
    cv2.setMouseCallback("Camera 2 - Perspective", mouse_callback_cam2)
    cv2.namedWindow("Fused View + Ball Track")
    cv2.setMouseCallback("Fused View + Ball Track", lambda *args: None)
    cv2.namedWindow("Robot Track - Cam0")
    cv2.namedWindow("Robot Track - Cam2")


    print("\n[系統操作說明]")
    print("1. 在 'Camera 0/2 - Perspective' 視窗各點選 4 個場地角點")
    print("2. 完成後在 'Fused View + Ball Track' 點擊球體設定顏色")
    print("3. 系統將同時追蹤球體與機器人")
    print("4. 按 ESC 離開\n")


    try:
        while True:
            ret0, frame0 = cap0.read()
            ret2, frame2 = cap2.read()
            if not ret0 or not ret2: break


            # ==========================================
            # 5. 影像前處理 (去畸變)
            # ==========================================
            undist0 = undistort_image(frame0, mtx_cam0, dist_cam0, 'clockwise')
            undist2 = undistort_image(frame2, mtx_cam2, dist_cam2, 'counterclockwise')


            # ==========================================
            # 6. 顯示選點畫面
            # ==========================================
            disp0, disp2 = undist0.copy(), undist2.copy()
            draw_points(disp0, points_cam0)
            draw_points(disp2, points_cam2)
            cv2.imshow("Camera 0 - Perspective", disp0)
            cv2.imshow("Camera 2 - Perspective", disp2)


            # ==========================================
            # 7. 機器人追蹤 (ArUco) - 使用獨立函數
            # ==========================================
            robot_res0 = process_robot_pipeline(
                frame0, detector, mtx_cam0, dist_cam0, target_ids, "Cam1", 
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE
            )
            robot_res2 = process_robot_pipeline(
                frame2, detector, mtx_cam2, dist_cam2, target_ids, "Cam2", 
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE
            )
            
            if robot_res0 is not None:
                cv2.imshow("Robot Track - Cam0", robot_res0)
            if robot_res2 is not None:
                cv2.imshow("Robot Track - Cam2", robot_res2)


            # ==========================================
            # 8. 球體追蹤 (需完成選點)
            # ==========================================
            if len(points_cam0) == 4 and len(points_cam2) == 4:
                warped0 = get_perspective_warp(undist0, points_cam0, WINDOW_WIDTH, WINDOW_HEIGHT)
                warped2 = get_perspective_warp(undist2, points_cam2, WINDOW_WIDTH, WINDOW_HEIGHT)


                # 偵測球體以動態調整權重
                _, _, tracked0 = my_ball.track(warped0)
                _, _, tracked2 = my_ball.track(warped2)


                if tracked0 and not tracked2:
                    weight_cam0, weight_cam2 = 1.0, 0.0
                elif tracked2 and not tracked0:
                    weight_cam0, weight_cam2 = 0.0, 1.0
                elif tracked0 and tracked2:
                    weight_cam0, weight_cam2 = 0.5, 0.5


                # 融合影像
                fused = cv2.addWeighted(warped0, weight_cam0, warped2, weight_cam2, 0)
                
                # 更新回調函數用的圖片
                cv2.setMouseCallback("Fused View + Ball Track", mouse_callback_fused, fused)


                # 正式追蹤並繪圖
                result_img, mask, is_tracked = my_ball.track(fused)
                
                # 在融合視窗顯示球體追蹤結果
                cv2.imshow("Fused View + Ball Track", result_img)
                if mask is not None:
                    cv2.imshow("Ball Mask", mask)
                
                if is_tracked:
                    print(f"\r[Ball] 座標: {my_ball.center_real}", end="")


            # ==========================================
            # 10. 等待按鍵
            # ==========================================
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break


    except KeyboardInterrupt:
        print("\n中斷信號收到")
    finally:
        cap0.release()
        cap2.release()
        cv2.destroyAllWindows()
        print("\n系統已關閉")
        
        # 顯示最終統計
        print("\n=== 追蹤統計 ===")
        for robot in robots:
            print(f"\nRobot {robot.id} (ArUco ID:{robot.aruco_id_list[0]}, Setting:{robot.camera_setting}):")
            print(f"  Cam1 偵測次數: {robot.amount_choose_left}")
            print(f"  Cam2 偵測次數: {robot.amount_choose_right}")
            if robot.amount_choose_left > 0:
                print(f"  Cam1 數據: X={robot.x_left:.1f}, Y={robot.y_left:.1f}, Deg={robot.degree_left:.1f}, Vec=({robot.vec_x_left:.2f},{robot.vec_y_left:.2f})")
            if robot.amount_choose_right > 0:
                print(f"  Cam2 數據: X={robot.x_right:.1f}, Y={robot.y_right:.1f}, Deg={robot.degree_right:.1f}, Vec=({robot.vec_x_right:.2f},{robot.vec_y_right:.2f})")
            if robot.amount_choose_left > 0 or robot.amount_choose_right > 0:
                print(f"  最終位置: ({robot.x:.1f}, {robot.y:.1f})")
                print(f"  最終角度: {robot.degree:.1f}°")
                print(f"  最終向量: ({robot.vec_x:.2f}, {robot.vec_y:.2f})")


if __name__ == "__main__":
    main()


# ============================================================
# Bridge for main_v5_nrfcontrol_mod2025 copy.py
#  - Expose team/ball state in the same shape as image_processing_Androsot_no_skin
#  - Provide start_image_thread() entrypoint compatible with main_v5...
# ============================================================
team_pos = []
team_degree = []
oppo_pos = []
ball_center = []

_state_lock = threading.Lock()
_worker = None
_running = False


def _update_shared_state(robots):
    """Update shared lists that main_v5 reads."""
    with _state_lock:
        team_pos.clear()
        team_degree.clear()
        team_pos.extend([[float(r.x), float(r.y)] for r in robots])
        team_degree.extend([float(r.degree) for r in robots])
        oppo_pos.clear()  # not produced by this pipeline

        if my_ball and my_ball.center_real:
            ball_center[:] = [float(my_ball.center_real[0]), float(my_ball.center_real[1])]
        else:
            ball_center.clear()


def _default_corners_from_frame(frame):
    """Return 4-corner warp points when user has not clicked any."""
    h, w = frame.shape[:2]
    return [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]


def _processing_loop(show_windows=False):
    global _running
    # Robot setup (same IDs as original complete1.py)
    robot_0 = Robot(0, [0], [0.0], [0.0], [0.0], "right")
    robot_1 = Robot(1, [10], [0.0], [0.0], [0.0], "right")
    robot_2 = Robot(2, [20], [0.0], [0.0], [0.0], "right")
    robots = [robot_0, robot_1, robot_2]

    target_ids = [rid for r in robots for rid in r.aruco_id_list]
    robots_dict = {rid: r for r in robots for rid in r.aruco_id_list}

    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')
    detector = robot_0.detector

    # Use DirectShow on Windows to avoid MSMF grab errors
    cap0 = _open_camera(robot_0.CAM1_ID)
    cap2 = _open_camera(robot_0.CAM2_ID)
    for cap in [cap0, cap2]:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap0.isOpened() or not cap2.isOpened():
        print("[imageprocess_complete1] Cannot open cameras 0/2")
        _running = False
        return

    try:
        while _running:
            ret0, frame0 = cap0.read()
            ret2, frame2 = cap2.read()
            if not ret0 or not ret2:
                print("[imageprocess_complete1] Camera read failed")
                break

            # Robot tracking (no UI drawing unless show_windows)
            robot_res0 = process_robot_pipeline(
                frame0, detector, mtx_cam0, dist_cam0, target_ids, "Cam1",
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE
            )
            robot_res2 = process_robot_pipeline(
                frame2, detector, mtx_cam2, dist_cam2, target_ids, "Cam2",
                robots_dict, robot_0.CONFIG_CAM1, robot_0.CONFIG_CAM2, robot_0.MARKER_SIZE
            )

            if show_windows:
                if robot_res0 is not None:
                    cv2.imshow("Robot Track - Cam0", robot_res0)
                if robot_res2 is not None:
                    cv2.imshow("Robot Track - Cam2", robot_res2)

            # Ball tracking (best-effort, using default corners if none selected)
            undist0 = undistort_image(frame0, mtx_cam0, dist_cam0, 'clockwise')
            undist2 = undistort_image(frame2, mtx_cam2, dist_cam2, 'counterclockwise')

            if len(points_cam0) != 4:
                points_cam0[:] = _default_corners_from_frame(undist0)
            if len(points_cam2) != 4:
                points_cam2[:] = _default_corners_from_frame(undist2)

            warped0 = get_perspective_warp(undist0, points_cam0, WINDOW_WIDTH, WINDOW_HEIGHT)
            warped2 = get_perspective_warp(undist2, points_cam2, WINDOW_WIDTH, WINDOW_HEIGHT)

            # Simple fuse and track
            fused = cv2.addWeighted(warped0, 0.5, warped2, 0.5, 0)
            result_img, mask, is_tracked = my_ball.track(fused)
            if show_windows:
                cv2.imshow("Fused View + Ball Track", result_img)
                if mask is not None:
                    cv2.imshow("Ball Mask", mask)

            _update_shared_state(robots)

            if show_windows and cv2.waitKey(1) & 0xFF == 27:
                break

            # small delay to avoid busy loop
            time.sleep(0.01)

    finally:
        cap0.release()
        cap2.release()
        if show_windows:
            cv2.destroyAllWindows()
        _running = False


def start_image_thread(show_windows=False):
    """Start background vision thread (non-blocking)."""
    global _worker, _running
    if _worker and _worker.is_alive():
        return
    _running = True
    _worker = threading.Thread(target=_processing_loop, args=(show_windows,), daemon=True)
    _worker.start()


# main_v5_nrfcontrol_mod2025 copy.py expects this name
def image_result():
    start_image_thread(show_windows=False)


# UI stubs to match old interface (HSV/Color mask not implemented here)
def HSVdetection(color, camera_selection_setting):
    print("[imageprocess_complete1] HSVdetection stub – not implemented in this pipeline")


def ColorMask(color, color_selection_setting, camera_selection_setting):
    print("[imageprocess_complete1] ColorMask stub – not implemented in this pipeline")


def imagedetection(fieldchoose=0):
    """Start vision pipeline with UI windows (matches old button behavior)."""
    start_image_thread(show_windows=True)
