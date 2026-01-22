import cv2
import numpy as np

# ==========================================
# [全域常數] 場地與視窗設定
# ==========================================
REAL_WIDTH = 360.0      # 場地實際寬度 (cm)
REAL_HEIGHT = 260.0     # 場地實際高度 (cm)
WINDOW_WIDTH = 720      # 視窗寬度 (pixel)
WINDOW_HEIGHT = 520     # 視窗高度 (pixel)

# ==========================================
# [類別定義] Ball
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
        """內部方法：將實際 cm 換算成 pixel 半徑"""
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
            minRadius=1, maxRadius=100  # 設寬一點，由後續邏輯過濾
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
        info_text = f"Pos: ({rx:.1f}, {ry:.1f}) cm"
        radius_text = f"r={self.radius_px}px"
        
        # 左上角顯示座標
        cv2.putText(image, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        # 球體旁顯示半徑
        cv2.putText(image, radius_text, (cx + 15, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    def _apply_clahe(self, image):
        """內部工具：CLAHE 增強"""
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        merged = cv2.merge((cl, a, b))
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

# ==========================================
# [全域變數 (外部控制用)]
# ==========================================
points_cam0 = []
points_cam2 = []
# 建立球體實例
my_ball = Ball(diameter_cm=14.0, size_tolerance=0.2)

# ==========================================
# [輔助函式]
# ==========================================
def load_calibration_params(filename):
    try:
        data = np.load(filename)
        return data['mtx'], data['dist']
    except:
        return np.eye(3), np.zeros(5)

def undistort_image(frame, mtx, dist, rotate_direction=None):
    h, w = frame.shape[:2]
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)
    if rotate_direction == 'clockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_CLOCKWISE)
    elif rotate_direction == 'counterclockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return undistorted

def sort_points(pts):
    pts = np.array(pts)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    return np.array([pts[np.argmin(s)], pts[np.argmin(diff)], 
                     pts[np.argmax(s)], pts[np.argmax(diff)]], dtype=np.float32)

def get_perspective_warp(image, pts, width, height):
    pts_sorted = sort_points(pts)
    dst = np.array([[0,0], [width-1,0], [width-1,height-1], [0,height-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts_sorted, dst)
    return cv2.warpPerspective(image, M, (width, height))

def draw_points(image, points):
    for pt in points:
        cv2.circle(image, pt, 5, (0,0,255), -1)

# ==========================================
# [滑鼠回調函式]
# ==========================================
def mouse_callback_fused(event, x, y, flags, param):
    """透過全域變數 my_ball 處理點擊"""
    if event == cv2.EVENT_LBUTTONDOWN:
        fused_image = param
        my_ball.set_hsv_range_from_selection(fused_image, x, y)

def mouse_callback_cam0(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam0) < 4:
        points_cam0.append((x,y))
        print(f"Camera 0 選點: ({x},{y})")

def mouse_callback_cam2(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam2) < 4:
        points_cam2.append((x,y))
        print(f"Camera 2 選點: ({x},{y})")

# ==========================================
# [主程式]
# ==========================================
def main():
    # 載入校正參數
    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')

    cap0 = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(2)
    
    # 設定相機解析度
    for cap in [cap0, cap2]:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap0.isOpened() or not cap2.isOpened():
        print("無法開啟相機")
        return

    # 權重初始值
    weight_cam0 = 0.5
    weight_cam2 = 0.5

    # 視窗設定
    cv2.namedWindow("Camera 0")
    cv2.setMouseCallback("Camera 0", mouse_callback_cam0)
    cv2.namedWindow("Camera 2")
    cv2.setMouseCallback("Camera 2", mouse_callback_cam2)
    cv2.namedWindow("Fused View")
    cv2.setMouseCallback("Fused View", lambda *args: None) 

    print("[系統就緒]")
    print("1. 請在兩個相機視窗中各點選 4 個角點")
    print("2. 在融合視窗點擊球體以設定顏色追蹤")

    try:
        while True:
            ret0, frame0 = cap0.read()
            ret2, frame2 = cap2.read()
            if not ret0 or not ret2: break

            # 1. 影像前處理 (去畸變)
            undist0 = undistort_image(frame0, mtx_cam0, dist_cam0, 'clockwise')
            undist2 = undistort_image(frame2, mtx_cam2, dist_cam2, 'counterclockwise')

            # 顯示選點畫面
            disp0, disp2 = undist0.copy(), undist2.copy()
            draw_points(disp0, points_cam0)
            draw_points(disp2, points_cam2)
            cv2.imshow("Camera 0", disp0)
            cv2.imshow("Camera 2", disp2)

            if cv2.waitKey(1) & 0xFF == 27: break

            # 2. 當選點完成，進行透視與追蹤
            if len(points_cam0) == 4 and len(points_cam2) == 4:
                warped0 = get_perspective_warp(undist0, points_cam0, WINDOW_WIDTH, WINDOW_HEIGHT)
                warped2 = get_perspective_warp(undist2, points_cam2, WINDOW_WIDTH, WINDOW_HEIGHT)

                # 分別在兩個相機視角嘗試追蹤 (純偵測，不繪圖)
                # 為了決定權重，我們暫時用 my_ball 的參數去測
                _, _, tracked0 = my_ball.track(warped0)
                _, _, tracked2 = my_ball.track(warped2)

                # 動態調整權重
                if tracked0 and not tracked2:
                    weight_cam0, weight_cam2 = 1.0, 0.0
                elif tracked2 and not tracked0:
                    weight_cam0, weight_cam2 = 0.0, 1.0
                elif tracked0 and tracked2:
                    weight_cam0, weight_cam2 = 0.5, 0.5

                # 3. 融合影像
                fused = cv2.addWeighted(warped0, weight_cam0, warped2, weight_cam2, 0)
                
                # 更新回調函數用的圖片 (讓滑鼠點擊能取色)
                cv2.setMouseCallback("Fused View", mouse_callback_fused, fused)

                # 4. 在融合影像上正式追蹤並繪圖
                result_img, mask, is_tracked = my_ball.track(fused)
                
                # 顯示結果
                cv2.imshow("Fused View", result_img)
                if mask is not None:
                    cv2.imshow("Mask", mask)
                
                if is_tracked:
                    print(f"\r球體座標: {my_ball.center_real}", end="")

    except KeyboardInterrupt:
        pass
    finally:
        cap0.release()
        cap2.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
