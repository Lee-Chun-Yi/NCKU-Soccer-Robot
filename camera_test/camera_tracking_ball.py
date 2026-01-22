import cv2
import numpy as np

# ==========================================
# [translate:全域變數]
# ==========================================
points_cam0 = []
points_cam2 = []
tracking_hsv_range = None

# [新增] 球場實際尺寸定義 (單位: cm 或自訂單位)
REAL_WIDTH = 360.0
REAL_HEIGHT = 260.0

# 視窗顯示解析度 (實際尺寸的 2 倍)
WINDOW_WIDTH = 720
WINDOW_HEIGHT = 520

# ==========================================
# [translate:功能函式]
# ==========================================

def load_calibration_params(filename):
    try:
        data = np.load(filename)
        mtx = data['mtx']
        dist = data['dist']
        if isinstance(dist, tuple):
            dist = np.array(dist)
        dist = dist.flatten()
        return mtx, dist
    except FileNotFoundError:
        print(f"[translate:警告] 找不到 {filename}，使用預設參數")
        return np.eye(3), np.zeros(5)

def undistort_image(frame, mtx, dist, rotate_direction=None):
    h, w = frame.shape[:2]
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)
    if rotate_direction == 'clockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_CLOCKWISE)
    elif rotate_direction == 'counterclockwise':
        return cv2.rotate(undistorted, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return undistorted

def sort_points(pts):
    pts = np.array(pts)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    return np.array([
        pts[np.argmin(s)],
        pts[np.argmin(diff)],
        pts[np.argmax(s)],
        pts[np.argmax(diff)]
    ], dtype=np.float32)

def get_perspective_warp(image, pts, width, height):
    pts_sorted = sort_points(pts)
    dst = np.array([[0,0], [width-1,0], [width-1,height-1], [0,height-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts_sorted, dst)
    warped = cv2.warpPerspective(image, M, (width, height))
    return warped

def apply_clahe(image):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l,a,b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    merged = cv2.merge((cl,a,b))
    enhanced = cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)
    return enhanced

def hsv_track_circle(image, hsv_range, dp=1.2, min_dist=20, param1=50, param2=15, min_radius=1, max_radius=30):
    """
    [translate:修改重點] 計算並回傳中心座標 (pixel_x, pixel_y)
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    enhanced = apply_clahe(image)
    hsv_img = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv_img, hsv_range[0], hsv_range[1])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    blurred = cv2.GaussianBlur(mask, (9,9), 2)

    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=dp, minDist=min_dist,
                               param1=param1, param2=param2,
                               minRadius=min_radius, maxRadius=max_radius)

    contour_mask = np.zeros_like(mask)
    tracked = False
    center = None # (pixel_x, pixel_y)
    result_img = image.copy()

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for (x, y, r) in circles[0,:]:
            if r >= min_radius:
                center = (int(x), int(y))
                
                # 畫球體外框
                cv2.circle(result_img, center, r, (0,255,0), 2)
                cv2.circle(result_img, center, 2, (0,0,255), 3)
                tracked = True
                break

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        cv2.drawContours(contour_mask, [largest], -1, 255, thickness=cv2.FILLED)

    return result_img, mask, tracked, center, contour_mask

def mouse_callback_fused(event, x, y, flags, param):
    global tracking_hsv_range
    fused_image = param
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_image = cv2.cvtColor(fused_image, cv2.COLOR_BGR2HSV)
        h_range = 5
        x_start = max(x - h_range, 0)
        x_end = min(x + h_range + 1, hsv_image.shape[1])
        y_start = max(y - h_range, 0)
        y_end = min(y + h_range + 1, hsv_image.shape[0])
        
        region = hsv_image[y_start:y_end, x_start:x_end]
        if region.size == 0: return
        
        avg_hsv = np.mean(region.reshape(-1, 3), axis=0).astype(int)
        
        lower = np.array([max(avg_hsv[0]-40,0), max(avg_hsv[1]-100,0), max(avg_hsv[2]-100,0)])
        upper = np.array([min(avg_hsv[0]+40,179), min(avg_hsv[1]+100,255), min(avg_hsv[2]+100,255)])
        tracking_hsv_range = (lower, upper)
        print(f"[translate:HSV 設定] {tracking_hsv_range}")

def mouse_callback_cam0(event, x, y, flags, param):
    global points_cam0
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam0) < 4:
        points_cam0.append((x,y))
        print(f"Camera 0 選點: ({x},{y})")

def mouse_callback_cam2(event, x, y, flags, param):
    global points_cam2
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam2) < 4:
        points_cam2.append((x,y))
        print(f"Camera 2 選點: ({x},{y})")

def draw_points(image, points):
    for pt in points:
        cv2.circle(image, pt, 5, (0,0,255), -1)

# ==========================================
# [translate:主程式]
# ==========================================
def main():
    global tracking_hsv_range
    
    mtx_cam0, dist_cam0 = load_calibration_params('calib_cam0.npz')
    mtx_cam2, dist_cam2 = load_calibration_params('calib_cam2.npz')

    cap0 = cv2.VideoCapture(0)
    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap0.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    cap2 = cv2.VideoCapture(2)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap0.isOpened() or not cap2.isOpened():
        print("無法開啟相機")
        return

    weight_cam0 = 0.5
    weight_cam2 = 0.5

    cv2.namedWindow("Camera 0 Undistorted")
    cv2.setMouseCallback("Camera 0 Undistorted", mouse_callback_cam0)
    cv2.namedWindow("Camera 2 Undistorted")
    cv2.setMouseCallback("Camera 2 Undistorted", mouse_callback_cam2)
    cv2.namedWindow("Fused Warped View")
    cv2.setMouseCallback("Fused Warped View", lambda *args : None)

    print("[translate:說明]")
    print(f"視窗大小: {WINDOW_WIDTH}x{WINDOW_HEIGHT}")
    print(f"實際對應: {REAL_WIDTH}x{REAL_HEIGHT}")
    print("座標 (0,0) 位於畫面左上角")

    try:
        while True:
            ret0, frame0 = cap0.read()
            ret2, frame2 = cap2.read()
            
            if not ret0 or not ret2: break

            undistorted0 = undistort_image(frame0, mtx_cam0, dist_cam0, rotate_direction='clockwise')
            undistorted2 = undistort_image(frame2, mtx_cam2, dist_cam2, rotate_direction='counterclockwise')

            disp0 = undistorted0.copy()
            disp2 = undistorted2.copy()
            draw_points(disp0, points_cam0)
            draw_points(disp2, points_cam2)

            cv2.imshow("Camera 0 Undistorted", disp0)
            cv2.imshow("Camera 2 Undistorted", disp2)

            key = cv2.waitKey(1) & 0xFF
            if key == 27: break

            if len(points_cam0) == 4 and len(points_cam2) == 4:
                # 進行透視變換，產生 720x520 的鳥瞰圖
                warped0 = get_perspective_warp(undistorted0, points_cam0, WINDOW_WIDTH, WINDOW_HEIGHT)
                warped2 = get_perspective_warp(undistorted2, points_cam2, WINDOW_WIDTH, WINDOW_HEIGHT)

                tracked0 = False
                tracked2 = False
                if tracking_hsv_range:
                    _, _, tracked0, _, _ = hsv_track_circle(warped0, tracking_hsv_range)
                    _, _, tracked2, _, _ = hsv_track_circle(warped2, tracking_hsv_range)

                if tracked0 and not tracked2:
                    weight_cam0 = 1.0; weight_cam2 = 0.0
                elif tracked2 and not tracked0:
                    weight_cam0 = 0.0; weight_cam2 = 1.0
                elif tracked0 and tracked2:
                    weight_cam0 = 0.5; weight_cam2 = 0.5

                fused = cv2.addWeighted(warped0, weight_cam0, warped2, weight_cam2, 0)
                fused_for_cb = fused.copy()
                cv2.setMouseCallback("Fused Warped View", mouse_callback_fused, fused_for_cb)

                if tracking_hsv_range is not None:
                    result_img, mask, tracked, center, contour_mask = hsv_track_circle(fused, tracking_hsv_range)
                    
                    if tracked and center is not None:
                        pixel_x, pixel_y = center
                        
                        # [關鍵計算] 座標映射
                        # Real_X = Pixel_X * (360 / 720)
                        # Real_Y = Pixel_Y * (260 / 520)
                        real_x = pixel_x * (REAL_WIDTH / WINDOW_WIDTH)
                        real_y = pixel_y * (REAL_HEIGHT / WINDOW_HEIGHT)
                        
                        # 在畫面左上角顯示
                        info_text = f"Real Pos: X={real_x:.1f}, Y={real_y:.1f}"
                        cv2.putText(result_img, info_text, (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                        
                        # 在球體旁邊也顯示 (選用)
                        ball_text = f"({real_x:.0f},{real_y:.0f})"
                        cv2.putText(result_img, ball_text, (pixel_x + 15, pixel_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                        print(f"Pixel:({pixel_x},{pixel_y}) -> Real:({real_x:.1f},{real_y:.1f})")
                    
                    cv2.imshow("Fused Warped View", result_img)
                    cv2.imshow("Mask", mask)
                else:
                    cv2.imshow("Fused Warped View", fused)

    except KeyboardInterrupt:
        pass
    finally:
        cap0.release()
        cap2.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
