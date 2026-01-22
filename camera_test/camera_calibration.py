import cv2
import numpy as np
import glob

points_cam0 = []
points_cam2 = []

def mouse_callback_cam0(event, x, y, flags, param):
    global points_cam0
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam0) < 4:
        points_cam0.append((x, y))
        print(f"攝影機0已選頂點 {len(points_cam0)}: {(x, y)}")

def mouse_callback_cam2(event, x, y, flags, param):
    global points_cam2
    if event == cv2.EVENT_LBUTTONDOWN and len(points_cam2) < 4:
        points_cam2.append((x, y))
        print(f"攝影機2已選頂點 {len(points_cam2)}: {(x, y)}")

def camera_calibration_and_point_selection(image_folder='calibration_images', chessboard_size=(8,6), cam_id=0):
    global points_cam0, points_cam2
    points = points_cam0 if cam_id == 0 else points_cam2
    mouse_callback = mouse_callback_cam0 if cam_id == 0 else mouse_callback_cam2

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    objpoints = []
    imgpoints = []
    gray = None

    images = glob.glob(f'{image_folder}/*.jpg')
    if len(images) == 0:
        raise FileNotFoundError(f"資料夾 {image_folder} 中沒有校正圖片！")

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"無法讀取 {fname}，跳過")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow(f'校正角點 Camera {cam_id}', img)
            cv2.waitKey(500)
    cv2.destroyAllWindows()

    if gray is None:
        raise RuntimeError("校正圖片讀取失敗")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(f"Camera {cam_id} 校正成功率: {ret}")
    print(f"Camera {cam_id} 相機內參:\n{mtx}")
    print(f"Camera {cam_id} 畸變係數:\n{dist}")

    np.savez(f'calib_cam{cam_id}.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
    print(f"Camera {cam_id} 校正參數已儲存至 calib_cam{cam_id}.npz")

    cap = cv2.VideoCapture(cam_id)
    ret, frame = cap.read()
    if not ret:
        print(f"Camera {cam_id} 無法讀取攝影機")
        return

    undistorted = cv2.undistort(frame, mtx, dist, None, mtx)
    rotated = cv2.rotate(undistorted, cv2.ROTATE_90_CLOCKWISE)
    winname = f'select_points_cam{cam_id}'
    cv2.namedWindow(winname)
    cv2.imshow(winname, rotated)
    cv2.setMouseCallback(winname, mouse_callback)
    print(f"Camera {cam_id} 請依序用滑鼠左鍵點選四個場地角點")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted = cv2.undistort(frame, mtx, dist, None, mtx)
        rotated = cv2.rotate(undistorted, cv2.ROTATE_90_CLOCKWISE)

        for pt in points:
            cv2.circle(rotated, pt, 5, (0, 0, 255), -1)

        cv2.imshow(winname, rotated)

        if len(points) == 4:
            print(f"Camera {cam_id} 四點已選取:", points)
            break

        if cv2.waitKey(1) & 0xFF == 27:
            print(f"Camera {cam_id} 使用者中止")
            points.clear()
            break

    cap.release()
    cv2.destroyAllWindows()

    np.savez(f'field_points_cam{cam_id}.npz', points=np.array(points))
    print(f"Camera {cam_id} 場地四點已儲存至 field_points_cam{cam_id}.npz")


if __name__ == "__main__":
    # 先校正攝影機0，並點選場地四角點（記得用各自校正圖片資料夾）
    camera_calibration_and_point_selection(image_folder='calibration_images_cam0', cam_id=4)
    # 再校正攝影機2，並點選場地四角點（用攝影機2的校正圖片資料夾）
    camera_calibration_and_point_selection(image_folder='calibration_images_cam2', cam_id=0)

