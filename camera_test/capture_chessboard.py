import cv2
import os

def capture_calibration_images(cam_port=0, save_folder='calibration_images_cam0'):
    # 如果資料夾不存在就建立
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
        print(f"已建立資料夾 {save_folder}")

    cap = cv2.VideoCapture(cam_port)
    if not cap.isOpened():
        print(f"Cannot open camera {cam_port}")
        return

    count = 0
    print(f"開始從攝影機 {cam_port} 擷取校正圖片，按空白鍵拍照，按 q 鍵退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("讀取畫面失敗")
            break

        cv2.imshow(f"Camera {cam_port}", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):  # 按空白鍵時存檔
            count += 1
            filename = os.path.join(save_folder, f"chessboard_{count}.jpg")
            cv2.imwrite(filename, frame)
            print(f"已保存圖片 {filename}")

        elif key == ord('q'):  # 按q退出
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    capture_calibration_images(cam_port=0, save_folder='calibration_images_cam0')
    capture_calibration_images(cam_port=2, save_folder='calibration_images_cam2')


