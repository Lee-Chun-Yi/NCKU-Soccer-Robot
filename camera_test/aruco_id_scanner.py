import cv2

def detect_aruco_ids(camera_port=0, dict_type=cv2.aruco.DICT_5X5_100):
    # 初始化相機
    cap = cv2.VideoCapture(camera_port)
    if not cap.isOpened():
        print("無法開啟相機")
        return

    # 初始化 ArUco 字典及檢測參數
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print("按 ESC 鍵結束程式")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("無法讀取影像")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            # 繪製偵測到的標記框
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # 顯示每個標記的 ID
            for id in ids:
                cv2.putText(frame, f"ID: {id[0]}", (10, 30 + 30 * id[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print(f"偵測到標記ID: {ids.flatten()}")

        cv2.imshow("ArUco ID Detection", frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC 鍵退出
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco_ids()