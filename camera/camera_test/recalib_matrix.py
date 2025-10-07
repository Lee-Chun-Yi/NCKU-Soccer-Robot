import cv2

cap = cv2.VideoCapture(0)  # 0 是攝影機 index，可以根據實際情況調整

# 嘗試開啟相機
if not cap.isOpened():
    print("無法開啟攝影機")
else:
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("實際相機 FPS:", fps)

cap.release()
