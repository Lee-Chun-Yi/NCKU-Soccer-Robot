import cv2

# 改成你想測的 index（例如 0 或 1 或 2）
camera_index = 0

cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print(f"❌ 無法開啟攝影機（index {camera_index}）")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 無法讀取影像")
        break

    cv2.imshow(f"Camera {camera_index}", frame)

    # 按下 Esc 鍵退出（鍵碼 27）
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
