import cv2

def list_available_cameras(max_ports=10):
    available = []
    for i in range(max_ports):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available.append(i)
            cap.release()
    print(f"Available camera ports: {available}")

if __name__ == "__main__":
    list_available_cameras()
