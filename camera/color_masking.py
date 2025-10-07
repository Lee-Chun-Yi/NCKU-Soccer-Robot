
import cv2
import numpy as np
import json
# # HSV format(8 bytes) => [ 0~180, 0~255, 0~255]

# # define range of blue color in HSV
# lower_blue = np.array([110,100,50]) #lower_blue = np.array([110,50,50])
# upper_blue = np.array([130,255,255])#upper_blue = np.array([130,255,255])

# # define range of green color in HSV 
# lower_green = np.array([50,50,100])
# upper_green = np.array([70,255,255])

# # define range of red color in HSV 
# lower_red_1= np.array([0, 50, 50])
# upper_red_1 = np.array([10, 255, 255])
# lower_red_2 = np.array([150, 50, 50])
# upper_red_2 = np.array([180, 255, 255])

# # define range of white color in HSV 
# lower_white = np.array([0,0,240])
# upper_white = np.array([150,5,255])

# # define range of yellow color in HSV 
# lower_yellow = np.array([20,50,100])
# upper_yellow = np.array([40,255,255])

def colorMasking(frame,color,color_selection_setting = 'custom',camera_selection_setting = 'left'):
    
    if camera_selection_setting == "left":
        file_path = 'color_settings_left.json'
    elif camera_selection_setting == "right":
        file_path = 'color_settings_right.json'
    # print("Select filepath",file_path, "selec color ", color)
    with open(file_path, 'r') as file:
        color_settings = json.load(file)

    settings = color_settings["settings"][color_selection_setting]

    lower_blue = np.array(settings["blue"][0])
    upper_blue = np.array(settings["blue"][1])
    lower_green = np.array(settings["green"][0])
    upper_green = np.array(settings["green"][1])

    lower_red_1 = np.array(settings["red_1"][0])
    upper_red_1 = np.array(settings["red_1"][1])
    lower_red_2 = np.array(settings["red_2"][0])
    upper_red_2 = np.array(settings["red_2"][1])

    lower_white = np.array(settings["white"][0])
    upper_white = np.array(settings["white"][1])

    lower_yellow = np.array(settings["yellow"][0])
    upper_yellow = np.array(settings["yellow"][1])

    lower_orange = np.array(settings["orange"][0])
    upper_orange = np.array(settings["orange"][1])

    lower_skin = np.array(settings["skin"][0])
    upper_skin = np.array(settings["skin"][1])


    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # for i in color:
    # Threshold the HSV image to get only blue colors
    if color == "blue":
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        return mask_blue
    
    elif color == "green":
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        return mask_green
    
    elif color == "red":
        mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red_1, upper_red_1) , cv2.inRange(hsv, lower_red_2, upper_red_2))
        return mask_red
    
    elif color == "white":
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        return mask_white
    
    elif color == "yellow":
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return mask_yellow
    
    elif color == "orange":
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        return mask_orange
    elif color == "skin":
        mask_skin = cv2.inRange(hsv, lower_skin, upper_skin)
        return mask_skin
        
    # return mask_blue, mask_green, mask_red, mask_white, mask_yellow, mask_orange


if __name__ == '__main__':
    # 正確寫法（for Linux）：
    cap = cv2.VideoCapture(2)  # 或改成 2，看你想接哪個攝影機

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    points_path = "./points_left.npy"
    points = np.load(points_path)
    p1 = np.float32(points)
    p2 = np.float32([[0,0],[1080,0],[0,1920],[1080,1920]])
    # m = cv2.getPerspectiveTransform(p1,p2)
    
    if not cap.isOpened():
        print("Cannot open camera")
        exit()


    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # 產生遮罩
        mask = colorMasking(frame=frame, color="orange")
        
        # 套用遮罩到原圖
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # 將彩色圖轉為灰階
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        # 邊緣偵測（可以調整閾值）
        edges = cv2.Canny(gray, threshold1=50, threshold2=150)
        mask_for_circle = cv2.medianBlur(mask, 5)

        # 1. 從邊緣圖找輪廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        edges_vis = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # ✅ 放在 for 外面！

        for cnt in contours:
            if len(cnt) < 40:  # fitEllipse needs at least 5 points
                continue

            ellipse = cv2.fitEllipse(cnt)
            (center, axes, angle) = ellipse
            major_axis, minor_axis = axes
            radius_estimate = (major_axis + minor_axis) / 4  # average radius estimate

            # Filter based on size
            if 20 < radius_estimate < 32:
                # Optionally filter by ellipse aspect ratio (how "round" it is)
                aspect_ratio = minor_axis / major_axis if major_axis != 0 else 0

                # If you're okay with ellipses that are not too "flat":
                if aspect_ratio > 0.5:
                    cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
                    cv2.putText(frame, f"R~{radius_estimate:.1f}", (int(center[0]) - 20, int(center[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


        # ✅ 這樣才能看到所有圈的疊加結果
        cv2.imshow("Edges with Circle", edges_vis)
        cv2.imshow('frame', frame)
        #cv2.imshow('Filtered Result', res)

        if cv2.waitKey(1) == 27:  # 按下 ESC 結束
            break

    cap.release()
    cv2.destroyAllWindows()