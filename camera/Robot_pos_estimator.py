# ref:https://github.com/ZengWenJian123/aruco_positioning_2D
import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math
#5/8 我量aruco的正方形大小是八公分整
ARUCO_SIZE = 0.08#0.078 #0.053
CAMERA_HEIGHT = 2.05  #2.23(ARUCO放地板) #
#149,102

#ref:https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    # for i in range(len(corners)):
    #     for j, pt in enumerate(corners[i]):
    #         print(f"  corner {j}: {pt}")  # ✅ pt 是 [x, y]



    for c in corners:
        #c 是image pt marker pt是 object pt
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R.flatten())
        tvecs.append(t.flatten())
        #nada 通常是一個布林值，表示求解过程是否成功
        trash.append(nada)
    rvecs = np.array(rvecs)
    rvecs = rvecs.reshape(-1,1,3)
    tvecs = np.array(tvecs)
    tvecs = tvecs.reshape(-1,1,3)
    # print(tvecs.shape)
    return rvecs, tvecs, trash

def rotation_mtx2euler_angle(Rotation_matrix):
    R = Rotation_matrix
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular=sy< 1e-6#解決Singular問題
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.asin(-R[2, 0])
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    #轉成角度
    rx = x * 180.0 / 3.141592653589793
    ry = y * 180.0 / 3.141592653589793
    rz = z * 180.0 / 3.141592653589793

    return rx,ry,rz
def calc_Aruco_distance(tvec):
    ###### 距離估計 #####
    distance_x = (tvec[0][0][0]) * 100 
    distance_y = (tvec[0][0][1]) * 100  
    distance_z = (tvec[0][0][2]) * 100  

    if distance_z:  # 判斷值是否正確
        tmp = distance_y**2 + distance_z**2 - (CAMERA_HEIGHT * 100)**2
        #print(f"distance_y^2 = {distance_y**2}, distance_z^2 = {distance_z**2}, tmp = {tmp}\n")
        if tmp < 0:
            print("[Warning] sqrt 參數為負數，可能導致 NaN")
            print(f"distance_y^2 = {distance_y**2}, distance_z^2 = {distance_z**2}, (CAMERA_HEIGHT*100)^2 = {(CAMERA_HEIGHT*100)**2}")
            print(f"tmp = {tmp}，設為 0 避免錯誤")
            tmp = 0  # 強制為 0 避免 np.sqrt 發生錯誤 #出來座標[0,0] 和沒測到一樣

        distance = np.sqrt(tmp)
        # print("distance", distance, "distance_x", distance_x, "distance_y", distance_y, "distance_z", distance_z)
    else:
        distance = -99999  # 無法計算，回傳異常值

    return distance, distance_x, distance_y, distance_z


def robot_pos_estimation(id,rvecs,tvecs):

    # print("id = ",id)
    rotation_bias = [[1,0,0],
                     [0,1,0],
                     [0,0,1]]


    # new_rvecs = np.ndarray((len(rvecs),3))
#5/14可以看的東西
    rvecs = rvecs[0]
    tvecs = tvecs[0]
    # print("rvecs",rvecs)
    R,_ = cv2.Rodrigues(rvecs[0])
    z_axis = R[:, 2]
    # print(R)
    inv_R = np.linalg.inv(R)
    
    if (id == 0 or id == 1 or id == 10 or id == 11 or id ==  20 or id == 21):
        t_vecs_bias = np.dot(inv_R,[[0],[0],[0]]).flatten()
        # print(f"UP    t_vecs_bias = {t_vecs_bias}\n")
        rotation_bias = [[1,0,0],
                         [0,1,0],
                         [0,0,1]]

    elif (id == 2 or id == 3 or id == 12 or id == 13 or id == 22 or id == 23):#front
        t_vecs_bias = np.dot(inv_R,[[0],[0.05],[-0.05]]).flatten()
        # print(f"FRONT t_vecs_bias = {t_vecs_bias}\n")
        rotation_bias_1 = [[np.cos(np.deg2rad(180)),-np.sin(np.deg2rad(180)),0],
                        [np.sin(np.deg2rad(180)),np.cos(np.deg2rad(180)),0],
                        [0,0,1]]
        
        rotation_bias_2 = [[1,0,0],
                         [0,np.cos(np.deg2rad(90)),-np.sin(np.deg2rad(90))],
                         [0,np.sin(np.deg2rad(90)),np.cos(np.deg2rad(90))]]
        
        rotation_bias = np.dot(rotation_bias_1,rotation_bias_2)
    elif (id == 4 or id == 5 or id == 14 or id == 15 or id == 24 or id == 25):#left
        t_vecs_bias = np.dot(inv_R,[[0],[0.05],[-0.05]]).flatten()
        # print(f"LEFT  t_vecs_bias = {t_vecs_bias}\n")
        
        rotation_bias_1 = [[np.cos(np.deg2rad(90)),0,np.sin(np.deg2rad(90))],
                        [0,1,0],
                        [-np.sin(np.deg2rad(90)),0,np.cos(np.deg2rad(90))]]
        rotation_bias_2 = [[1,0,0],
                         [0,np.cos(np.deg2rad(-90)),-np.sin(np.deg2rad(-90))],
                         [0,np.sin(np.deg2rad(-90)),np.cos(np.deg2rad(-90))]]
             
        rotation_bias = np.dot(rotation_bias_1,rotation_bias_2)
    # elif(id == 8 or id == 9):#back
    elif(id == 6 or id == 7 or id == 16 or id == 17 or id == 26 or id == 27):#back
        # print("go id == 9")
        t_vecs_bias = np.dot(inv_R,[[0],[0.05],[-0.05]]).flatten()
        # print(f"BACK  t_vecs_bias = {t_vecs_bias}\n")
        rotation_bias = [[1,0,0],
                         [0,np.cos(np.deg2rad(-90)),-np.sin(np.deg2rad(-90))],
                         [0,np.sin(np.deg2rad(-90)),np.cos(np.deg2rad(-90))]]

        
    elif(id == 8 or id == 9 or id == 18 or id == 19 or id == 28 or id == 29):#right
        # print("go id == 9")
        t_vecs_bias = np.dot(inv_R,[[0],[0.05],[-0.05]]).flatten()
        # print(f"RIGHT t_vecs_bias = {t_vecs_bias}\n")
        
        rotation_bias_1 = [[1,0,0],
                    [0,np.cos(np.deg2rad(-90)),-np.sin(np.deg2rad(-90))],
                    [0,np.sin(np.deg2rad(-90)),np.cos(np.deg2rad(-90))]]
        
        rotation_bias_2 = [[np.cos(np.deg2rad(-90)),-np.sin(np.deg2rad(-90)),0],
                        [np.sin(np.deg2rad(-90)),np.cos(np.deg2rad(-90)),0],
                        [0,0,1]]

        rotation_bias = np.dot(rotation_bias_1,rotation_bias_2)

    else : 
        t_vecs_bias = np.dot(inv_R,[[0],[0],[0]]).flatten()
        rotation_bias = [[1,0,0],
                         [0,1,0],
                         [0,0,1]]

    #new_tvecs = tvecs + t_vecs_bias 
    new_tvecs = tvecs - (z_axis*0.04)
    new_rvecs,_ = cv2.Rodrigues(np.dot(R,rotation_bias))
    new_rvecs = np.array(new_rvecs)
    new_rvecs = new_rvecs.reshape(-1,1,3)
    new_tvecs = new_tvecs.reshape(-1,1,3)
        
    return new_rvecs,new_tvecs

def display(frame,id_list,degree_list_x,degree_list_y,degree_list_z,distance_list):

    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    cv2.putText(frame,'ids:'+ str(id_list) ,(0, 110), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
    cv2.putText(frame,'deg_x:'+ str(degree_list_x) ,(0, 150), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
    cv2.putText(frame,'deg_y:'+ str(degree_list_y) ,(0, 190), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
    cv2.putText(frame,'deg_z:'+ str(degree_list_z) ,(0, 230), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
    cv2.putText(frame,'distance:'+ str(distance_list) ,(0, 270), font, 1, (0, 255, 0), 2,cv2.LINE_AA)

########畫出六面體########
def draw_robot_cube_and_direction(frame, rvec, tvec, camera_matrix, dist_matrix, size=0.08):
    half = size / 2

    # ✅ 通用格式解包：支援 (1,1,3), (1,3), (3,1), (3,)
    rvec = np.array(rvec, dtype=np.float32).reshape(-1)
    tvec = np.array(tvec, dtype=np.float32).reshape(-1)

    rvec = rvec.reshape(3, 1)
    tvec = tvec.reshape(3, 1)

    # 轉成旋轉矩陣
    R, _ = cv2.Rodrigues(rvec)

    # 將頂部中心往下 half 高度，得出幾何中心
    cube_center_offset = np.array([[0], [0], [-half]], dtype=np.float32)
    cube_center_world = tvec + R @ cube_center_offset


    # 定義立方體的 3D 頂點（以中心為原點）
    cube_points = np.float32([
        [-half, -half, -half],
        [ half, -half, -half],
        [ half,  half, -half],
        [-half,  half, -half],
        [-half, -half,  half],
        [ half, -half,  half],
        [ half,  half,  half],
        [-half,  half,  half]
    ])

    forward_point = np.float32([[0 , size * 2, 0]])

    # 投影 3D 點到影像
    imgpts, _ = cv2.projectPoints(cube_points, rvec, cube_center_world, camera_matrix, dist_matrix)
    forward_imgpt, _ = cv2.projectPoints(forward_point, rvec, cube_center_world, camera_matrix, dist_matrix)

    imgpts = np.int32(imgpts).reshape(-1, 2)
    forward_imgpt = np.int32(forward_imgpt).reshape(-1, 2)

    # 畫 cube 線段
    frame = cv2.drawContours(frame, [imgpts[:4]], -1, (255, 0, 0), 2)  # bottom
    frame = cv2.drawContours(frame, [imgpts[4:]], -1, (0, 255, 0), 2)  # top
    for i in range(4):
        frame = cv2.line(frame, tuple(imgpts[i]), tuple(imgpts[i + 4]), (0, 0, 255), 2)

    # 畫面向箭頭（朝向）
    center_imgpt = np.mean(imgpts, axis=0).astype(int)
    frame = cv2.arrowedLine(frame, tuple(center_imgpt), tuple(forward_imgpt[0]), (0, 255, 255), 3, tipLength=0.2)

    return frame

def get_Aruco_information(camera_matrix,dist_matrix,frame,corners,ids):
    distance_list = []
    distance_x_list = []
    degree_list = []
    degree_list_x = []
    degree_list_y = []
    degree_list_z = []

    id_list = []
    # 拿攝影機的原圖來偵測 所以就算在場外也有拍到
#5/15
    #ban id
    # ALLOWED_IDS = [0,1,4,5,8]# 假設你只允許出現這些 ID（根據機器人設定）
    # filtered = [(i, c) for i, c in zip(ids, corners) if i[0] in ALLOWED_IDS]
    # ids = np.array([i for i, _ in filtered])
    # corners = [c for _, c in filtered]
    #ban id

    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    #    如果找不到id
    if ids is not None:
            for index,i in enumerate(ids):

                rvec, tvec, _ = my_estimatePoseSingleMarkers(corners[index], ARUCO_SIZE, camera_matrix, dist_matrix)
                new_rvec,new_tvec = robot_pos_estimation(ids[index][0],rvec,tvec)

                #5/15
                #print(f"ID {ids[index][0]} tvec: {new_tvec.flatten()} rvec(deg): {rotation_mtx2euler_angle(cv2.Rodrigues(new_rvec)[0])}")


                for i in range(new_rvec.shape[0]):
                    #X: red, Y: green, Z: blue
                    cv2.drawFrameAxes(frame, camera_matrix, dist_matrix, new_rvec, new_tvec, 0.03)
                    frame = aruco.drawDetectedMarkers(frame.copy(), corners, ids,borderColor=(0, 255, 0))
                    frame = draw_robot_cube_and_direction(frame, new_rvec, new_tvec, camera_matrix, dist_matrix)
                R = np.zeros((3,3),dtype=np.float64)
                cv2.Rodrigues(new_rvec,R)
                rx,ry,rz = rotation_mtx2euler_angle(Rotation_matrix=R) #5/10

                ###### 距離估計 #####
                distance,distance_x,distance_y,distance_z = calc_Aruco_distance(tvec=new_tvec)
                # print("distance",distance)
                # print("distance",distance_x)
                distance_list.append(round(distance,2))
                distance_x_list.append(round(distance_x,2))
                degree_list.append(round(rz,2))
                degree_list_x.append(round(rx,2))
                degree_list_y.append(round(ry,2))
                degree_list_z.append(round(rz,2))
                id_list.append(ids[index][0])

    else:
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    # display(frame,id_list,degree_list_x,degree_list_y,degree_list_z,distance_list)
    return frame,id_list,distance_list,distance_x_list,degree_list,degree_list_x,degree_list_y,degree_list_z

if __name__ == '__main__':
    calibration_matrix_path = "./calibration_matrix_right.npy"
    distortion_coefficients_path = "./distortion_coefficients_right.npy"
    camera_matrix = np.load(calibration_matrix_path)
    dist_matrix = np.load(distortion_coefficients_path)

    cap = cv2.VideoCapture(1,cv2.CAP_DSHOW) 
    # cap = cv2.VideoCapture('2024-03-27_19-20-54.mp4') # 讀取電腦中的影片
    # cap = cv2.VideoCapture('2024-04-04_02-57-45.mp4') # 讀取電腦中的影片
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)  # 设置帧率为60帧/秒  
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    #num = 0
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        ret, frame = cap.read()
        h1, w1 = frame.shape[:2]
        #糾正畸變
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, (h1, w1), 1, (h1, w1))
        dst1 = cv2.undistort(frame, camera_matrix, dist_matrix, None, newcameramtx)
        x, y, w1, h1 = roi
        dst1 = dst1[y:y + h1, x:x + w1]
        # frame = dst1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        #使用aruco.detectMarkers()函數可以檢測到marker，返回ID和标志板的4个角点坐標
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        frame,id_list,distance_list,distance_x_list,degree_list,degree_list_x,degree_list_y,degree_list_z\
              = get_Aruco_information(camera_matrix=camera_matrix,dist_matrix=dist_matrix,frame=frame,corners=corners,ids=ids)
        
        display(frame,id_list,degree_list_x,degree_list_y,degree_list_z,distance_list)
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', 1200, 800)
        cv2.imshow("frame",frame)
        key = cv2.waitKey(1)

        if key == 27:
            print('esc break...')
            cap.release()
            cv2.destroyAllWindows()
            break

        if key == ord(' '):
            filename = str(time.time())[:10] + ".jpg"
            cv2.imwrite(filename, frame)