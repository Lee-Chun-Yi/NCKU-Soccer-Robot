import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math
from Robot_pos_estimator import my_estimatePoseSingleMarkers,calc_Aruco_distance,rotation_mtx2euler_angle,get_Aruco_information
from color_masking import colorMasking
import json

import challenge2_1 as challenge2
import challenge3_forward as challenge3
import copy
import challenge1_2024_Final as challenge1
#5/8 我量aruco的正方形大小是八公分整
ARUCO_SIZE = 0.08 #0.078 #0.053
CAMERA_DISTANCE = 1.2 #相機在底線後1.2 meter
FIELD_LENGTH = 3.6
FIELD_WIDTH = 2.6 #(meter)
DEGREE_THRESHOLD = 1000000


#5/8 要固定插左邊的兩個藍色usb位置 每次插完有可能會變 所以建議不要動
# CAMERA_INDEX_RIGHT = 0
# CAMERA_INDEX_LEFT = 2
CAMERA_INDEX_RIGHT = "/dev/video0"
CAMERA_INDEX_LEFT = "/dev/video2"

TEAM_COLOR = "blue"
NUM_OF_BLUE_TEAM = 3
NUM_OF_RED_TEAM = 3
SIDE = 1

IS_ENEMY_DETECT = True



# IS_ROI_USED = True
class Robot():
    def __init__(self,id,aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list,camera_setting = "right"):
        self.id = id 
        self.aruco_id_list = aruco_id_list
        self.aruco_x_list = aruco_x_list
        self.aruco_y_list = aruco_y_list
        self.aruco_degree_list = aruco_degree_list
        self.x = 0
        self.y = 0
        self.degree = 0
        self.x_left,self.x_right,self.y_left,self.y_right,self.degree_left,self.degree_right = 0,0,0,0,0,0
        
        self.camera_setting = camera_setting
        self.amount_choose_left = 0
        self.amount_choose_right = 0 
        self.is_enemy = False
    
    def robot_pose_estimation(self):
        index = 999
        # print(self.id)
        # if self.id == 0:
        #     order = [0,6,2,8,4]
        # elif self.id == 1:
        #     order = [1,7,3,5,9]
        # elif self.id == 2:
        #     order = [10,16,12,18,14]
        # elif self.id == 3:
        #     order = [11,17,13,15,19]
        # elif self.id == 4:
        #     order = [20,26,22,28,24]
        # elif self.id == 5:
        #     order = [21,27,23,25,19]
        if self.is_enemy == True:

            if self.aruco_id_list != []:
                # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@",self.aruco_id_list)
                # print("debug",self.aruco_id_list)
                index = 0
            else:
                index = 999
        else:
            if self.id == 0:
                order = [0,6,2,8,4]
            elif self.id == 1:
                order = [1,7,3,5,9]
            elif self.id == 2:
                order = [10,16,12,18,14]
            elif self.id == 3:
                order = [11,17,13,15,19]
            elif self.id == 4:
                order = [20,26,22,28,24]
            elif self.id == 5:
                order = [21,27,23,25,19]

            for id in order:
                if id in self.aruco_id_list:
                    index = self.aruco_id_list.index(id)
                    break

#相機校正
        if(index != 999):#,3,5,7,9,counterclockwise
            # print("index",index,"sef",self.aruco_x_list )
            
            if self.camera_setting == "left":
                # offset
                self.x_left = round(self.aruco_x_list[index] - (CAMERA_DISTANCE*100       ) ,2)
                self.y_left = round((FIELD_WIDTH * 100 // 2 + (self.aruco_y_list[index] )     ) ,2)

                tmp_degree = degree0to360transform(self.aruco_degree_list[index])

                if abs(tmp_degree - self.degree_left) < DEGREE_THRESHOLD:
                    self.degree_left = round(degree0to360transform(self.aruco_degree_list[index]),2)
                else:
                    print("Left camera:robot",self.id," got wrong degree, use data before")

            elif self.camera_setting == "right":
                self.x_right = round(FIELD_LENGTH* 100 - (self.aruco_x_list[index] - (CAMERA_DISTANCE*100 )      ),2)+ 40
                self.y_right = round((FIELD_WIDTH * 100 // 2 - (self.aruco_y_list[index] )       ),2)

                ##
                tmp_degree = degree0to360transform(self.aruco_degree_list[index])
                if abs(tmp_degree - self.degree_right) < DEGREE_THRESHOLD:
                    self.degree_right = degree0to360transform(self.aruco_degree_list[index])
                    self.degree_right = self.degree_right + 180
                    if self.degree_right >= 360:
                        self.degree_right = round(self.degree_right - 360,2)
                else:
                    print("Right camera:robot",self.id," got wrong degree, use data before")
    
    def get_information(self,needed_direction):
        if needed_direction == 'left':
            # self.x = self.x_left
            # self.y = self.y_left
            # self.degree = self.degree_left
            return [self.x_left,self.y_left],self.degree_left

        elif needed_direction == 'right':
            # self.x = self.right
            # self.y = self.right
            # self.degree = self.degree_right
            return [self.x_right,self.y_right],self.degree_right
        
        elif needed_direction == 'overall':
            # self.position_estimator()
            return [self.x,self.y],self.degree
        
    def position_estimator(self):
        self.x = self.x_right
        self.y = self.y_right
        self.degree = self.degree_right
        # print("robot esimator for",self.id)
        # print(self.x_left,self.x_right)
        # distance_from_right_camera = FIELD_LENGTH * 100 - self.x_right 
        # print(distance_from_right_camera)
        # if self.x_right == 0 and self.x_left == 0 :
        #     print("========== Not estimation value for :robot",self.id)

        # elif (self.x_left <= distance_from_right_camera and self.x_left!=0 and self.x_left > 0) or distance_from_right_camera == 0:
        #     print("========== Used the information of LEFT camera :robot",self.id)
        #     self.x = self.x_left
        #     self.y = self.y_left
        #     self.degree = self.degree_left
        #     self.amount_choose_left += 1

        # elif (self.x_left > distance_from_right_camera and distance_from_right_camera!= 0 and distance_from_right_camera > 0) or self.x_left == 0:
        #     print("========== Used the information of RIGHT camera :robot",self.id)
        #     self.x = self.x_right
        #     self.y = self.y_right
        #     self.degree = self.degree_right
        #     self.amount_choose_right += 1

        # else:
        #     print("========== Not consider case :robot",self.id)

        # print("Amount of choose left and right",self.amount_choose_left,self.amount_choose_right)

        
        # self.x = self.x_right
        # self.y = self.x_right
        # self.degree = self.degree_right



    def update_robot_position(self,id,aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list,camera_setting):
        self.camera_setting = camera_setting
        self.id = id
        self.aruco_id_list = aruco_id_list
        self.aruco_x_list = aruco_x_list
        self.aruco_y_list = aruco_y_list
        self.aruco_degree_list = aruco_degree_list
        self.robot_pose_estimation()

class Robot_Team():
    def __init__(self,color,length):
        self.color = color
        self.team_length = length
        self.Robot_list = []
        self.is_enemy = False
        for i in range(length):
            aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list = [],[],[],[]
            self.Robot_list.append(Robot(i,aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list))
#########################################################

    def update_position(self,aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list,camera_setting):
        ##################hard code
        #print("updating position...")
        for i in range(self.team_length):
            if self.is_enemy == True:
                self.Robot_list[i].is_enemy = True

        for robot_id in range(self.team_length):
            slice_aruco_id_list,slice_aruco_x_list,slice_aruco_y_list,slice_aruco_degree_list = [],[],[],[]
            for index,aruco_id in enumerate(aruco_id_list):
                if aruco_id >= 10*(robot_id) and aruco_id < 10*(robot_id + 1):
                    slice_aruco_id_list.append(aruco_id_list[index])
                    slice_aruco_x_list.append(aruco_x_list[index])
                    slice_aruco_y_list.append(aruco_y_list[index])
                    slice_aruco_degree_list.append(aruco_degree_list[index])

                robot_uni_id = robot_id
                if(self.color == "red"):
                    # print(self.color)
                    robot_uni_id = 2*robot_id + 1
                else:
                    robot_uni_id  = 2*robot_id

                self.Robot_list[robot_id].update_robot_position(robot_uni_id,slice_aruco_id_list,slice_aruco_x_list,slice_aruco_y_list,slice_aruco_degree_list,camera_setting)
    
    def get_information(self,needed_direction):
        # print("Team:",self.color," num:",len(self.Robot_list),"\n")
        id_result = []
        x_result = []
        y_result = []
        pos_result = []
        degree_result = []
        for robot in self.Robot_list:
            if needed_direction == "left":
                #5/15未解決
                #print("[left]robot:",robot.id," x:",robot.x_left," y:",robot.y_left," degree",robot.degree_left)
                id_result.append(robot.id)
                # x_result.append(robot.x_left)
                # y_result.append(robot.y_left)
                pos_result.append([robot.x_left,robot.y_left])
                degree_result.append(robot.degree_left)
            elif needed_direction == "right":
                #5/15未解決
                #print("[right]robot:",robot.id," x:",robot.x_right," y:",robot.y_right," degree",robot.degree_right)
                id_result.append(robot.id)
                # x_result.append(robot.x_right)
                # y_result.append(robot.y_right)
                pos_result.append([robot.x_right,robot.y_right])
                degree_result.append(robot.degree_right)
                # return robot.id_right,robot.x_right,robot.y_right,robot.degree_right

            elif needed_direction == "overall":
                robot.get_information(needed_direction)
                #5/15未解決
                #print("[overall]robot:",robot.id," x:",robot.x," y:",robot.y," degree",robot.degree)
                # return robot.id,robot.x,robot.y,robot.degree
                id_result.append(robot.id)
                # x_result.append(robot.x)
                # y_result.append(robot.y)
                pos_result.append([robot.x,robot.y])
                degree_result.append(robot.degree)
        
        return id_result,pos_result,degree_result



class Ball():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.x_left = 0
        self.x_right = 0
        self.y_left = 0
        self.y_right = 0
        self.previous_positions = deque(maxlen=5)

    def update_position(self,frame,camera_setting):
        font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
        mask = colorMasking(frame,"orange",color_selection_setting='custom',camera_selection_setting=camera_setting)
        mask = cv2.medianBlur(mask, 5)  # 中值濾波
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 抓出輪廓
        datacenter = []  # 儲存球心座標
        dataradius = []  # 儲存半徑座標
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)  # 取出輪廓外接的最小正矩形座標
            # print("check",w,h)
            #1300 2500
            if w * h > 1000 and w * h < 4500 and w < 3 * h and h < 3 * w:  # 過濾面積小於150的矩形
                (xx, yy), radius = cv2.minEnclosingCircle(cnt)  # 找出最小外接圓
                ball_x = int(xx)
                ball_y = int(yy)
                center = [ball_x,ball_y]
                radius = int(radius)
                # print(radius,"radius")
                # if radius > 15 and int((ball_y / 1080) * FIELD_WIDTH * 100) < 140:
                if 50 > radius > 5:
                    cv2.circle(frame, center, radius, (0,255,0), 3)  # 畫圓
                    cv2.putText(frame, "Ball", (x, y - 5), font, 0.7, (0,255,0), 2)  # 螢幕上寫出"Ball"
                    datacenter.append(center)  # 儲存球心座標
                    dataradius.append(radius)  # 儲存半徑座標
                    
                    # print(center)
        #只取第一個
        # print("datacenter",contours)
        if (datacenter):#這裡可以做一個要求只在范圍內的過濾
            if camera_setting =="right":
                ballcenter = datacenter[0]
                # print(ballcenter)
                # offset
                transform_ball_x = int((ballcenter[0] / 1920) * FIELD_LENGTH * 100) +3
                transform_ball_y = int((ballcenter[1] / 1080) * FIELD_WIDTH * 100) 
                [self.x_right,self.y_right] = [transform_ball_x,transform_ball_y]
            
            elif camera_setting == "left":
                ballcenter = datacenter[0]
                # print(ballcenter)
                transform_ball_x = int((ballcenter[0] / 1920) * FIELD_LENGTH * 100) -3
                transform_ball_y = int((ballcenter[1] / 1080) * FIELD_WIDTH * 100)
                [self.x_left,self.y_left] = [transform_ball_x,transform_ball_y]


                
    def get_information(self,needed_direction):

        if needed_direction == 'left':
            return [self.x_left,self.y_left]

        elif needed_direction == 'right':
            return [self.x_right,self.y_right]
        
        elif needed_direction == 'overall':
            if self.x_right == 0 and self.y_right == 0 :
                return [self.x_left,self.y_left]
            
            elif self.x_left == 0 and self.y_left == 0 :
                return [self.x_right,self.x_right]
        
            else : 
                print("Ball position all equal to Zero")
   
class Camera():
    def __init__(self,index,number,video_path = False):
        self.index = index
        self.video_path = video_path
        self.number = number
        self.camera_setting()

    def __call__(self):
        return self.cap
    
    def camera_setting(self):
        if SIDE == 1:
            if self.index == "left":
                # calibration_matrix_path = "./calibration_matrix_left.npy"
                # distortion_coefficients_path = "./distortion_coefficients_left.npy"
                calibration_matrix_path = "./calibration_matrix_left.npy"
                distortion_coefficients_path = "./distortion_coefficients_left.npy"
            elif self.index == "right":
                calibration_matrix_path = "./calibration_matrix_right.npy"
                distortion_coefficients_path = "./distortion_coefficients_right.npy"

        elif SIDE == -1 :#(相機調換了 但不需要做重新校正)
            if self.index == "right":
                # calibration_matrix_path = "./calibration_matrix_left.npy"
                # distortion_coefficients_path = "./distortion_coefficients_left.npy"
                calibration_matrix_path = "./calibration_matrix_left.npy"
                distortion_coefficients_path = "./distortion_coefficients_left.npy"

            elif self.index == "left":
                calibration_matrix_path = "./calibration_matrix_right.npy"
                distortion_coefficients_path = "./distortion_coefficients_right.npy"

        self.camera_matrix = np.load(calibration_matrix_path)
        self.dist_matrix = np.load(distortion_coefficients_path)

        # cap = cv2.VideoCapture(self.number,cv2.CAP_DSHOW) 
        cap = cv2.VideoCapture(self.number)
        print(f"Trying to open camera {self.number}...")


        if not cap.isOpened():
            print(f"[ERROR] Failed to open camera {self.number}")
            exit()

        # if (self.video_path):
        # # cap = cv2.VideoCapture('2024-03-27_19-20-54.mp4') # 讀取電腦中的影片
        #     cap = cv2.VideoCapture(self.video_path) # 讀取電腦中的影片
            

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS, 30)  # 最高29幀 #5/18

        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        
        self.cap = cap

################################################################
#smooth position
from collections import deque

# 中值濾波器：每個 ID 有自己的緩衝區
from collections import deque

class MedianFilterPerAgent:
    def __init__(self, num_agents, window_size):
        self.window_size = window_size
        self.agent_buffers = [deque(maxlen=window_size) for _ in range(num_agents)]

    def add(self, agent_id, pos, angle):
        self.agent_buffers[agent_id].append((pos, angle))  # 每筆都是 (pos, angle)

    def get_median(self, agent_id):
        buf = list(self.agent_buffers[agent_id])
        if not buf:
            return [0, 0], [0, 0]

        # 收集所有 x 值
        x_list = [item[0][0] for item in buf]

        # 找 x 值的中位數
        median_x = np.median(x_list)

        # 從原本 buffer 中找出第一筆 x == median_x 的資料
        for pos, angle in buf:
            if np.isclose(pos[0], median_x):
                return pos, angle
        # 若找不到（極少見），回傳最後一筆備用
        return buf[-1]


# 初始化：每隊三人（index 0~2）,每人五個數據取中位數
team_filter = MedianFilterPerAgent(num_agents=3, window_size=7) #還是七個？
oppo_filter = MedianFilterPerAgent(num_agents=3, window_size=7)




 
# class Aruco():
#     def __init__(self):
#         #### when enemy dict is known
#         # self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
#         # self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
#         #### when enemy dict isnot know
#         aruco_dict_1 = cv2.aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
#         aruco_dict_2 = cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
#         self.dictionaries = [aruco_dict_1, aruco_dict_2]

    
#     def update(self,frame):
#         #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         #使用aruco.detectMarkers()函數可以檢測到marker，返回ID和标志板的4个角点坐標
#         corners, ids,rejectedImgPoints = self.detect_and_merge_markers(frame, self.dictionaries)
#         return corners,ids,rejectedImgPoints
  
#     def detect_and_merge_markers(self,frame, dictionaries):
#         all_corners = []
#         all_ids = []
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         parameters = cv2.aruco.DetectorParameters_create()
#         #5/14

#         for dictionary in dictionaries:
#             detector = cv2.aruco.ArucoDetector(dictionary, parameters)
#             # detector_2 = cv2.aruco.ArucoDetector(aruco_dict_2, parameters)
#             corners, ids, rejected = detector.detectMarkers(gray)
#             if ids is not None:
#                 all_corners.extend(corners)
#                 # 直接保留NumPy数组格式
#                 if len(all_ids) == 0:
#                     # print("all_ids 是空的")
#                     all_ids = ids
#                 else:
#                     all_ids = np.vstack((all_ids, ids))

#         # print(all_corners,all_ids)
#         return all_corners, all_ids,rejected
import cv2
import numpy as np

class Aruco():
    def __init__(self):
        # 建立多個字典
        self.dictionaries = [
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100),
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        ]
        self.parameters = cv2.aruco.DetectorParameters()

    def update(self, frame):
        corners, ids, rejectedImgPoints = self.detect_and_merge_markers(frame, self.dictionaries)
        return corners, ids, rejectedImgPoints

    def detect_and_merge_markers(self, frame, dictionaries):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        all_corners = []
        all_ids = []
        all_rejected = []

        for dictionary in dictionaries:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, dictionary, parameters=self.parameters
            )
            if ids is not None:
                all_corners.extend(corners)
                if len(all_ids) == 0:
                    all_ids = ids
                else:
                    all_ids = np.vstack((all_ids, ids))
            if rejected is not None:
                all_rejected.extend(rejected)

        return all_corners, all_ids, all_rejected


def degree2unit_vector(angle_deg):
    angle_rad = math.radians(angle_deg)
    x_component = math.cos(angle_rad)
    y_component = math.sin(angle_rad)

    # Normalize the components to get the unit vector
    length = math.hypot(x_component, y_component)
    unit_vector = [x_component / length, y_component / length]
    # print("length == ",length)
    return unit_vector
    
def degree0to360transform(ori_degree):
    if ori_degree >= 0:
        return ori_degree
    elif ori_degree < 0:
        
        new_degree = 360 + ori_degree
        return new_degree

def gamma_correction(frame, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
    	# apply gamma correction using the lookup table
	return cv2.LUT(frame, table)



team_pos = []
oppo_pos = []
team_degree = []
oppo_degree = []
ball_center = []
new_team_pos = [[0, 0] for _ in range(3)]
new_team_degree = [[0, 0] for _ in range(3)]
new_oppo_pos = [[0, 0] for _ in range(3)]
new_oppo_degree = [[0, 0] for _ in range(3)]

def image_result():
    new_kick_point = [0,0]
    new_first_arri = [0,0]
    # camera1 = Camera("left",number=0,video_path="WIN_20240327_19_20_56_Pro.mp4")
    # camera2 = Camera("right",number=1,video_path="2024-03-27_19-20-54.mp4")
    ####################################################################################
    ### number is the camera index used by cv2.VideoCapture()
    if SIDE == 1:
        camera1 = Camera("left",number=CAMERA_INDEX_LEFT,video_path=False)
        camera2 = Camera("right",number=CAMERA_INDEX_RIGHT,video_path=False)

    if SIDE == -1:
        camera1 = Camera("left",number=CAMERA_INDEX_RIGHT,video_path=False)
        camera2 = Camera("right",number=CAMERA_INDEX_LEFT,video_path=False)

    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    red_team = Robot_Team(color='red',length = NUM_OF_RED_TEAM)
    blue_team = Robot_Team(color='blue',length = NUM_OF_BLUE_TEAM)
    if(IS_ENEMY_DETECT):
        if TEAM_COLOR == 'blue':
            red_team.is_enemy = True
        elif TEAM_COLOR == 'red':
            blue_team.is_enemy == True
    ball = Ball()
    aruco_define = Aruco()
    camera_list = [camera1,camera2]
    frame_left,frame_right = 0,0
    
   
    # num = 0

    while True:
        for camera in camera_list:
            # 5/8 
            # 把left/right camera ban掉
            # if camera.index != "right":
            #     continue  
            # 5/8
            cap = camera()
            ret, frame = cap.read()

            if not ret or frame is None:
                print(f"[ERROR] Failed to read from camera {camera.index}")
                continue  # 或 break，看你是否還想繼續跑其他鏡頭

            h1, w1 = frame.shape[:2]
            #糾正畸變
            # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera.camera_matrix, camera.dist_matrix, (h1, w1), 1, (h1, w1))
            # dst1 = cv2.undistort(frame, camera.camera_matrix, camera.dist_matrix, None, newcameramtx)
            # x, y, w1, h1 = roi
            # dst1 = dst1[y:y + h1, x:x + w1]
            # frame = dst1
            ####################### hard code
            if camera.index == "right":
                ball.x_right = 0
                ball.y_right = 0
            elif camera.index == "left":
                ball.x_left = 0
                ball.y_left = 0
            #######################
            if SIDE == 1:
                if camera.index == "right":
                    points_path = "./points_right.npy"
                elif camera.index == "left":
                    points_path = "./points_left.npy"

            elif SIDE == -1 :
                if camera.index == "right":
                    points_path = "./points_right_another_side.npy"
                elif camera.index == "left":
                    points_path = "./points_left_another_side.npy"

            
            points = np.load(points_path)
            p1 = np.float32(points)
            p2 = np.float32([[0,0],[1080,0],[0,1920],[1080,1920]])
            m = cv2.getPerspectiveTransform(p1,p2)
            
            frame = gamma_correction(frame, gamma=0.5)
            
            corners,ids,rejectedImgPoints = aruco_define.update(frame = frame)

            frame,id_list,distance_list,distance_x_list,degree_list,degree_list_x,degree_list_y,degree_list_z\
                = get_Aruco_information(camera_matrix=camera.camera_matrix,dist_matrix=camera.dist_matrix,frame=frame,corners=corners,ids=ids)
            # print("dis",distance_list,distance_x_list)
            red_id_list,blue_id_list,red_x_list,blue_x_list,red_y_list,blue_y_list,red_degree_list,blue_degree_list = [],[],[],[],[],[],[],[]
#5/14
            #print("id list",id_list)
            for index,id in enumerate(id_list):
                if id % 2 == 0:
                    #even,blue team
                    blue_id_list.append(id)
                    blue_x_list.append(distance_list[index])
                    blue_y_list.append(distance_x_list[index])
                    blue_degree_list.append(degree_list[index])
                    #print("blue_id_list",blue_id_list)
                    #print("blue_x_list",blue_x_list)
                    #print("blue_y_list",blue_y_list)
                    #print("blue_degree_list",blue_degree_list)

                else:
                    red_id_list.append(id)
                    red_x_list.append(distance_list[index])
                    red_y_list.append(distance_x_list[index])
                    red_degree_list.append(degree_list[index])
                    # print("red_id_list",red_id_list)
                    # print("red_x_list",red_x_list)
                    # print("red_y_list",red_y_list)
                    # print("red_degree_list",red_degree_list)

            blue_team.update_position(blue_id_list,blue_x_list,blue_y_list,blue_degree_list,camera_setting=camera.index)
            red_team.update_position(red_id_list,red_x_list,red_y_list,red_degree_list,camera_setting=camera.index)

            # red_team.get_information(needed_direction=camera.index)
            id_result_blue,pos_blue,degree_result_blue = blue_team.get_information(needed_direction=camera.index)
            id_result_red,pos_red,degree_result_red = red_team.get_information(needed_direction=camera.index)

            # frame_resized = cv2.resize(frame, (0, 0), fx=1000, fy=800)
            frame = cv2.warpPerspective(frame, m, (1080, 1920))
            if camera.index == "left":
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                frame_left = frame
            elif camera.index =="right":
                # pass
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frame_right = frame

            ball.update_position(frame = frame,camera_setting = camera.index)
            ball_center_each_side = ball.get_information(needed_direction = camera.index)


            transform_degree_red = []
            transform_degree_blue = []
            for degree in degree_result_blue:
                # degree = degree0to360transform(degree)
                degree = degree2unit_vector(degree)
                transform_degree_blue.append(degree)

            for degree in degree_result_red:
                # degree = degree0to360transform(degree)
                degree = degree2unit_vector(degree)
                transform_degree_red.append(degree)

            cv2.putText(frame,'Blue:' ,(0, 30), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'ids:'+ str(id_result_blue) ,(0, 70), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame, 'coordinates:' + str([[round(int(x), 0) for x in p] for p in pos_blue]), (0, 110), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame,'deg_z:'+ str(degree_result_blue) ,(0, 150), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'Red:' ,(0, 230), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'ids:'+ str(id_result_red) ,(0, 270), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame, 'coordinates:' + str([[round(int(x), 0) for x in p] for p in pos_red]), (0, 310), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame,'deg_z:'+ str(degree_result_red) ,(0, 350), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'ballcenter:'+ str(ball_center_each_side) ,(0, 430), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
    ############################################################################################# draw
            # ball_pos_in_frame = change_coordinate_for_draw([[ball.x,ball.y]])
            # draw_the_point(frame, ball_pos_in_frame, "ball_pos")
            # pos_red_in_frame = change_coordinate_for_draw(pos_red)
            # draw_the_point(frame, pos_red_in_frame, "pos_red")
            # pos_blue_in_frame = change_coordinate_for_draw(pos_blue)
            # draw_the_point(frame, pos_blue_in_frame, "pos_blue")
            
            if challenge3.first_arri != [0, 0]:
                new_first_arri = challenge3.first_arri
                arri_pos_in_frame = change_coordinate_for_draw([new_first_arri])
                draw_the_point(frame, arri_pos_in_frame, "fp", [new_first_arri])
            
            if challenge3.kick_point != [0, 0]:  # new_kick_point 如果上一秒沒有測到球,會保留之前的kick point
                new_kick_point = challenge3.kick_point
                arri_pos_in_frame = change_coordinate_for_draw([new_kick_point])
                draw_the_point(frame, arri_pos_in_frame, "k", [new_kick_point])
            # if challenge2.kick_goal != [0, 0]:
            #     kick_goal_in_frame = change_coordinate_for_draw([challenge2.kick_goal])#[challenge2.kick_goal[0], challenge2.kick_goal[1]]
            #     draw_the_point(frame, kick_goal_in_frame, "Goal")
            # if challenge2.kick_point != [0, 0]:
            #     kick_point_in_frame = change_coordinate_for_draw([challenge2.kick_point])
            #     draw_the_point(frame, kick_point_in_frame, "kick_point")
            # if challenge1.first_arri != [0, 0]:
            #     arri_pos_in_frame = change_coordinate_for_draw([challenge1.first_arri])
            #     draw_the_point(frame, arri_pos_in_frame, "first_arri")
            # if challenge1.kick_goal != [0, 0]:
            #     kick_goal_in_frame = change_coordinate_for_draw([challenge1.kick_goal])
            #     draw_the_point(frame, kick_goal_in_frame, "Goal")
            # if challenge1.kick_point != [0, 0]:
            #     kick_point_in_frame = change_coordinate_for_draw([challenge1.kick_point])
            #     draw_the_point(frame, kick_point_in_frame, "kick_point")
            if challenge3.player_p[2] != [0, 0]:
                player_posi_in_frame = change_coordinate_for_draw([challenge3.player_p[2]])
                draw_the_point(frame, player_posi_in_frame, "player", [challenge3.player_p[2]])
            if challenge3.player_p[2] != [0, 0] and challenge3.player_d[2] != [0, 0]:
                # 計算方向點（位置 + 方向向量）
                player_dir = [
                    challenge3.player_p[2][0] + challenge3.player_d[2][0]*50,
                    challenge3.player_p[2][1] + challenge3.player_d[2][1]*50
                ]

                # 投影到畫面座標
                player_dir_in_frame = change_coordinate_for_draw([player_dir])

                # 畫出來
                draw_the_point(frame, player_dir_in_frame, "player_dir", [player_dir])

    ############################################################################################# draw

            
            cv2.namedWindow('frame '+ camera.index, cv2.WINDOW_NORMAL)
            
            # cv2.resizeWindow('frame ' + camera.index, 720, 520)
           
            cv2.imshow('frame ' + camera.index,frame)
            


        # camera_select = camera_selection(frame_left = frame_left,frame_right = frame_right,color_selection_setting='default',team_color = TEAM_COLOR)
        ball_left = ball.get_information(needed_direction="left")
        ball_right = ball.get_information(needed_direction="right")
        camera_select = camera_selection_use_ball(ball_left = ball_left, ball_right = ball_right)
        # camera_select = "right"
        # id_result_blue,pos_blue,degree_result_blue = blue_team.get_information(needed_direction="overall")
        # id_result_red,pos_red,degree_result_red = red_team.get_information(needed_direction="overall")
        print("Camera select",camera_select)
        if camera_select == 'left':
            id_result_blue,pos_blue,degree_result_blue = blue_team.get_information(needed_direction="left")
            id_result_red,pos_red,degree_result_red = red_team.get_information(needed_direction="left")
            ballcenter_selected = ball.get_information(needed_direction=camera_select)
        elif camera_select == 'right':
            id_result_blue,pos_blue,degree_result_blue = blue_team.get_information(needed_direction="right")
            id_result_red,pos_red,degree_result_red = red_team.get_information(needed_direction="right")
            ballcenter_selected = ball.get_information(needed_direction=camera_select)
        else:
            print("Not informmation from both camera")

        transform_degree_red = []
        transform_degree_blue = []
        for degree in degree_result_blue:
            # degree = degree0to360transform(degree)
            degree = degree2unit_vector(degree)
            transform_degree_blue.append(degree)

        for degree in degree_result_red:
            # degree = degree0to360transform(degree)
            degree = degree2unit_vector(degree)
            transform_degree_red.append(degree)

        global team_degree,team_pos,oppo_pos,oppo_degree,ball_center
        ball_center = ballcenter_selected
        # print("team_pos_im",team_pos)
        # print("pos_red",pos_red)
        if TEAM_COLOR == 'blue':
            team_degree = transform_degree_blue
            team_pos = pos_blue
            oppo_pos = pos_red
            # oppo_pos_alter = enemy_detect(frame=frame,activate=False,enemy_color='red',color_selection_setting='custom',camera_selection_setting=camera_select,ball_pos=ball_center)
            # if (oppo_pos_alter):
            #     for i in range(len(oppo_pos_alter)):
            #         oppo_pos[i] = oppo_pos_alter[i]
            #         if i > 2:
            #             break
            #     print("Enemy detect activate",oppo_pos)
            oppo_degree = transform_degree_red

        elif TEAM_COLOR == 'red':
            team_degree = transform_degree_red
            team_pos = pos_red
            oppo_pos = pos_blue
            # oppo_pos_alter = enemy_detect(frame=frame,activate=True,enemy_color='blue',color_selection_setting='custom',camera_selection_setting=camera_select,ball_pos=ball_center)
            # if (oppo_pos_alter):
            #     for i in range(oppo_pos_alter):
            #         oppo_pos[i] = oppo_pos_alter[i]
            #         if i > 2:
            #             break
            #     print("Enemy detect activate",team_pos)
            oppo_degree = transform_degree_blue
            

        else: 
            raise ValueError("Please set the color of team correctly")
        
        #######################################testing function##############################
        # finder.addValues(team_pos[0],team_pos[1],team_degree[0],team_degree[1],oppo_pos[0],oppo_pos[1],oppo_degree[0],oppo_degree[1],ball_center[0],ball_center[1])
        # team_pos[0],team_pos[1],team_degree[0],team_degree[1],oppo_pos[0],oppo_pos[1],oppo_degree[0],oppo_degree[1],ball_center[0],ball_center[1] = finder.findMedians()

        # finder.addValues([team_pos[0][0],team_pos[0][1]])
        # output = finder.findMedians()
        # # print(finder.findMedians())
        # team_pos[0][0] = output[0]
        # team_pos[0][1] = output[1]    

        # 更新中值緩衝（只加入非 [0,0] 的資料）
        for i in range(3):
            if team_pos[i] != [0, 0]:
                team_filter.add(i, team_pos[i], team_degree[i])
            if oppo_pos[i] != [0, 0]:
                oppo_filter.add(i, oppo_pos[i], oppo_degree[i])


        # 套用中值結果
        for i in range(3):
            new_team_pos[i], new_team_degree[i] = team_filter.get_median(i)
            new_oppo_pos[i], new_oppo_degree[i] = oppo_filter.get_median(i)


        print("team pos   ", [[int(round(x)) for x in pair] for pair in team_pos])
        print("team angle", [[f"{val:.3f}" for val in pair] for pair in team_degree])

        if IS_ENEMY_DETECT == True:
            print("enemy pos  ", [[int(round(x)) for x in pair] for pair in new_oppo_pos])
            print("team angle", [[f"{val:.3f}" for val in pair] for pair in new_oppo_degree])
            print("============================================================")
        key = cv2.waitKey(1)

        if key == 27:
            print('esc break...')
            cap.release()
            cv2.destroyAllWindows()
            break

        if key == ord(' '):
            filename = str(time.time())[:10] + ".jpg"
            cv2.imwrite(filename, frame)
    


H_value_list = []
S_value_list = []
V_value_list = []

def enemy_detect(frame,activate,enemy_color,color_selection_setting,camera_selection_setting,ball_pos):
    if not activate:
        return False
    else : 
        mask= colorMasking(frame=frame,color=enemy_color,color_selection_setting=color_selection_setting,camera_selection_setting=camera_selection_setting)
        mask = cv2.medianBlur(mask, 5)  # 中值濾波
        datacolor,box_list = findcolor(mask=mask)
        print("enemy_pos",datacolor)
        return datacolor


    

def create_callback(frame):
    def get_hsv_value(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 获取点击位置的HSV值
            global H_value_list,S_value_list,V_value_list
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_value = hsv_image[y, x]
            H_value_list.append(int(hsv_value[0]))
            S_value_list.append(int(hsv_value[1]))
            V_value_list.append(int(hsv_value[2]))

            print("HSV Value",hsv_value)
    return get_hsv_value

def write_to_json(color,lower_hsv,upper_hsv,camera_selection_setting):
    if camera_selection_setting == "left":
        file_path = 'color_settings_left.json'
    elif camera_selection_setting == "right":
        file_path = 'color_settings_right.json'

    with open(file_path, 'r') as file:
        color_settings = json.load(file)

    transform_lower_hsv = []
    transform_upper_hsv = []

    for item in lower_hsv:
        item = int(item)
        transform_lower_hsv.append(item)

    for item in upper_hsv:
        item = int(item)
        transform_upper_hsv.append(item)

    custom_setting = color_settings["settings"]["custom"]

    custom_setting[color][0] = transform_lower_hsv
    custom_setting[color][1] = transform_upper_hsv
    print(color_settings)

    with open(file_path, 'w') as file:
        json.dump(color_settings, file,indent= 4 ) 

def get_hsv_range(color,camera_selection_setting):
    # print(H_value_list)
    H_max = np.max(H_value_list)
    H_min = np.min(H_value_list)
    # print(S_value_list)
    S_max = np.max(S_value_list)
    S_min = np.min(S_value_list)
    # print(V_value_list)
    V_max = np.max(V_value_list)
    V_min = np.min(V_value_list)
    print("camera from",camera_selection_setting)
    color_range = {
        "orange": [
            [9, 169, 129],
            [12, 214, 244]
        ]
    }

    print(json.dumps(color_range, indent=4))
    if 5>H_min or H_max>20:
        print('❌ a little bit weird. Try again,plz .')

    # if color == 'red':
    #         # hard code
    #         if H_max > 120:
    #             upper_H_red_min = H_min
    #             lower_hsv = [upper_H_red_min,S_min,V_min]
    #             upper_hsv = [H_max,S_max,V_max]
    #             color = 'red_2'
    #             print("Write choosen HSV of ",color)
    #             write_to_json(color,lower_hsv,upper_hsv,camera_selection_setting = camera_selection_setting)

    #         elif H_max < 120:
    #             lower_H_red_max = H_max
    #             lower_hsv = [H_min,S_min,V_min]
    #             upper_hsv = [lower_H_red_max,S_max,V_max]
    #             color = 'red_1'
    #             print("Write choosen HSV of ",color)
    #             write_to_json(color,lower_hsv,upper_hsv,camera_selection_setting = camera_selection_setting)

    #         else:
    #             print("Error : Unconsider situation when get hsv range of red")
    #             # raise AttributeError("Error : Unconsider situation when get hsv range of red")        
    # else:
    #     lower_hsv = [H_min,S_min,V_min]
    #     upper_hsv = [H_max,S_max,V_max]
    #     write_to_json(color,lower_hsv,upper_hsv,camera_selection_setting = camera_selection_setting)

def HSVdetection(color,camera_selection_setting):
    ##### hard code

    if SIDE == 1:
        if camera_selection_setting == 'right':
            camera_HSV = Camera("right",number=CAMERA_INDEX_RIGHT,video_path=False)###VideoCapture
            points_path = "./points_right.npy"
        elif camera_selection_setting == 'left':
            camera_HSV = Camera("left",number=CAMERA_INDEX_LEFT,video_path=False)###VideoCapture
            points_path = "./points_left.npy"

    elif SIDE == -1 :
        if camera_selection_setting == "right":
            camera_HSV = Camera("right",number=CAMERA_INDEX_LEFT,video_path=False)###VideoCapture
            points_path = "./points_right_another_side.npy"
        elif camera_selection_setting == "left":
            points_path = "./points_left_another_side.npy"
            camera_HSV = Camera("left",number=CAMERA_INDEX_RIGHT,video_path=False)###VideoCapture


    points = np.load(points_path)    
    p1 = np.float32(points)
    p2 = np.float32([[0,0],[1080,0],[0,1920],[1080,1920]])
    m = cv2.getPerspectiveTransform(p1,p2)
    cap = camera_HSV()
    frame_name = "HSV detection"

    while True:
        # 從攝像頭讀取一幀
        ret, frame = cap.read()
        frame = gamma_correction(frame, gamma=0.5) 
        frame = cv2.warpPerspective(frame, m, (1080, 1920))
        
        # 如果讀取失敗，跳出循環
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        if camera_selection_setting == "left":
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif camera_selection_setting =="right":
            # pass
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
        cv2.namedWindow(frame_name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(frame_name , 1000, 700)
        cv2.imshow(frame_name, frame)
        
        cv2.setMouseCallback(frame_name, create_callback(frame=frame))
        if cv2.waitKey(1) == 27:
            cap.release()
            cv2.destroyAllWindows()
            break

    get_hsv_range(color = color , camera_selection_setting = camera_selection_setting)  

def ColorMask(color,color_selection_setting,camera_selection_setting):
    if  SIDE == 1:
        if camera_selection_setting == 'right':
            camera_mask = Camera("right",number=CAMERA_INDEX_RIGHT,video_path=False)
            points_path = "./points_right.npy"

        elif camera_selection_setting == 'left':
            camera_mask = Camera("left",number=CAMERA_INDEX_LEFT,video_path=False)###VideoCapture
            points_path = "./points_left.npy"
    
    elif SIDE == -1:
        if camera_selection_setting == 'left':
            camera_mask = Camera("left",number=CAMERA_INDEX_RIGHT,video_path=False)
            points_path = "./points_left_another_side.npy"

        elif camera_selection_setting == 'right':
            camera_mask = Camera("right",number=CAMERA_INDEX_LEFT,video_path=False)###VideoCapture
            points_path = "./points_right_another_side.npy"

    points = np.load(points_path)    
    p1 = np.float32(points)
    p2 = np.float32([[0,0],[1080,0],[0,1920],[1080,1920]])
    m = cv2.getPerspectiveTransform(p1,p2)

    cap = camera_mask()
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    while True:
        ret, frame = cap.read()
        frame = gamma_correction(frame, gamma=0.5)
        frame = cv2.warpPerspective(frame, m, (1080, 1920))
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        if camera_selection_setting == "left":
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif camera_selection_setting =="right":
            # pass
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
        mask= colorMasking(frame=frame,color=color,color_selection_setting=color_selection_setting,camera_selection_setting=camera_selection_setting)
        mask = cv2.medianBlur(mask, 5)  # 中值濾波
        res = cv2.bitwise_and(frame,frame, mask= mask)

        cv2.namedWindow('frame '+ camera_mask.index, cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame '+ camera_mask.index , 600, 400)
        cv2.imshow('frame '+ camera_mask.index ,frame)

        cv2.namedWindow('mask '+ camera_mask.index, cv2.WINDOW_NORMAL)
        cv2.resizeWindow('mask '+ camera_mask.index, 600, 400)
        cv2.imshow('mask '+ camera_mask.index , mask)

        cv2.namedWindow('result '+ camera_mask.index, cv2.WINDOW_NORMAL)
        cv2.resizeWindow('result '+ camera_mask.index, 600, 400)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 抓出輪廓
        datacolor = []  # 儲存色塊中心座標
        # for cnt in contours:
        #     (x, y, w, h) = cv2.boundingRect(cnt)  # 取出輪廓外接的最小正矩形座標
        #     if w * h > 500 and w * h < 5000 and w < 2 * h and h < 2 * w:  # 過濾面積小於150的矩形
        #         rect = cv2.minAreaRect(cnt)  # 計算包圍目標的最小矩形區域
        #         rectlist = rect[0]  # 取得色塊中心座標
        #         datacolor.append(rectlist)
        #         box = cv2.boxPoints(rect)  # calculate coordinate of the minimum area rectangle
        #         box = np.int0(box)  # normalize coordinates to integers
        datacolor,box_list = findcolor(mask)
        for i in range(len(datacolor)):
            x = int(datacolor[i][0])
            y = int(datacolor[i][1])
            ### hard code
            x = int((x / 1920) * FIELD_LENGTH*100)
            y = int((y / 1080) * FIELD_WIDTH*100)
            print("trans_x,y",x,y)
            cv2.drawContours(frame, [box_list[i]], -1,(0,255,0) , 5)
            cv2.putText(frame, color, (x, y - 5), font, 0.7, 2)

        print('datacolor',datacolor)
        cv2.imshow('result '+ camera_mask.index , frame)
        if cv2.waitKey(1) == 27:
            cap.release()
            cv2.destroyAllWindows()
            break

def findcolor(mask):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 抓出輪廓
    datacolor = []  # 儲存色塊中心座標
    box_list = []
    # for cnt in contours:
    #     (x, y, w, h) = cv2.boundingRect(cnt)  # 取出輪廓外接的最小正矩形座標
    #     # w * h > 100 and w * h < 3000
    #     if w * h > 1000 and w * h < 6000 and w < 1.8 * h and h < 1.8 * w:  # 過濾面積小於150的矩形
    #     # if w * h > 100 and w * h < 3000 and w < 5 * h and h < 5 * w:  # 過濾面積小於150的矩形
    #         rect = cv2.minAreaRect(cnt)  # 計算包圍目標的最小矩形區域
    #         rectlist = rect[0]  # 取得色塊中心座標
    #         datacolor.append(rectlist)
    #         box = cv2.boxPoints(rect)  # calculate coordinate of the minimum area rectangle
    #         box = np.int0(box)  # normalize coordinates to integers
    #         box_list.append(box)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000 or area > 6000:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < 0.65 or circularity > 1.3:
            continue  # 過濾掉非圓形物件

        (x, y, w, h) = cv2.boundingRect(cnt)
        if w > 1.8 * h or h > 1.8 * w:
            continue  # 排除過於長條的物體

        rect = cv2.minAreaRect(cnt)  # 計算包圍目標的最小矩形區域
        rectlist = rect[0]           # 取得中心座標
        datacolor.append(rectlist)

        box = cv2.boxPoints(rect)    # 計算最小矩形四個頂點
        box = np.int0(box)
        box_list.append(box)
        #5/14 球

    return datacolor,box_list

def camera_selection(frame_left,frame_right,color_selection_setting,team_color):
    red_mask_left = colorMasking(frame=frame_left,color= 'red',color_selection_setting=color_selection_setting)
    red_location_left,_ = findcolor(mask = red_mask_left)
    blue_mask_left = colorMasking(frame=frame_left,color= 'blue',color_selection_setting=color_selection_setting)
    blue_location_left,_ = findcolor(mask = blue_mask_left) 

    red_mask_right = colorMasking(frame=frame_right,color= 'red',color_selection_setting=color_selection_setting)
    red_location_right,_ = findcolor(mask = red_mask_right)
    blue_mask_right = colorMasking(frame=frame_right,color= 'blue',color_selection_setting=color_selection_setting)
    blue_location_right,_ = findcolor(mask = blue_mask_right) 
    
    # if (len(red_location) == num_red_team and len(blue_location) == num_blue_team):
    count_red_left_left = 0
    count_red_left_right = 0
    count_red_right_right = 0
    count_red_right_left = 0

    count_blue_left_left = 0
    count_blue_left_right = 0
    count_blue_right_right = 0
    count_blue_right_left = 0

    if team_color == 'red':
        for loc in red_location_left:
            if loc[0] < FIELD_LENGTH//2:
                count_red_left_left += 1

            else:
                count_red_right_left += 1

        for loc in red_location_right:
            if loc[0] < FIELD_LENGTH//2:
                count_red_left_right += 1

            else:
                count_red_right_right += 1

        if count_red_left_right + count_red_right_right:
            return 'right'
        
        elif count_red_left_left + count_red_right_left:
            return 'left'

    elif team_color == 'blue':
        for loc in blue_location_left:
            if loc[0] < FIELD_LENGTH//2:
                count_blue_left_left += 1

            else:
                count_blue_right_left += 1

        for loc in blue_location_right:
            if loc[0] < FIELD_LENGTH//2:
                count_blue_left_right += 1

            else:
                count_blue_right_right += 1

        if count_blue_left_right + count_blue_right_right:
            return 'right'
        
        elif count_blue_left_left + count_blue_right_left:
            return 'left'

        


        


    
        

# def findred(frame,x1, x2, x3, y1, y2, y3, x11, x22, x33, y11, y22, y33):
#     lower_red1 = np.array([x1, x2, x3])  # 紅色範圍低閾值
#     upper_red1 = np.array([y1, y2, y3])  # 紅色範圍高閾值
#     lower_red2 = np.array([x11, x22, x33])  # 紅色範圍低閾值
#     upper_red2 = np.array([y11, y22, y33])  # 紅色範圍高閾值
#     hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 轉乘HSV模式
#     mask_red1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
#     mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
#     mask_red1 = cv2.medianBlur(mask_red1, 5)  # 中值濾波
#     mask_red2 = cv2.medianBlur(mask_red2, 5)  # 中值濾波
#     mask_red = cv2.bitwise_or(mask_red2, mask_red1)
#     contours2, hierarchy2 = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     datared = []
#     for cnt2 in contours2:
#         (x2, y2, w2, h2) = cv2.boundingRect(cnt2)
#         if w2 * h2 > 200 and w2 * h2 < 2000 and w2 < 2 * h2 and h2 < 2 * w2:
#             rect2 = cv2.minAreaRect(cnt2)
#             rect2list = rect2[0]
#             datared.append(rect2list)
#             box2 = cv2.boxPoints(rect2)
#             box2 = np.int0(box2)
#             # cv2.drawContours(frame, [box2], 0, (0, 0, 255), 3)
#             # cv2.putText(frame, "Red", (x2, y2 - 5), font, 0.7, (0, 0, 255), 2)
#     return datared

def change_coordinate_for_draw(coordinate_in_field):
    coordinate_in_frame = copy.deepcopy(coordinate_in_field)#################################################
    for i in range(len(coordinate_in_field)):
        coordinate_in_frame[i][0] = int((coordinate_in_field[i][0] / (FIELD_LENGTH * 100)) * 1920)
        coordinate_in_frame[i][1] = int((coordinate_in_field[i][1] / (FIELD_WIDTH * 100)) * 1080)
    return coordinate_in_frame

def draw_the_point(frame, coordinate_in_frame, object_name, object_position):
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i in range(len(coordinate_in_frame)):
        center = coordinate_in_frame[i]
        pos = object_position[i]

        # 畫圓點
        cv2.circle(frame, center, 6, (0, 255, 0), 3)

        # 寫物件名稱
        cv2.putText(frame, object_name, (center[0], center[1] - 10), font, 0.7, (0, 255, 0), 2)

        # 寫座標（稍微往下偏一點）
        pos_text = f"({int(pos[0])}, {int(pos[1])})"
        cv2.putText(frame, pos_text, (center[0], center[1] + 20), font, 0.7, (0, 255, 0), 2)

    return

    ############################################################################################# draw

def camera_selection_use_ball(ball_left,ball_right):
    print("ball position from left :",ball_left,"from right :",ball_right)
    if ball_left[0] == 0 and ball_left[1] == 0:##########因此如果00的話是right hard code
        print("ball in left camera disappear")
        # return [ball_left.x,ball_left.y]
        return "right"
    
    elif ball_right[0] == 0 and ball_right[1] == 0:
        print("ball in right camera disappear")
        # return [ball_right.x,ball_right.y]
        return "left"
    
    else:
        tmp = (ball_left[0] + ball_right[0])//2
        # print("tmp",tmp,FIELD_LENGTH*100//2)
        if (tmp < (FIELD_LENGTH*100//2)):
            return "left"
        
        else:
            return "right"

    
if __name__ == '__main__':

    image_result()


        # transform_degree = []
        # for degree in degree_result_blue:
        #     degree = degree0to360transform(degree)
        #     degree = degree2unit_vector(degree)
        #     transform_degree.append(degree)

        # for degree in degree_result_red:
        #     degree = degree0to360transform(degree)
        #     degree = degree2unit_vector(degree)
        #     transform_degree.append(degree)



        

        