import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math
from Robot_pos_estimator0520backup import my_estimatePoseSingleMarkers,calc_Aruco_distance,rotation_mtx2euler_angle,get_Aruco_information
from color_masking import colorMasking
import challenge2_1 as challenge2

ARUCO_SIZE = 0.078#0.053
CAMERA_HEIGHT = 1.5
CAMERA_DISTANCE = 1.2
# FIELD_LENGTH = 3.4
FIELD_LENGTH = 3.6
FIELD_WIDTH = 1.8

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
    
    def robot_pose_estimation(self):
        index = 999
        # print(self.id)
        if self.id == 0:
            order = [0,6,2,8,4]
        elif self.id == 1:
            order = [1,7,3,5,9]
        elif self.id == 2:
            order = [0,16,12,18,14]
        elif self.id == 3:
            order = [11,19,13,15,11]

        for id in order:
            if id in self.aruco_id_list:
                index = self.aruco_id_list.index(id)
                break

        if(index != 999):#,3,5,7,9,counterclockwise
            # print("index",index,"sef",self.aruco_x_list )
            if self.camera_setting == "left":
                self.x_left = self.aruco_x_list[index] - CAMERA_DISTANCE*100
                self.y_left = (FIELD_WIDTH * 100 // 2 + self.aruco_y_list[index])
                self.degree_left = degree0to360transform(self.aruco_degree_list[index])
            elif self.camera_setting == "right":
                self.x_right = FIELD_LENGTH* 100 - (self.aruco_x_list[index] - CAMERA_DISTANCE*100)
                self.y_right = (FIELD_WIDTH * 100 // 2 - self.aruco_y_list[index])
                self.degree_right = self.aruco_degree_list[index]
                self.degree_right = degree0to360transform(self.aruco_degree_list[index])
                self.degree_right = self.degree_right + 180
                if self.degree_right >= 360:
                    self.degree_right = self.degree_right - 360

    
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
            self.position_estimator()
            # print("inside",[self.x,self.y],self.degree)
            return [self.x,self.y],self.degree
        
    def position_estimator(self):
        self.x = self.x_right
        self.y = self.y_right
        self.degree = self.degree_right



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
        for i in range(length):
            aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list = [],[],[],[]
            self.Robot_list.append(Robot(i,aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list))

    def update_position(self,aruco_id_list,aruco_x_list,aruco_y_list,aruco_degree_list,camera_setting):
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
                # print("robot:",robot.id," x:",robot.x_left," y:",robot.y_left," degree",robot.degree_left)
                id_result.append(robot.id)
                # x_result.append(robot.x_left)
                # y_result.append(robot.y_left)
                pos_result.append([robot.x_left,robot.y_left])
                degree_result.append(robot.degree_left)
            elif needed_direction == "right":
                # print("robot:",robot.id," x:",robot.x_right," y:",robot.y_right," degree",robot.degree_right)
                id_result.append(robot.id)
                # x_result.append(robot.x_right)
                # y_result.append(robot.y_right)
                pos_result.append([robot.x_right,robot.y_right])
                degree_result.append(robot.degree_right)
                # return robot.id_right,robot.x_right,robot.y_right,robot.degree_right

            elif needed_direction == "overall":
                robot.get_information(needed_direction)
                # print("robot:",robot.id," x:",robot.x," y:",robot.y," degree",robot.degree)
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

    def update_position(self,frame,camera_setting):
        font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
        # self.x = 0
        # self.y = 0
        mask = colorMasking(frame,"yellow")
        mask = cv2.medianBlur(mask, 5)  # 中值濾波
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 抓出輪廓
        datacenter = []  # 儲存球心座標
        dataradius = []  # 儲存半徑座標
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)  # 取出輪廓外接的最小正矩形座標
            # print("check",w,h)
            if w * h > 100 and w * h < 3000 and w < 5 * h and h < 5 * w:  # 過濾面積小於150的矩形
                (xx, yy), radius = cv2.minEnclosingCircle(cnt)  # 找出最小外接圓
                ball_x = int(xx)
                ball_y = int(yy)
                center = [ball_x,ball_y]
                radius = int(radius)
                # print(radius,"radius")
                # if radius > 15 and int((ball_y / 1080) * FIELD_WIDTH * 100) < 140:
                if radius > 15 :
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
                transform_ball_x = int((ballcenter[0] / 1920) * FIELD_LENGTH * 100)
                transform_ball_y = int((ballcenter[1] / 1080) * FIELD_WIDTH * 100)
                [self.x,self.y] = [transform_ball_x,transform_ball_y]
            
            elif camera_setting == "left":
                ballcenter = datacenter[0]
                # print(ballcenter)
                transform_ball_x = int((ballcenter[0] / 1920) * FIELD_LENGTH * 100)
                transform_ball_y = int((ballcenter[1] / 1080) * FIELD_WIDTH * 100)
                [self.x,self.y] = [transform_ball_x,transform_ball_y]
   
class Camera():
    def __init__(self,index,number,video_path = False):
        self.index = index
        self.video_path = video_path
        self.number = number
        self.camera_setting()

    def __call__(self):
        return self.cap
    
    def camera_setting(self):

        if self.index == "left":
            # calibration_matrix_path = "./calibration_matrix_left.npy"
            # distortion_coefficients_path = "./distortion_coefficients_left.npy"
            calibration_matrix_path = "./calibration_matrix_left.npy"
            distortion_coefficients_path = "./distortion_coefficients_left.npy"
        elif self.index == "right":
            calibration_matrix_path = "./calibration_matrix_right.npy"
            distortion_coefficients_path = "./distortion_coefficients_right.npy"

        self.camera_matrix = np.load(calibration_matrix_path)
        self.dist_matrix = np.load(distortion_coefficients_path)

        cap = cv2.VideoCapture(self.number,cv2.CAP_DSHOW) 
        if (self.video_path):
        # cap = cv2.VideoCapture('2024-03-27_19-20-54.mp4') # 讀取電腦中的影片
            cap = cv2.VideoCapture(self.video_path) # 讀取電腦中的影片

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS, 60)  # 设置帧率为60帧/秒

        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        
        self.cap = cap
    
def gamma_correction(frame, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
    	# apply gamma correction using the lookup table
	return cv2.LUT(frame, table)
 
class Aruco():
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.parameters =  cv2.aruco.DetectorParameters()
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.3
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 60
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
    
    def update(self,frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #使用aruco.detectMarkers()函數可以檢測到marker，返回ID和标志板的4个角点坐標
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        return corners,ids,rejectedImgPoints

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
    

      
team_degree = []
team_pos = []
oppo_pos = []
oppo_degree = []
ball_center = []

def image_result():
# if __name__ == '__main__':
    # camera1 = Camera("left",number=0,video_path="WIN_20240327_19_20_56_Pro.mp4")
    # camera2 = Camera("right",number=1,video_path="2024-03-27_19-20-54.mp4")
    camera1 = Camera("left",number=0,video_path=False)
    camera2 = Camera("right",number=2,video_path=False)
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    red_team = Robot_Team(color='red',length = 2)
    blue_team = Robot_Team(color='blue',length = 2)
    ball = Ball()
    aruco_define = Aruco()
    camera_list = [camera1,camera2]
    print("go")
    while True:
        for camera in camera_list:
            # print(camera.index)
            # if camera.index == "right":
            #     break
            # result = frame_run(camera)
            # if not result:
            #     break
        # cap = cap1
            cap = camera()
            ret, frame = cap.read()
            # if (cap.isOpened()):  # 視訊開啟成功
            #     flag = 1
            # else:
            #     flag = 0
            # num = 0

            # if (flag):
            # while (True):
            h1, w1 = frame.shape[:2]
            #糾正畸變
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera.camera_matrix, camera.dist_matrix, (h1, w1), 1, (h1, w1))
            # dst1 = cv2.undistort(frame, camera.camera_matrix, camera.dist_matrix, None, newcameramtx)
            # x, y, w1, h1 = roi
            # dst1 = dst1[y:y + h1, x:x + w1]
            # frame = dst1
            if camera.index == "right":
                points_path = "./points.npy"
            elif camera.index == "left":
                points_path = "./points_left.npy"
            points = np.load(points_path)
            p1 = np.float32(points)
            p2 = np.float32([[0,0],[1080,0],[0,1920],[1080,1920]])
            m = cv2.getPerspectiveTransform(p1,p2)
            # frame = cv2.warpPerspective(frame, m, (1080, 1920))
            # frame = gamma_correction(frame,0.2)
            frame = gamma_correction(frame, gamma=0.5)
            corners,ids,rejectedImgPoints = aruco_define.update(frame = frame)

            frame,id_list,distance_list,distance_x_list,degree_list,degree_list_x,degree_list_y,degree_list_z\
                = get_Aruco_information(camera_matrix=camera.camera_matrix,dist_matrix=camera.dist_matrix,frame=frame,corners=corners,ids=ids)
            # print("dis",distance_list,distance_x_list)
            red_id_list,blue_id_list,red_x_list,blue_x_list,red_y_list,blue_y_list,red_degree_list,blue_degree_list = [],[],[],[],[],[],[],[]

            for index,id in enumerate(id_list):
                if id % 2 == 0:
                    #even,blue team
                    blue_id_list.append(id)
                    blue_x_list.append(distance_list[index])
                    blue_y_list.append(distance_x_list[index])
                    blue_degree_list.append(degree_list[index])

                else:
                    red_id_list.append(id)
                    red_x_list.append(distance_list[index])
                    red_y_list.append(distance_x_list[index])
                    red_degree_list.append(degree_list[index])
            # print("check",red_degree_list)
            # print("red_dist",red_x_list)
            # print("blue_dist",blue_x_list)
            blue_team.update_position(blue_id_list,blue_x_list,blue_y_list,blue_degree_list,camera_setting=camera.index)
            red_team.update_position(red_id_list,red_x_list,red_y_list,red_degree_list,camera_setting=camera.index)

            # red_team.get_information(needed_direction=camera.index)
            id_result_blue,pos_blue,degree_result_blue = blue_team.get_information(needed_direction=camera.index)
            id_result_red,pos_red,degree_result_red = red_team.get_information(needed_direction=camera.index)

            # frame_resized = cv2.resize(frame, (0, 0), fx=1000, fy=800)
            frame = cv2.warpPerspective(frame, m, (1080, 1920))
            if camera.index == "left":
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif camera.index =="right":
                # pass
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            ball.update_position(frame=frame,camera_setting=camera.index)
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
            cv2.putText(frame,'coordinates:'+ str(pos_blue) ,(0, 110), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'deg_z:'+ str(degree_result_blue) ,(0, 150), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            # cv2.putText(frame,'ballcenter:'+ str([ball.x,ball.y]) ,(0, 190), font, 1, (0, 255, 0), 2,cv2.LINE_AA)

            cv2.putText(frame,'Red:' ,(0, 230), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'ids:'+ str(id_result_red) ,(0, 270), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'coordinates:'+ str(pos_red) ,(0, 310), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'deg_z:'+ str(degree_result_red) ,(0, 350), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            cv2.putText(frame,'ballcenter:'+ str([ball.x,ball.y]) ,(0, 390), font, 1, (0, 255, 0), 2,cv2.LINE_AA)
            
            draw_arri_point(frame=frame)
            cv2.namedWindow('frame '+ camera.index, cv2.WINDOW_NORMAL)
            cv2.resizeWindow('frame ' + camera.index, 600, 400)
            cv2.imshow('frame ' + camera.index,frame)

            id_result_blue,pos_blue,degree_result_blue = blue_team.get_information(needed_direction="overall")
            id_result_red,pos_red,degree_result_red = red_team.get_information(needed_direction="overall")

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
            # print("final")
            global team_degree,team_pos,oppo_pos,oppo_degree,ball_center
            print("team_pos_im",team_pos)
            print("pos_blue",pos_blue)
            team_degree = transform_degree_blue
            team_pos = pos_blue
            oppo_pos = pos_red
            oppo_degree = transform_degree_red
            ball_center = [ball.x,ball.y]
            print("ball_center inside",ball.x,ball.y)


            # key = cv2.waitKey(1)

            # if key == 27:
            #     print('esc break...')
            #     cap.release()
            #     cv2.destroyAllWindows()
            #     # break

            # if key == ord(' '):
            #     filename = str(time.time())[:10] + ".jpg"
            #     cv2.imwrite(filename, frame)
            if cv2.waitKey(5) & 0xFF == 27:
                break
        # cap.release()
        # cv2.destroyAllWindows()
                # return team_degree,team_pos,oppo_pos,oppo_degree,ball_center

                    

                    # transform_degree = []
                    # for degree in degree_result_blue:
                    #     degree = degree0to360transform(degree)
                    #     degree = degree2unit_vector(degree)
                    #     transform_degree.append(degree)

                    # for degree in degree_result_red:
                    #     degree = degree0to360transform(degree)
                    #     degree = degree2unit_vector(degree)
                    #     transform_degree.append(degree)

def draw_arri_point(frame):
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    if challenge2.first_arri != []:
        arri_point = challenge2.first_arri
        print("arri_point",arri_point)
        cv2.circle(frame, arri_point, 1, (360, 100, 100), 3)  # 畫圓
        cv2.putText(frame, "ARRI", (arri_point[0], arri_point[1] - 5), font, 0.7, (360, 100, 100), 2)  # 螢幕上寫出"ARRI"
    
    if challenge2.kick_goal != []:
        kick_point = challenge2.kick_goal

        cv2.circle(frame, kick_point, 1, (360, 100, 100), 3)  # 畫圓
        cv2.putText(frame, "target", (kick_point[0], kick_point[1] - 5), font, 0.7, (360, 100, 100), 2)  # 螢幕上寫出"ARRI"

    if challenge2.oppo_p != [] and challenge2.oppo_p != [0,0]:
        oppo_point = challenge2.oppo_p
        cv2.circle(frame, (int(oppo_point[0]), int(oppo_point[1])) , 1, (360, 100, 100), 3)  # 畫圓
        cv2.putText(frame, "oppoP", (int(oppo_point[0]), int(oppo_point[1])), font, 0.7, (360, 100, 100), 2)  # 螢幕上寫出"ARRI"

    if challenge2.player_p != [] and challenge2.player_p != [0,0]:
        player_p_point = challenge2.player_p
        cv2.circle(frame, (int(player_p_point[0]), int(player_p_point[1])) , 1, (360, 100, 100), 3)  # 畫圓
        cv2.putText(frame, "player_p_point", (int(player_p_point[0]), int(player_p_point[1])), font, 0.7, (360, 100, 100), 2)  # 螢幕上寫出"ARRI"
    pass


    



        

        