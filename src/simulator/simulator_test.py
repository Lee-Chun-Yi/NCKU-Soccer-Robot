import cv2
import math
import time
import enum
import numpy as np
import os
import sys

#from field import *
#from robot import *
#from vector import *

 
robo_num = 6  # 設定為 6 個機器人

from simulator_appearance import *

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from config.constants import BOUNDARY, CENTER
from strategies import challenge3_forward as st_strategy


FRAME_WIDTH, FRAME_HEIGHT = 1920/2, 1280/2 
OFFSET = [0, 0] #Offset is to calibrate the total field position
SCALE = 1

# Game parameter
Side = -1 #1:Left A  Right B  #-1:Left B  Right A
CMD_FPS = 2
CAM_FPS = 15

# Robot movement parameter
Robo_rotate_speed = 2.5*math.pi/180 #radius change
'''
    the step size of a robot for each command
    [fast_for, for, back, move_left, move_right, fast_move_l, fast_move_r]
'''
Robo_move = [1.2, 0.66, 0.66, 0.53, 0.53, 5.3, 5.3]  # [4, 3, 3, 4, 4, 6, 6]

# Ball simulation parameter
Ball_bump_speed = 1         # Bump speed when robot walking #走路碰到時起使速度
Ball_kicked_speed = 6       # Robot kick speed #踢擊時起使速度
Ball_n_acce = 0.1          # Ball moving deacceleration #球的移動減速度
Ball_passing_speed = 2      # Robot pass speed #經過碰到時
Ball_rotbump_speed = 4      # Bump speed when robot turning #旋轉踢到時的速度

# Simulator debug option
Cursor_Position_Show = 0

# Simulator inner parameters
ball_direction = [1.0, 0.0]
ball_pos = [0, 0] 
ball_speed = 0 
FRAME_FPS = 30 
Kickable_distance = 70 
mode = 1 
Mouse_Position = [0, 0]
CMD = ['N1', 'N1','N1'] # 初始只有3個，後面會自動補齊
# 因為使用機器人的class，裡面有定義方向
# Robo_Direction = [[1.0, 0.0], [1.0, 0.0], [1.0, 0.0], [-1.0, 0.0], [-1.0, 0.0], [-1.0, 0.0]]
Robo_kick_angle_threshold = 60*math.pi/180 
Robo_rotate_rect = [None, None, None, None, None, None]
Robo_rotate_vert = [None, None, None, None, None, None]
Setup_Index = 0 

class _DummyRobot:
    def __init__(self, pos=None, direction=None):
        self.pos = pos if pos is not None else [0.0, 0.0]
        self.dir = direction if direction is not None else [1.0, 0.0]

class _State:
    pass

st_A = _State()
st_A.Player1 = _DummyRobot([float(CENTER[0]), float(CENTER[1])])
st_A.Player2 = _DummyRobot([float(CENTER[0] - 100), float(CENTER[1] - 50)])
st_A.Player3 = _DummyRobot([float(CENTER[0] - 100), float(CENTER[1] + 50)])
st_A.OPPO1 = _DummyRobot([float(CENTER[0] + 100), float(CENTER[1])], [-1.0, 0.0])
st_A.OPPO2 = _DummyRobot([float(CENTER[0] + 100), float(CENTER[1] - 50)], [-1.0, 0.0])
st_A.OPPO3 = _DummyRobot([float(CENTER[0] + 100), float(CENTER[1] + 50)], [-1.0, 0.0])
st_A.AllRobots = [st_A.Player1, st_A.Player2, st_A.Player3, st_A.OPPO1, st_A.OPPO2, st_A.OPPO3]
st_A.ball_pos = ball_pos

# The simulation of object is simulated with update frequency @FRAME_FPS
# In practice, the camera has maximum update frequency of 30 fps. You can set this parameter with @CAM_FPS
# Also, the sending frequency in practice will not be 30 fps because it is too frequent for robot to reacat.
# YOu can modify the sending frequency in @CMD_FPS
# If you modify the value, you should modify object speed with appropriate scalar. Implement by yourself.
STRATEGY_UP, SEND_UP = int(FRAME_FPS/CAM_FPS), int(FRAME_FPS/CMD_FPS)

class MOVE(enum.Enum):
    NONE = 'N'
    FORWARD = 'w'
    BACKWARD = 's'
    TRUN_LEFT = 'q'
    TRUN_RIGHT = 'e'
    MOVE_LEFT = 'a'
    MOVE_RIGHT = 'd'
    F_FORWARD = 'W'
    F_MOVE_LEFT = 'A'
    F_MOVE_RIGHT = 'D'
    F_TRUN_LEFT = 'Q'
    F_TRUN_RIGHT = 'E'
    RFSHOOT = 'i'
    LFSHOOT = 'u'
    RSSHOOT = 'j'
    LSSHOOT = 'h'
    RBSHOOT = 'n'
    LBSHOOT = 'b'
    RPASS = 'p'
    LPASS = 'o'
    F_DEFENSE = 'Y'
    L_DEFENSE = 'f'
    R_DEFENSE = 'g'
    DEFENSE = 'y' 
    REST = 'r' 
    STAND = 'R'   
    STOP = 'x'  
    START = 'z'
    PK_MODE = 'c'

############################# Simulator Function ##################################

def draw_empty(frame,FB_x, FB_y, PK_x, Penalty_y, Center_Circle, GA_x, GA_y):#frame 為生成圖檔的名子
    """透過已知的資訊生成圖檔(名字叫frame)"""
    #mark center
    cv2.rectangle(frame, (int(CENTER[0])-CenterHalfSize, int(CENTER[1])-CenterHalfSize), (int(CENTER[0])+CenterHalfSize, int(CENTER[1])+CenterHalfSize), Line_Color, -1)
    #mark middle line
    cv2.line(frame, (CENTER[0], BOUNDARY[0][1]), (CENTER[0], BOUNDARY[7][1]), Line_Color, 3)
    #mark goal line
    cv2.line(frame, (BOUNDARY[8][0],BOUNDARY[8][1]),(BOUNDARY[11][0],BOUNDARY[11][1]), Line_Color,3)
    cv2.line(frame, (BOUNDARY[2][0],BOUNDARY[2][1]),(BOUNDARY[5][0],BOUNDARY[5][1]), Line_Color,3)
    #Build FK FB PK point
    cv2.circle(frame, (int(CENTER[0]+FB_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]+FB_x), (int(CENTER[1]+FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]+FB_x), (int(CENTER[1]-FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-FB_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-FB_x), (int(CENTER[1]+FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-FB_x), (int(CENTER[1]-FB_y))), 3, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]+PK_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    cv2.circle(frame, (int(CENTER[0]-PK_x), (int(CENTER[1]))), 5, Line_Color, -1, 8, 0)
    #Build Penalty Area
    cv2.line(frame,(int(CENTER[0]+PK_x),(int(CENTER[1]+Penalty_y))),(int(CENTER[0]+PK_x),(int(CENTER[1]-Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-PK_x),(int(CENTER[1]+Penalty_y))),(int(CENTER[0]-PK_x),(int(CENTER[1]-Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+PK_x),(int(CENTER[1]+Penalty_y))),(BOUNDARY[5][0],(int(CENTER[1]+Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+PK_x),(int(CENTER[1]-Penalty_y))),(BOUNDARY[2][0],(int(CENTER[1]-Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-PK_x),(int(CENTER[1]+Penalty_y))),(BOUNDARY[11][0],(int(CENTER[1]+Penalty_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-PK_x),(int(CENTER[1]-Penalty_y))),(BOUNDARY[8][0],(int(CENTER[1]-Penalty_y))), Line_Color,3)
    #Build Goal Area
    cv2.line(frame,(int(CENTER[0]+GA_x),(int(CENTER[1]+GA_y))),(int(CENTER[0]+GA_x),(int(CENTER[1]-GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-GA_x),(int(CENTER[1]+GA_y))),(int(CENTER[0]-GA_x),(int(CENTER[1]-GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+GA_x),(int(CENTER[1]+GA_y))),(BOUNDARY[5][0],(int(CENTER[1]+GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]+GA_x),(int(CENTER[1]-GA_y))),(BOUNDARY[2][0],(int(CENTER[1]-GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-GA_x),(int(CENTER[1]+GA_y))),(BOUNDARY[11][0],(int(CENTER[1]+GA_y))), Line_Color,3)
    cv2.line(frame,(int(CENTER[0]-GA_x),(int(CENTER[1]-GA_y))),(BOUNDARY[8][0],(int(CENTER[1]-GA_y))), Line_Color,3)
    #Center Circle
    cv2.circle(frame,(int(CENTER[0]),(int(CENTER[1]))),int(Center_Circle),Line_Color ,3, 8,0)
    #Draw Boundary
    cv2.line(frame, BOUNDARY[0],BOUNDARY[1], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[1],BOUNDARY[2], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[2],BOUNDARY[3], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[3],BOUNDARY[4], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[4],BOUNDARY[5], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[5],BOUNDARY[6], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[6],BOUNDARY[7], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[7],BOUNDARY[8], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[8],BOUNDARY[9], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[9],BOUNDARY[10], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[10],BOUNDARY[11], (0, 0, 255),5)
    cv2.line(frame, BOUNDARY[11],BOUNDARY[0], (0, 0, 255),5)

def draw_object(frame,FB_x, FB_y, PK_x, Penalty_y, Center_Circle, GA_x, GA_y):
    """Draw robot, ball and other info"""
     ##########################################################
    # Draw runtime image,like moving locus or moving target  #
    # Since color of runtime image may affect boundary judge #
    # of moving object, they cannot be placed 'in front of'  #
    # field's object (ex, boundary should never hoverd by    #
    # colors other than red), so we should draw them first.  #
    ##########################################################
    #Draw other information for debug. Ignore this block when you are not using strategy1 and 2
    try:
        pass
        frame = st_strategy.draw_on_simulator(frame)
    except AttributeError:
        pass

    #mark mouse marker
    cv2.rectangle(frame, (Mouse_Position[0], Mouse_Position[1]), (Mouse_Position[0]+MarkerSize, Mouse_Position[1]+MarkerSize), (255, 255, 255), -1)#將目前鼠標參數位置印出
    #mark mode text
    txt = ''#設定螢幕左側的控制狀態
    if mode == 2:
        txt= mode_txt[mode]+str(Setup_Index)
    elif mode == 3:
        txt = 'test'
    else:
        txt = mode_txt[mode]
    cv2.putText(frame, txt, (0, int(CENTER[1])), cv2.FONT_HERSHEY_SIMPLEX,1, TXT_Color, 5, cv2.LINE_AA)
    
    # Draw Robo Name and Direction for all 6 robots
    # RoboColor 定義了每個機器人的顏色，順序對應 RoboIndex 0~5
    RoboColor = [TeamA_Color, TeamB_Color, TeamC_Color, OppoA_Color, OppoB_Color, OppoC_Color]
    
    for RoboIndex in [0,1,3,4]: 
        setup = st_A.AllRobots[RoboIndex]
        
        # Mark Robot Name
        # Robo_name 需要在 simulator_appearance.py 定義成有 6 個元素的列表，若不足會報錯
        # 預設如果只有兩個名字，這裡可以加個保護或直接用 Index 顯示
        name_str = Robo_name[RoboIndex] if RoboIndex < len(Robo_name) else f"Robo{RoboIndex}"
        
        cv2.putText(frame, name_str, 
                    (round(setup.pos[0]), round(setup.pos[1]) - 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (102, 102, 0), 2, cv2.LINE_AA)

        # Calculate rotated rectangle
        angle = 0
        cross, dot, robo_angle, err = vector_product(setup.dir[0], setup.dir[1], 1, 0, 1)
        angle = 180 if err == 1 else robo_angle / math.pi * 180
        angle = -1 * angle if cross >= 0 else angle
        
        rot_rect = (tuple(setup.pos), (RoboDepth, RoboWidth), angle)
        box = cv2.boxPoints(rot_rect)
        box = box.astype(int) #取整數
        
        # Draw Robot Body
        color = RoboColor[RoboIndex] if RoboIndex < len(RoboColor) else (128, 128, 128)
        cv2.drawContours(frame, [box], 0, color, -1) 
        
        global Robo_rotate_rect, Robo_rotate_vert
        Robo_rotate_vert[RoboIndex] = box
        Robo_rotate_rect[RoboIndex] = rot_rect
        
        # Mark Robot Direction Line
        cv2.line(frame, (round(setup.pos[0]), round(setup.pos[1])), 
                 (int(setup.pos[0] + Robo_direction_length * setup.dir[0]), 
                  int(setup.pos[1] + Robo_direction_length * setup.dir[1])), 
                 Robo_direction_Color, 3)

    #mark ball
    cv2.circle(frame,(st_A.ball_pos[0],st_A.ball_pos[1]),BallHalfSize,Ball_Color,-1, 8,0)

def _strategy_update_info(team_d, team_p, opp_d, opp_p, ball):
    try:
        st_strategy.Update_Robo_Info(team_d, team_p, opp_p, ball)
    except TypeError:
        try:
            st_strategy.Update_Robo_Info(team_d, team_p, opp_d, opp_p, ball)
        except Exception:
            return

def CV_event(event, x, y, flags, param):
    global ball_pos, ball_speed
    
    # 使用 Setup_Index (0~5) 選擇對應的機器人
    if 0 <= Setup_Index < len(st_A.AllRobots):
        setup = st_A.AllRobots[Setup_Index]
    else:
        return # Index out of range
    
    if event == cv2.EVENT_MOUSEMOVE and Cursor_Position_Show:
        print('Cousor position X: ',x,' Y: ',y,' Pixel BGR value: ',empty_field[y][x])  
    if event == cv2.EVENT_LBUTTONDOWN:
        if mode == 2 and flags == cv2.EVENT_FLAG_LBUTTON:#Modify Robo position
            setup.pos = [float(x), float(y)]
        elif mode == 2 and flags == cv2.EVENT_FLAG_SHIFTKEY + cv2.EVENT_FLAG_LBUTTON :      
            d_x = x - setup.pos[0]
            d_y = y - setup.pos[1]
            try:
                x_comp = d_x/math.hypot(d_x,d_y)
                y_comp = d_y/math.hypot(d_x,d_y)
                setup.dir = [x_comp,y_comp]
            except ZeroDivisionError:
                return
        else:
            #change mouse mark coordinate
            global Mouse_Position
            #Robo_Position[Setup_Index] = [x,y].copy()
            Mouse_Position = [x,y]
            print('Modify mouse position X: ',x,' Y: ',y,' Pixel BGR value: ',empty_field[y][x])
        team_d = [r.dir for r in st_A.AllRobots[0:3]]
        team_p = [r.pos for r in st_A.AllRobots[0:3]]
        opp_d  = [r.dir for r in st_A.AllRobots[3:6]]
        opp_p  = [r.pos for r in st_A.AllRobots[3:6]]

        _strategy_update_info(team_d, team_p, opp_d, opp_p, ball_pos)
    if event == cv2.EVENT_RBUTTONDOWN:#mouse right click event
        ball_speed = 0
        ball_pos[0] = x
        ball_pos[1] = y
        
        # 這裡需要傳遞所有機器人的資訊給 Update_Robo_Info
        # Update_Robo_Info 目前接收 team_d, team_p, opp_d, opp_p
        team_d = [r.dir for r in st_A.AllRobots[0:3]]
        team_p = [r.pos for r in st_A.AllRobots[0:3]]
        opp_d  = [r.dir for r in st_A.AllRobots[3:6]]
        opp_p  = [r.pos for r in st_A.AllRobots[3:6]]

        
        # st_B.Update_Robo_Info(Robo_Direction[3:6], Robo_Position[3:6], Robo_Position[0:3], Ball_Position)
        print('Modify Ball position X: ',x,' Y: ',y,' Pixel BGR value: ',empty_field[y][x]) 
        if mode != 0:
            _strategy_update_info(team_d, team_p, opp_d, opp_p, ball_pos)

def allmovement():
    """Tell all the robot for it's command"""
    
    # 確保 CMD 列表長度足夠，不足則補上 'N1' (停止指令)
    while len(CMD) < robo_num:
        CMD.append('N1')

    for RoboIndex, command in enumerate(CMD):
        if RoboIndex < robo_num:
            # command 是一個字串，例如 'N1', 'w1', 'a1'
            # robomovement 需要的是指令的第一個字元，例如 'N', 'w', 'a'
            # 這是對應 Enum MOVE 的 value
            if isinstance(command, str) and len(command) > 0:
                 robomovement(RoboIndex, command[0])
    pass

'''
Move the robot in the simulator
We give a command to the robot, and the robot need to react to it as in the real world,
for this function, we give the robot id and the command give to the robot, then it judge the legitimacy of the move, 
if the move is legal, move it, if not, step one step back, and blink 
@param RoboIndex    =>  the ID of the robot to control
@param command      =>  the command for the robot (Char)
'''
def robomovement(RoboIndex, command):
    #Decide direction
    global ball_speed, ball_direction # get excess to the robot position, direction, ballspeed and direction is for accidentily touching the ball

    # Total: 7
    if command == MOVE.FORWARD.value or command == MOVE.BACKWARD.value or \
        command == MOVE.MOVE_LEFT.value or command == MOVE.MOVE_RIGHT.value or \
            command == MOVE.F_FORWARD.value or command == MOVE.F_MOVE_LEFT.value or \
                command == MOVE.F_MOVE_RIGHT.value:
        robo_walk(RoboIndex, command)
        pass

    # Total: 4
    elif command == MOVE.TRUN_LEFT.value or command == MOVE.TRUN_RIGHT.value or \
        command == MOVE.F_TRUN_LEFT.value or command == MOVE.F_TRUN_RIGHT.value:
        robo_turn(RoboIndex, command)
        pass
    
    # Total: 2
    elif command == MOVE.RFSHOOT.value or command == MOVE.LFSHOOT.value:
        robo_shoot(RoboIndex, 0)
        pass

    # Total: 2
    elif command == MOVE.RSSHOOT.value or command == MOVE.LSSHOOT.value :
        robo_shoot(RoboIndex, 1)
        pass

    # Total: 2
    elif command == MOVE.RBSHOOT.value or command == MOVE.LBSHOOT.value:
        robo_shoot(RoboIndex, 2)
        pass

    # Total: 2
    elif command == MOVE.LPASS.value or command == MOVE.RPASS.value:
        robo_pass(RoboIndex)
        pass

    # Total: 5
    elif command == MOVE.NONE.value or command == MOVE.DEFENSE.value or \
        command == MOVE.STAND.value or command == MOVE.STOP.value or \
            command == MOVE.REST.value  or command == MOVE.START.value  or \
                command == MOVE.PK_MODE.value:
        pass

    # Total: 3
    elif command == MOVE.F_DEFENSE.value or command == MOVE.L_DEFENSE.value or \
        command == MOVE.R_DEFENSE.value:
        pass
    else:
        pass # print("False command, error")

def robo_walk(RoboIndex, command):
    """Move one step"""
    #Decide direction
    global ball_speed, ball_direction

    d_robo_x,d_robo_y = 0.0,0.0

    distance, unit_x, unit_y = 0.0, 0.0, 0.0
    
    if 0 <= RoboIndex < len(st_A.AllRobots):
        setup = st_A.AllRobots[RoboIndex]
    else:
        return
    
    if command == MOVE.F_FORWARD.value:
        distance = Robo_move[0]
        unit_x = setup.dir[0]
        unit_y = setup.dir[1]
    elif command == MOVE.FORWARD.value:
        distance = Robo_move[1]
        unit_x = setup.dir[0]
        unit_y = setup.dir[1]
    elif command == MOVE.BACKWARD.value:        
        distance = Robo_move[2]
        unit_x = -setup.dir[0]
        unit_y = -setup.dir[1]
    elif command == MOVE.MOVE_LEFT.value:         
        distance = Robo_move[3]
        unit_x = setup.dir[1]
        unit_y = -setup.dir[0]
    elif command == MOVE.MOVE_RIGHT.value:        
        distance = Robo_move[4]
        unit_x = -setup.dir[1]
        unit_y = setup.dir[0]
    elif command == MOVE.F_MOVE_LEFT.value:         
        distance = Robo_move[5]
        unit_x = setup.dir[1]
        unit_y = -setup.dir[0]
    elif command == MOVE.F_MOVE_RIGHT.value:        
        distance = Robo_move[6]
        unit_x = -setup.dir[1]
        unit_y = setup.dir[0]
    else:
        pass

    for i in range(0,round(distance),1):   
        d_robo_x = d_robo_x + unit_x
        d_robo_y = d_robo_y + unit_y
        #When moving, check if bump into other robot or boundary or ball
        bump_boundary = 0
        #Calculate new rectangle of next move (in OpenCV, rotated rectangle angle is in degree not radius)
        tmp_angle = 0
        tmp_cross, tmp_dot, robo_angle, tmp_err = vector_product(setup.dir[0], setup.dir[1], 1, 0, 1)
        tmp_angle = 180 if tmp_err == 1 else robo_angle /math.pi * 180
        tmp_angle = -1 * tmp_angle if tmp_cross >= 0 else tmp_angle
        tmp_rot_rect = ((setup.pos[0]+d_robo_x, setup.pos[1]+d_robo_y), (RoboDepth, RoboWidth), tmp_angle)
        #Parse this new rectangle to bump detection
        bump_boundary,__ = robo_bump(RoboIndex, tmp_rot_rect)
        #If bump with ball then ball moving
        if bump_boundary == 3:
            ball_speed = 3*Ball_bump_speed
            ball_direction = [unit_x, unit_y]
        #Below should be the final case since it might break the loop
        #If robot do bump something
        if bump_boundary != 0 :
            # d_robo_x = d_robo_x - unit_x
            # d_robo_y = d_robo_y - unit_y
            d_robo_x = 0
            d_robo_y = 0
            break
        
        
    setup.pos[0] = setup.pos[0] + d_robo_x
    setup.pos[1] = setup.pos[1] + d_robo_y

def robo_turn(RoboIndex, command):#direction 1:counterclockwise  0:clockwise
    """Turn Robo one time"""
    global ball_speed, ball_direction

    if 0 <= RoboIndex < len(st_A.AllRobots):
        setup = st_A.AllRobots[RoboIndex]
    else:
        return
    
    x_dir = setup.dir[0]
    y_dir = setup.dir[1]

    rotate_angle = 0
    if command == MOVE.TRUN_RIGHT.value:
        rotate_angle = Robo_rotate_speed
    elif command == MOVE.TRUN_LEFT.value:
        rotate_angle = -1 * Robo_rotate_speed
    elif command == MOVE.F_TRUN_RIGHT.value:
        rotate_angle = Robo_rotate_speed*2
    elif command == MOVE.F_TRUN_LEFT.value:
        rotate_angle = -2 * Robo_rotate_speed
    rotate_cos = math.cos(rotate_angle)
    rotate_sin = math.sin(rotate_angle)
    x_final, y_final = x_dir*rotate_cos - y_dir*rotate_sin, x_dir*rotate_sin + y_dir*rotate_cos
    
    #Collision might happen when rotate. Just as what we do in Robo moving, Calculate new rectangle of next move first.
    tmp_angle = 0
    tmp_cross, tmp_dot, robo_angle, tmp_err = vector_product(x_final, y_final, 1, 0, 1)
    tmp_angle = 180 if tmp_err == 1 else robo_angle /math.pi * 180
    tmp_angle = -1 * tmp_angle if tmp_cross >= 0 else tmp_angle
    tmp_rot_rect = ((setup.pos[0], setup.pos[1]), (RoboDepth, RoboWidth), tmp_angle)
    #Parse this new rectangle to bump detection
    bump_boundary,rot_bump_position = robo_bump(RoboIndex, tmp_rot_rect)
    if bump_boundary == 3:
        ball_speed = Ball_rotbump_speed
        ball_dir_x = ball_pos[0] - rot_bump_position[0]
        ball_dir_y = ball_pos[1] - rot_bump_position[1]
        length = math.hypot(ball_dir_x, ball_dir_y)
        ball_d_x = ball_dir_x/length
        ball_d_y = ball_dir_y/length
        ball_direction = [ball_d_x, ball_d_y]
    elif bump_boundary == 0:
        setup.dir = [x_final, y_final]

def robo_bump(RoboIndex, rot_rect):
    """Detect bumping with Boundary, ball,Robot when robot is moving"""
    bump_boundary = 0
    rot_bump_position = [0, 0]
    vertices = cv2.boxPoints(rot_rect)
    vertices = vertices.astype(int).tolist()
    ###################################################################################
    # Detect Robot-Robot collision by OpenCV rotatedRectangleIntersection function    #
    # Return 1 if bump                                                                #
    ###################################################################################
    if bump_boundary == 0:
        for rec_index, rect in enumerate(Robo_rotate_rect):
            if rect is None: # 避免未初始化時報錯
                continue
            if rec_index == RoboIndex:
                pass
            else:
                intersect = cv2.rotatedRectangleIntersection(rect, rot_rect)
                if intersect[0] != 0:
                    bump_boundary = 1
                    print(RoboIndex,' bump with ',rec_index)
                    break
    ###################################################################################
    # Detect Robot-boundary collision by reading rectangle vertices' BGR value        #
    # Return 2 if bump                                                                #
    ###################################################################################
    if bump_boundary == 0:
        for vert_index, vert in enumerate(vertices):
            # Check boundaries for image array access
            if 0 <= vert[1] < frame.shape[0] and 0 <= vert[0] < frame.shape[1]:
                if (frame[vert[1]][vert[0]] == Boundary_Color).all():
                    bump_boundary = 2
                    break
    ###################################################################################
    # Detect Robot-ball collision by vector (use vector to calculate distance)        #
    # Return 3 if bump                                                                #
    # To calculate point P to line AB, consider vector AC,which is projection of      #
    # vector AP on Vector AB. AC has same direction as AB and its scale can be        #
    # obtained by [(AP*AB)/(AB*AB)]. If scale <1 then C is located between AB, length #
    # is equal to length of vector CP needless to say. If scale <=0 then length is    #
    # equal to length AP since AP has a opposite projection on AB. If scale >=1 then  #
    # length is equal to lengthBP clearly.                                            #
    ###################################################################################
    if bump_boundary == 0:
        for vert_index, vert in enumerate(vertices):
            line_pointA = [vert[0], vert[1]]
            end_index = 0 if vert_index == 3 else vert_index + 1
            line_pointB = [vertices[end_index][0], vertices[end_index][1]]
            vect_A_B = [line_pointB[0]-line_pointA[0], line_pointB[1]-line_pointA[1]]#Vector A2B
            vect_A_ball = [ball_pos[0] - line_pointA[0], ball_pos[1] - line_pointA[1]]#Vector A2Ball
            #Calculate the Projection of A2Ball on A2B i.e  (AdotB)/lengthAB^2 * vectorAB
            dot = vector_product(vect_A_ball[0],vect_A_ball[1],vect_A_B[0],vect_A_B[1],0)[1]
            ABlength_sqa = math.pow(math.hypot(vect_A_B[0], vect_A_B[1]),2)
            Projection_scale = dot/ABlength_sqa
            BallonEgde_x, BallonEgde_y = vect_A_B[0]*Projection_scale, vect_A_B[1]*Projection_scale
            vect_BallonEgde = [BallonEgde_x, BallonEgde_y]
            #Calculate ball to edge distance
            edge_ball_distance = 0
            if Projection_scale <= 0:
                rot_bump_position = line_pointA
                edge_ball_distance = math.hypot(vect_A_ball[0],vect_A_ball[1])
            elif Projection_scale >= 1:
                rot_bump_position = line_pointB
                vect_B_ball = [x1 - x2 for (x1, x2) in zip(vect_A_ball, vect_A_B)]
                edge_ball_distance = math.hypot(vect_B_ball[0], vect_B_ball[1])
            else:
                rot_bump_position = [x1 + x2 for (x1, x2) in zip(line_pointA, vect_BallonEgde)]
                vect_P_ball = [x1 - x2 for (x1, x2) in zip(vect_A_ball, vect_BallonEgde)]#Projection point to Ball
                edge_ball_distance = math.hypot(vect_P_ball[0], vect_P_ball[1])
            #Judge if bump with ball
            if edge_ball_distance <= 6:
                bump_boundary = 3
                print(RoboIndex,' bump ball --distance: ',edge_ball_distance)
                break
    return bump_boundary ,rot_bump_position #Second return only meaningful when bump_boundary == 3

def vector_product(p1_x, p1_y, p2_x, p2_y, unit_vector=0):
    """
    
    輸入兩項量，回傳值為外積、內積、角度、錯誤訊息()
    
    Description:
        CAlculate cross/dot product and angle of two given vector
    Parameters:
        p1, p2 are two endpoints
        unit_vector: set to 1 if given vectors are unit vector
    Return:
        cross product, dor product, angle, and error flag, 1 if there is error.
    """
    ValueErrorFlag = 0
    cro = p1_x * p2_y - p1_y * p2_x
    dot = p1_x * p2_x + p1_y * p2_y
    dot_angle = dot if unit_vector == 1 else dot/(math.hypot(p1_x,p1_y)*math.hypot(p2_x,p2_y))
    angle = 0
    try:
        angle = math.acos(dot_angle)
    except  ValueError:
        ValueErrorFlag = 1
        return cro,dot,0,ValueErrorFlag
    return cro,dot,angle,ValueErrorFlag

def robo_shoot(RoboIndex, cmd = 0):

    global ball_speed, ball_direction

    if 0 <= RoboIndex < len(st_A.AllRobots):
        setup = st_A.AllRobots[RoboIndex]
    else:
        return
    
    d_ball_x = ball_pos[0]-setup.pos[0]
    d_ball_y = ball_pos[1]-setup.pos[1]
    distance = math.hypot(d_ball_x, d_ball_y)
    unit_ball_x = d_ball_x/distance
    unit_ball_y = d_ball_y/distance
    cross,dot,ball_angle_dif,err = vector_product(setup.dir[0], setup.dir[1], unit_ball_x, unit_ball_y,1)
    ball_angle_dif = math.pi if err and dot <= -1 else ball_angle_dif
    if ball_angle_dif > Robo_kick_angle_threshold and cmd == 0:
        print(RoboIndex,' kick angle too large, miss')
        return
    elif (ball_angle_dif - 0.5 * math.pi) > Robo_kick_angle_threshold and cmd == 1:
        print(RoboIndex,' kick angle too large, miss')
        return
    elif (ball_angle_dif - 1 * math.pi) > Robo_kick_angle_threshold and cmd == 2:
        print(RoboIndex,' kick angle too large, miss')
        return
    else:
        #If ball is not too far
        if distance <= Kickable_distance:
            if cmd == 0:
                ball_direction = [setup.dir[0],setup.dir[1]]
                ball_speed = Ball_kicked_speed
            elif cmd == 1:
                ball_direction = [unit_ball_x,unit_ball_y]
                ball_speed = Ball_kicked_speed * 1  # change
            elif cmd == 2:
                ball_direction = [-setup.dir[0],-setup.dir[1]]
                ball_speed = Ball_kicked_speed
            else:
                pass
            
            print('Robo ',RoboIndex,' Play a shoot')   
        else:
            print(RoboIndex,' too far, miss')
            return

def robo_pass(RoboIndex):
    """Robot Pass"""
    global ball_speed, ball_direction

    if 0 <= RoboIndex < len(st_A.AllRobots):
        setup = st_A.AllRobots[RoboIndex]
    else:
        return
    
    #Angle is correct then kick
    d_ball_x = ball_pos[0]-setup.pos[0]
    d_ball_y = ball_pos[1]-setup.pos[1]
    distance = math.hypot(d_ball_x, d_ball_y)
    unit_ball_x = d_ball_x/distance
    unit_ball_y = d_ball_y/distance
    cross,dot,ball_angle_dif,err = vector_product(setup.dir[0], setup.dir[1], unit_ball_x, unit_ball_y,1)
    ball_angle_dif = math.pi if err and dot <= -1 else ball_angle_dif
    if ball_angle_dif > Robo_kick_angle_threshold:
        print(RoboIndex,' PASS angle too large, miss')
        return
    else:
        #If ball is not too far
        if distance <= Kickable_distance:
            ball_direction = [setup.dir[0],setup.dir[1]]
            ball_speed = Ball_passing_speed
            print('Robo ',RoboIndex,' Play a PASS')   
        else:
            print(RoboIndex,'PASS too far, miss')
            return

def ball_move():
    """Simulation of ball - Optimized version"""
    global ball_speed,ball_direction
    # 初始化變量，避免 finally 塊中出現 UnboundLocalError
    d_ball_X, d_ball_Y = 0, 0
    try:
        if ball_speed <= 0:
            return
        # 優化：只檢查邊界顏色，減少顏色檢查次數（從7種減少到1種）
        # 機器人碰撞應該通過其他機制檢測
        Boundary_Color_array = np.array(Boundary_Color)
        # 預先計算場地尺寸，避免重複訪問
        field_h, field_w = empty_field.shape[0], empty_field.shape[1]
        
        speed_int = int(ball_speed)
        for i in range(speed_int):
            bound_x, bound_y = 0, 0

            # 檢查 X 方向碰撞 - 優化：減少重複計算
            test_x = d_ball_X + ball_direction[0]
            size_offset_x = BallHalfSize if test_x >= 0 else -BallHalfSize
            
            # 檢查兩個檢查點（球的上邊和下邊）
            chk_y1 = int(round(ball_pos[1] + d_ball_Y + size_offset_x))
            chk_x1 = int(round(ball_pos[0] + test_x + size_offset_x))
            if 0 <= chk_y1 < field_h and 0 <= chk_x1 < field_w:
                if np.array_equal(empty_field[chk_y1][chk_x1], Boundary_Color_array):
                    bound_x = 1
            
            if not bound_x:
                chk_y2 = int(round(ball_pos[1] + d_ball_Y - size_offset_x))
                chk_x2 = int(round(ball_pos[0] + test_x + size_offset_x))
                if 0 <= chk_y2 < field_h and 0 <= chk_x2 < field_w:
                    if np.array_equal(empty_field[chk_y2][chk_x2], Boundary_Color_array):
                        bound_x = 1

            # 檢查 Y 方向碰撞
            test_y = d_ball_Y + ball_direction[1]
            size_offset_y = BallHalfSize if test_y >= 0 else -BallHalfSize
            
            chk_y3 = int(round(ball_pos[1] + test_y + size_offset_y))
            chk_x3 = int(round(ball_pos[0] + d_ball_X + size_offset_y))
            if 0 <= chk_y3 < field_h and 0 <= chk_x3 < field_w:
                if np.array_equal(empty_field[chk_y3][chk_x3], Boundary_Color_array):
                    bound_y = 1
            
            if not bound_y:
                chk_x4 = int(round(ball_pos[0] + d_ball_X - size_offset_y))
                chk_y4 = int(round(ball_pos[1] + test_y + size_offset_y))
                if 0 <= chk_y4 < field_h and 0 <= chk_x4 < field_w:
                    if np.array_equal(empty_field[chk_y4][chk_x4], Boundary_Color_array):
                        bound_y = 1

            # 更新位置和方向（保持原有邏輯）
            if bound_x:
                ball_direction[0] = -ball_direction[0]
            else:
                d_ball_X += ball_direction[0]
            
            if bound_y:
                ball_direction[1] = -ball_direction[1]
            else:
                d_ball_Y += ball_direction[1]

    except ZeroDivisionError:
        d_ball_X, d_ball_Y = 0, 0
    except Exception as e:
        d_ball_X, d_ball_Y = 0, 0
    finally:
        ball_pos[0] = ball_pos[0] + int(round(d_ball_X))
        ball_pos[1] = ball_pos[1] + int(round(d_ball_Y))
        ball_speed = max(0, ball_speed - Ball_n_acce)

def get_strategy():
    """Simulator get strategy"""
    try:
        cmd_a = st_strategy.strategy()
        #cmd_b = st_B.strategy()
    except AttributeError:
        return 0,0
    #print('CMD2',cmd_a)
    return 1, cmd_a

def parse_strategy_field(PK_x, FB_x, FB_y, Penalty_y):
    """Parse field parameter to strategy"""
    try:
        sideA = 1 if Side == 1 else -1
        st_strategy.strategy_update_field(sideA, BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y)
        # st_B.strategy_update_field(sideB, BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y)
    except AttributeError:
        pass

def return_sent_cmd(send_data, sent):
    """Pass sent command to strategy"""
    try:
        st_strategy.get_sent_cmd(send_data[0:3], sent)
        # st_B.get_sent_cmd(send_data[3:6], sent)
    except AttributeError:
        pass

def upd_strategy_position():
    """Simulator Opject position update"""

    # 需要傳遞所有機器人的資訊
    team_d = [r.dir for r in st_A.AllRobots[0:3]]
    team_p = [r.pos for r in st_A.AllRobots[0:3]]
    opp_d  = [r.dir for r in st_A.AllRobots[3:6]]
    opp_p  = [r.pos for r in st_A.AllRobots[3:6]]

    _strategy_update_info(team_d, team_p, opp_d, opp_p, ball_pos)

def reset_obj(Robo_start_x,Robo_start_y,Keeper_start_x):
    ball_pos[0] = int(CENTER[0] + Robo_start_x)
    ball_pos[1] = int(CENTER[1])
    st_A.ball_pos = ball_pos
    
    # 設定所有 6 個機器人的初始位置
    # Team A (0, 1, 2)
    st_A.Player1.pos = [float(CENTER[0]), float(CENTER[1])]
    st_A.Player2.pos = [float(CENTER[0] - 100), float(CENTER[1] - 50)]
    st_A.Player3.pos = [float(CENTER[0] - 100), float(CENTER[1] + 50)]
    
    # Team B (3, 4, 5) (OPPO)
    st_A.OPPO1.pos = [float(CENTER[0] + Keeper_start_x), float(CENTER[1])]
    st_A.OPPO2.pos = [float(CENTER[0] + 100), float(CENTER[1] - 50)]
    st_A.OPPO3.pos = [float(CENTER[0] + 100), float(CENTER[1] + 50)]
    
    print(round(st_A.Player1.pos[0],2),round(st_A.Player1.pos[1],2))


if __name__ == '__main__':

    for index,position in enumerate(BOUNDARY):
        BOUNDARY[index] = (int(position[0]*SCALE)+OFFSET[0],int(position[1]*SCALE)+OFFSET[1])
    center_x = (BOUNDARY[0][0] + BOUNDARY[1][0])/2
    center_y = (BOUNDARY[0][1] + BOUNDARY[7][1])/2
    CENTER = [int(center_x),int(center_y)]
    print('Virtual Field Center Position',CENTER)

    # Calculate field building parameter
    In_Field_Width = abs(BOUNDARY[0][0] - BOUNDARY[1][0])  
   
    In_Field_Height = abs(BOUNDARY[0][1] - BOUNDARY[7][1])
    FB_x           = int(In_Field_Width*80/340)
    FB_y           = int(In_Field_Height*40/180)
    PK_x           = int(In_Field_Width*110/340)
    Penalty_y      = int(In_Field_Height*65/180)
    Center_Circle  = int(In_Field_Height*30/180)
    Robo_start_x   = int(In_Field_Width*80/340)
    Robo_start_y   = int(In_Field_Height*40/180)
    Keeper_start_x = int(In_Field_Width*160/340)
    GA_x            = int(In_Field_Width*150/340)
    GA_y            = int(In_Field_Height*45/180)
    
    cv2.namedWindow('my image')
    cv2.setMouseCallback('my image',CV_event)#228行
    # Draw empty field
    empty_field = np.zeros((1080,1920,3), np.uint8)
    empty_field = cv2.resize(empty_field, ( int(FRAME_WIDTH) , int(FRAME_HEIGHT) ))
    
    draw_empty(empty_field,FB_x,FB_y,PK_x,Penalty_y,Center_Circle, GA_x, GA_y)
    # st_A.Initialize(CMD,st_A.Player.pos) # Initialize 可能需要修改以不依賴單一 Player
    st_strategy.Initialize()
    
    reset_obj(Robo_start_x,Robo_start_y,Keeper_start_x)
    #Parse field parameter to strategy and initialize strategy
    sideA = 1 if Side == 1 else -1
    st_strategy.strategy_update_field(sideA, BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y) #更正後

    parse_strategy_field(PK_x, FB_x, FB_y, Penalty_y)

    cmd_a, cmd_b = [None]*3, [None]*3

    start = 0 #start after enter 's'
    strategy_timer, send_timer = 0, 0
    
    while True:
        # Start timer
        tStart = time.time()

        # copy empty field and draw object
        frame = empty_field.copy()
        draw_object(frame,FB_x,FB_y,PK_x,Penalty_y,Center_Circle, GA_x, GA_y)
        

        #If ball is in the goal area then this match is finished.
        if st_A.ball_pos[0] <= BOUNDARY[0][0] or st_A.ball_pos[0] >= BOUNDARY[1][0]\
                or st_A.ball_pos[1] <= BOUNDARY[0][1] or st_A.ball_pos[1] >= BOUNDARY[7][1]:
            start, mode = 0, 1
            print(st_A.ball_pos)

        
        if start:
        ###################################### strategy ##########################################
            print('strategy_timer',strategy_timer)
            print('send_timer',send_timer)
            if strategy_timer == 0:
                # Update strategy information first and then get strategy
                upd_strategy_position()
                # Get stategy
                success,cmd_atmp = get_strategy()
                cmd_a = cmd_atmp if success else cmd_a
                print('CMD3',cmd_a)
                print('send_timer = ',send_timer)

            if send_timer == 0:
                # Send(simulate) command
                # CMD 長度應為 robo_num (6)
                while len(CMD) < robo_num:
                    CMD.append('N1')
                
                # 這裡假設 strategy 只回傳 Team A (3隻) 的指令
                # 若 cmd_a 是 list of 3 commands:
                if isinstance(cmd_a, list) and len(cmd_a) >= 3:
                     CMD[0:3] = cmd_a
                else:
                    # 如果 strategy 回傳格式不同，需自行調整
                    pass
                    
                # Robo_command[3:6] = cmd_b
                return_sent_cmd(CMD, True)
            else:
                return_sent_cmd(cmd_a, False)

            print('CMD4',CMD)
            allmovement()
            ball_move()

            # Update timer#延遲部分
            # strategy_timer = strategy_timer + 1 if strategy_timer < STRATEGY_UP-1 else 0
            # send_timer = send_timer + 1 if send_timer < SEND_UP-1 else 0
#
        # Show the result of this loop
        # print(ball_pos[0],ball_pos[1])
        cv2.imshow('my image', frame )
        
        ################################## Keyboard detector #####################################
        k = cv2.waitKey(1) 
        if  k == ord('q') or k == 27:  # q 或 ESC 都能退出:
            break
        elif  k == ord('s'):# Start
            start, mode = 1, 0
        elif  k == ord('p'):# Pause
            start, mode = 0, 1
            
        elif  k == ord('r'):# Reset Simulation
            start, mode = 0, 1
            reset_obj(Robo_start_x,Robo_start_y,Keeper_start_x)
        elif  k == ord('c'):# Open Cursor Tracker
            Cursor_Position_Show = not Cursor_Position_Show
            print('Cursor','On' if Cursor_Position_Show else 'OFF')
        elif  k == ord('k'):# Kick all robot outside the field
            start, mode = 0, 1
            # 重置所有機器人位置
            for r in st_A.AllRobots:
                r.pos = [0.0, 0.0]
        elif  k == ord('o'): # Set robot position
            start, mode = 0, 2
        #test
        elif  k == ord('t'): # test
            mode = 3
        #test
        elif  mode == 3 : 
            # 測試模式：只對第一隻機器人下達指令
            # 注意：需維持 CMD 結構
            while len(CMD) < robo_num:
                CMD.append('N1')
            
            if k == ord('a'):
                CMD[0] = 'a1'
                allmovement()
                ball_move()
                CMD[0] = 'N1' # 執行完恢復，避免連續移動（依需求調整）
                print('Player.pos = ',round(st_A.Player1.pos[0],3),round(st_A.Player1.pos[1],3))
            elif k == ord('d'):
                CMD[0] = 'd1'
                allmovement()
                ball_move()
                CMD[0] = 'N1'
                print('Player.pos = ',round(st_A.Player1.pos[0],3),round(st_A.Player1.pos[1],3))
            elif k == ord('n'):
                CMD[0] = 'q1' #向左轉 
                allmovement()
                ball_move()
                CMD[0] = 'N1'
                print('Player.pos = ',round(st_A.Player1.pos[0],3),round(st_A.Player1.pos[1],3))
            elif k == ord('m'):
                CMD[0] = 'e1' #向右轉 
                allmovement()
                ball_move()
                CMD[0] = 'N1'
                print('Player.pos = ',round(st_A.Player1.pos[0],3),round(st_A.Player1.pos[1],3))

        elif  mode == 2 and k <= ord('5') and k >= ord('0'):
            Setup_Index = k-ord('0') #Robo 0 to 5 index 0 to 5

        while time.time() - tStart < 1/FRAME_FPS:
            pass

    # Close all frames #
    cv2.destroyAllWindows()
