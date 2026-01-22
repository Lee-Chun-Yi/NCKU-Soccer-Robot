import cv2
import math
import numpy as np
from simulator import vec_cal_func as vec
import os
import time
 

# 紀錄場地資訊變數(ball....)
global  BOUNDARY, CENTER

BOUNDARY = [(0, 0), (360, 0), (360, 70), (375, 70), (375, 290), (360, 190), (360, 260), (0, 260),(0,190), (-15, 190),(-15, 70), (0, 70)]
#BOUNDARY = [(140, 70), (870, 70), (870, 170), (900, 170), (900, 350), (870, 350), (870, 450), (140, 450), (140, 350),(110, 350), (110, 170), (140, 170)]
CENTER = [180, 130] #list[int]=[cx,cy] 紀錄中心位置

"""每次都忘記 這邊註記一下
    7               6
9   8               5   4
          中心 
10  11              2   3
    0               1
"""
x = 0
y = 1
ball_pos = [0, 0]
player_pos = [0,0]
player_dir = [0,0]
player_dir_right = [0,0]
player_dir_left = [0,0]
oppo_pos = [0,0]
PK = [0, 0]
FK = [0, 0]
FB = [0, 0]
PENALTY_AREA = [0, 0]
GOAL_AREA = [0, 0]
goal_point = [0,0]
ready_point = [0,0]
ready_point2 = [0,0]
target_vec_90 = [0,0]
target_vec_270 = [0,0]
fb_st_7_st = 0
fb_st_3_st = 0
fb_st_2_st = 0
find_gp_st = 0
fb_st = 0
temp_player_pos = [0,0]

last_ball_pos = None
_turn_cmd_hist = []
_turn_cmd_max = 4






#----------微調用----------
set_min_dis = 25
set_min_ang = 30
ready_point_2_mag = 30
set_turn_min_ang = 60
boundary_top_y = 0 
boundary_bottom_y = 260
#----------微調用----------

def strategy_update_field(side, boundary, center, pk_x, fb_x, fb_y, penalty_y, ga_x, ga_y):
    """
    Description:
        Pass field information into strategy system.
        This will be called only once by the simulator.
    Parameter:
        param1: 0/1 -> 1 if left attack right, -1 if right attack left
        param2: list[tuple(int)] -> 12 boundary points of the field
        param3: list[int] -> center of the field
        param4: int -> x coordinate of penalty kick point w.r.t center
        param5: int -> x coordinate of free ball point w.r.t center
        param6: int -> y coordinate of free ball point w.r.t center
        param7: int -> x coordinate of goal area w.r.t center
        param8: int -> y coordinate of goal area w.r.t center
    """
    # Your code
    global BOUNDARY, CENTER, PK, FK, FB, PENALTY_AREA, GOAL_AREA

    BOUNDARY = boundary

    CENTER = center

    PK[x] = pk_x

    FB[x] = fb_x
    FB[y] = fb_y

    PENALTY_AREA[x] = PK[x]
    PENALTY_AREA[y] = penalty_y

    GOAL_AREA[x] = ga_x
    GOAL_AREA[y] = ga_y

def Initialize():
    """
    Description:
        Initialuze the strategy.
        This function will be called by the simulator before a simulation is started.
    """
    # Your code

   

def draw_on_simulator(frame):
    """
    Description:
        Draw whatever you want on the simulator.
        Before the simulator update window, it will call this function and you can just draw anything you want.
        This function will be called everytime the simulator is going to update frame.
    Parameter:
        param1: numpy array -> the frame that will be displayed
    Return:
        retva1: numpy array -> the frame that will be displayed
    """
    # Your code
    
    global target_vec_90,ready_point2,ready_point,ready_point2
    #Tips
    #純粹的標點而已 而且根據你們得變數宣告也會有差 給你們模板 實際上可以畫很多點(複製好多次)
    #simulator 呼叫跟存取的時候都是 frame 可以知道應該是要在 frame 上加些東西
    # 假設 player_pos = (x, y) 位置
# 假設 player_dir = (dx, dy) 方向單位向量 (或未正規化)

    start_point = (int(player_pos[0]), int(player_pos[1]))  # 起點
    scale = 50  # 箭頭長度倍率，可以依需要調整
    end_point = (int(player_pos[0] + player_dir[0] * scale),
                int(player_pos[1] + player_dir[1] * scale))  # 箭頭終點 = 位置 + 方向 * 長度

    # 畫箭頭
    cv2.arrowedLine(frame, start_point, end_point, (0, 250, 250), 3, tipLength=0.3)

    # 在箭頭起點標註文字
    cv2.putText(frame, f"({start_point[0]}, {start_point[1]})", 
                start_point, cv2.FONT_HERSHEY_SIMPLEX, 
                0.8, (0, 250, 250), 2, cv2.LINE_AA)
    
    target = [int(goal_point[x]),int(goal_point[y])]
    cv2.circle(frame, (goal_point[x],goal_point[y]) , 10 , (0,250,250) , 3, 8, 0)
    cv2.putText(frame, str(goal_point[x]) + ' ' + str(goal_point[y]), (goal_point[x],goal_point[y]), cv2.FONT_HERSHEY_SIMPLEX,1, (0,250,250), 2, cv2.LINE_AA)

    target = [int(ready_point[x]),int(ready_point[y])]
    cv2.circle(frame, (ready_point[x],ready_point[y]) , 10 , (0,250,0) , 3, 8, 0)
    cv2.putText(frame, str(ready_point[x]) + ' ' + str(ready_point[y]), (ready_point[x],ready_point[y]), cv2.FONT_HERSHEY_SIMPLEX,1, (0,250,0), 2, cv2.LINE_AA)

    target = [int(ready_point2[x]),int(ready_point2[y])]
    cv2.circle(frame, (ready_point2[x],ready_point2[y]) , 10 , (210,100,150) , 3, 8, 0)
    cv2.putText(frame, str(ready_point2[x]) + ' ' + str(ready_point2[y]), (ready_point2[x],ready_point2[y]), cv2.FONT_HERSHEY_SIMPLEX,1, (210,100,150), 2, cv2.LINE_AA)

    ready_point_int = [int(ready_point[0]), int(ready_point[1])]
    cv2.arrowedLine(frame, start_point, ready_point_int, (250, 250, 250), 3, tipLength=0.3)
    ready_point_int2 = [int(ready_point2[0]), int(ready_point2[1])]
    end_point2 = [int(ready_point_int[x] + 30 * target_vec_90[x]),int(ready_point_int[y] + 30 *target_vec_90[y])]
    cv2.arrowedLine(frame, ready_point_int, end_point2, (100, 250, 250), 1, tipLength=0.3)
    cv2.arrowedLine(frame, start_point, ready_point_int2, (100, 150, 250), 1, tipLength=0.3)


    
    return frame

def _first_point(value, default=None):
    if isinstance(value, (list, tuple)):
        if value and isinstance(value[0], (list, tuple)) and len(value[0]) >= 2:
            return value[0]
        if len(value) >= 2 and all(isinstance(v, (int, float)) for v in value[:2]):
            return value
    return default if default is not None else [0, 0]


def Update_Robo_Info(teamD, teamP, oppoP, ballP):
    """
    Description:
        Pass robot and ball info into strategy.
        This function will be called before everytime the simulator ask for strategy
    Parameter:
        param1: list[list[float]] -> [x,y] for our teamate robot direction
        param2: list[list[int]] -> [x,y] for our teamate robot position
        param3: list[list[int]] -> [x,y] for opponent's robot position
        param4: list[int] -> [x,y] for ball position
    """
    # Your code
    #print('Updating Robot Info')
    global ball_pos, goal_point, ready_point, target_vec_90, temp_player_pos, ready_point2,target_vec_270
    global fb_st_3_st, find_gp_st, last_ball_pos, player_pos, player_dir, oppo_pos, fb_st
    player_pos = _first_point(teamP)
    player_dir = _first_point(teamD, default=[1.0, 0.0])
    oppo_pos = _first_point(oppoP)
    ball_pos = _first_point(ballP)
    OPPO_pos_list = [oppo_pos, oppo_pos]
    if(last_ball_pos == None):
        last_ball_pos = ball_pos
    if(last_ball_pos == last_ball_pos):
        print("ball change")
    last_ball_pos = ball_pos
    # 計算即時目標球點（隨對手/球位置更新）
    try:
        default_goal = [BOUNDARY[2][0], CENTER[1]]
        goal_point = choose_goal_point(BOUNDARY, OPPO_pos_list, 25, 30, ball_pos, default_goal, 10)
    except Exception:
        goal_point = [BOUNDARY[2][0], CENTER[1]]
    u_v_gp_to_bp1, _ = vec.vector_param(goal_point,ball_pos)
    ready_point = [int(ball_pos[x] + u_v_gp_to_bp1[x] * 15),int(ball_pos[y] + u_v_gp_to_bp1[y] * 5)]#
    #print("state_target",fb_st_3_st)
    #if(find_gp_st == 0):
        #計算ready_point2，優先以機器人靠近的那邊圍方向設置，除非會超出界外，才換邊
    if(player_pos[y] > ball_pos[y]):
        target_vec_90 = vec.rotate_vector(u_v_gp_to_bp1,90)
        ready_point2 = [int(ready_point[x] - target_vec_90[x] * ready_point_2_mag),int(ready_point[y] - target_vec_90[y] * ready_point_2_mag)]
        if(ready_point2[y] >= boundary_bottom_y):
            ready_point2 = [int(ready_point[x] + target_vec_90[x] * ready_point_2_mag),int(ready_point[y] + target_vec_90[y] * ready_point_2_mag)]
    elif(player_pos[y] < ball_pos[y]):
        target_vec_90 = vec.rotate_vector(u_v_gp_to_bp1,270)
        ready_point2 = [int(ready_point[x] - target_vec_90[x] * ready_point_2_mag),int(ready_point[y] - target_vec_90[y] * ready_point_2_mag)]
        if(ready_point2[y] <= boundary_top_y):
            ready_point2 = [int(ready_point[x] + target_vec_90[x] * ready_point_2_mag),int(ready_point[y] + target_vec_90[y] * ready_point_2_mag)]
    print("Update_Robo_Info")


def strategy():
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """
    
    # print("player position:",[[int(round(x)) for x in pair] for pair in player_p])#確定一下進來的數據
    # print("ball position:",[int(round(x)) for x in ball_p])

    global ball_pos,goal_point,ready_point,target_vec_90,fb_st_3_st,fb_st_2_st,ready_point2,temp_player_pos,find_gp_st,fb_st,last_ball_pos,fb_st_7_st,player_dir_right,player_dir_left,_turn_cmd_hist
    print("START")
    print("player_dir",player_dir)
    print("goal_point",goal_point)
    print("fb_st = =",fb_st)
    if fb_st != 4:
        _turn_cmd_hist = []
    player_dir_right = vec.rotate_vector(player_dir,90)
    player_dir_left = vec.rotate_vector(player_dir,270)
    if(last_ball_pos != ball_pos):
        time.sleep(0.3)
        find_gp_st = 1
    print("1---")
    if(fb_st == 0):
        print("fb_st = 0")
        fb_st = 1
        return ['N1','N1','N1']
    print("2---")
    if(fb_st == 1):
        print("fb_st = 1")
        bpos_forb = det_bpos_forb(ball_pos,player_pos,"right")
        print("det_bpos_forb:",bpos_forb)
        #time.sleep(30)
        if(bpos_forb == "behind"):
            fb_st = 3
            return ['N1','N1','N1']
        else:
            fb_st = 3
            return ['N1','N1','N1']
    print("3---")
    
    print("4---")
    if(fb_st == 2) :#or fb_st_2_st == 1  移動到ready_point2
        
        print("fb_st = 2")
        """
        fb_st = change_st_when_moving(fb_st,ball_pos,player_pos,"right")
        if(fb_st != 2):
            return ['N1','N1','N1']
        """
        u_v_p_to_r2, _ = vec.vector_param(player_pos,ready_point2)
        p_dir_to_r_ang ,_= vec.full_angle(player_pos,ball_pos,ready_point)
        _, p_dir_p_to_r2_ang = vec.vector_angle(player_dir_right,u_v_p_to_r2)
        p_dir_p_to_r2_ang = normalize_angle(p_dir_p_to_r2_ang)
        print("fb_st_2_st==",fb_st_2_st)
        #time.sleep(0.5)
        if(p_dir_to_r_ang < 40 or fb_st_2_st == 0 or fb_st_2_st == 1):
            command, fb_st, fb_st_2_st = move_to_point(fb_st, fb_st_2_st, p_dir_p_to_r2_ang, set_min_ang,ready_point2, player_pos, set_min_dis)
            return command              
        else: 
            fb_st = 3
            return ['N1','N1','N1']
    print("5---")        
                
    if(fb_st == 3):#移動到read_point,並調整成可以側踢的角度
        print("fb_st = 3")
        #time.sleep(10)
        fb_st = change_st_when_moving(fb_st,ball_pos,player_pos,"right")
        if(fb_st != 3):
            return ['N1','N1','N1']
        u_v_p_to_r, _ = vec.vector_param(player_pos,ready_point)
        if(player_dir[y] < 0):
            _, p_dir_p_to_r_ang = vec.vector_angle(player_dir_right,u_v_p_to_r)
        else:
            _, p_dir_p_to_r_ang = vec.vector_angle(player_dir_left,u_v_p_to_r)
        #p_dir_p_to_r_ang = normalize_angle(p_dir_p_to_r_ang)
        print("p_dir_p_to_r_ang",p_dir_p_to_r_ang)
        print("dis",math.hypot(ready_point[x] - player_pos[x],ready_point[y] - player_pos[y]))
        command, fb_st, fb_st_3_st = move_to_point(fb_st, fb_st_3_st, p_dir_p_to_r_ang, set_min_ang,
                           ready_point, player_pos, set_min_dis)
        if(fb_st_3_st == 0 or fb_st_3_st == 1):
            print("--2")
            """
            if(p_dir_p_to_r_ang > set_min_ang):
                fb_st = 1
                fb_st_3_st = 0
                return ['N1','N1','N1']
            """
            return command
    
    if(fb_st == 4):
        print("fb_st = 4")
        if(player_dir[y] < 0):
            _, p_dir_tvec = vec.vector_angle(player_dir,target_vec_270)
        else:
            _, p_dir_tvec = vec.vector_angle(player_dir,target_vec_90)
        print("--3")
        p_dir_tvec = normalize_angle(p_dir_tvec)

        if(abs(p_dir_tvec) > set_min_ang):
            command = turn(p_dir_tvec)
            cmd_head = command[0] if isinstance(command, (list, tuple)) and command else None
            if cmd_head in ("q1", "e1"):
                _push_turn_cmd(cmd_head)
                if _is_turn_alternating() and _side_vec_hits_goal(player_pos, player_dir):
                    fb_st = 7
                    fb_st_7_st = 1
                    _turn_cmd_hist = []
                    print("--4")
                    return player_action(player_dir, "right")
            print("--4")
            return command
        else:
            fb_st += 1
            print("--5")
            return ['N1','N1','N1']
       
    if(fb_st == 5 or fb_st == 6):
        find_gp_st = 0
        fb_st_2_st = 0
        fb_st_3_st = 0
        if(fb_st == 5):
            fb_st  = 7
        else:
            fb_st  = 1
        return ['N1','N1','N1']

    if(fb_st == 7):
        print("fb_st_7_st",fb_st_7_st)
        if(fb_st_7_st <= 2):
            command = player_action(player_dir, "right")
            fb_st_7_st += 1
            return command
        else:
            fb_st = 1
            fb_st_7_st = 0
            return ['N1','N1','N1']
def get_sent_cmd(sentcmd, update):
    """
    Description:
        Simulator will pass the received strategy and a sending state
    Parameter:
        param1: list[str] -> received command
        param2: bool -> sent or not
    """
    # Your code



def turn(angle):
    ##print(f'angle = {angle}')
    if(angle <= -set_turn_min_ang):
        return ['e1', 'N1','N1']
    elif(-set_turn_min_ang < angle <= 0):
        return ['e1','N1','N1']
    elif(0 < angle <= set_turn_min_ang):
        return ['q1','N1','N1']
    elif(set_turn_min_ang < angle ):
        return ['q1','N1','N1']
    
def choose_goal_point(
    BOUNDARY,
    oppo_p,
    ROB_RANG,
    KICKABLE_RANGE,
    ball_p,
    default_goal_point,
    BallHalfSize
):
    """
    依據邊界、守方兩名球員位置與各種參數，計算射門目標點。

    參數說明：
    - BOUNDARY: list/tuple，至少包含索引 2 與 5，且每個元素為 [x, y]
    - oppo_p:   對手位置列表，至少含 index 1 與 2，oppo_p[i] = [x, y]
    - ROB_RANG: 守門員/防守球員的阻擋半徑(在 y-軸上的遮擋範圍半徑)
    - KICKABLE_RANGE: 可接受的最小縱向(沿 y)可踢寬度
    - ball_p:  我方球位置 [x, y]
    - default_goal_point: 預設的射門目標 [x, y]
    - BallHalfSize: 球半徑(或判斷所需的等效「半徑」)，用於 2*BallHalfSize 的最小縫隙判斷

    回傳：
    - [goal_x, goal_y]
    """

    # === 初始化 ===
    goal_point = [None, None]

    # 射門 x 固定為球門線 x
    goal_point[0] = BOUNDARY[2][0]

    # 確保球門範圍的順序是從小到大（allow = [y_low, y_high]）
    allow = [BOUNDARY[2][1], BOUNDARY[5][1]] \
        if BOUNDARY[2][1] < BOUNDARY[5][1] else [BOUNDARY[5][1], BOUNDARY[2][1]]

    # === 第一位守門/防守者的阻擋範圍 ===
    block  = [oppo_p[1][1] - ROB_RANG, oppo_p[1][1] + ROB_RANG]

    # === 第二位防守者的阻擋範圍 ===
    #block2 = [oppo_p[2][1] - ROB_RANG, oppo_p[2][1] + ROB_RANG]
    block2 = [oppo_p[1][1] - ROB_RANG, oppo_p[1][1] + ROB_RANG]#測試用

    # === 把要扣除的所有區段放一起處理 ===
    blocks = [block, block2]

    # === 依序從 allow 扣除 blocks ===
    for b in blocks:
        new_allow = []
        for i in range(0, len(allow), 2):
            a1, a2 = allow[i], allow[i+1]

            # 如果 block 與此段無交集，整段保留
            if b[1] <= a1 or b[0] >= a2:
                new_allow.extend([a1, a2])
                continue

            # 上半段：保留 block 上方
            if b[0] > a1:
                new_allow.extend([a1, max(a1, min(b[0], a2))])

            # 下半段：保留 block 下方
            if b[1] < a2:
                new_allow.extend([max(a1, min(b[1], a2)), a2])

        # 清理零寬度或反序區間
        cleaned = []
        for i in range(0, len(new_allow), 2):
            y1, y2 = new_allow[i], new_allow[i+1]
            if y2 > y1:
                cleaned.extend([y1, y2])
        allow = cleaned

    # === 若所有可踢區間長度都 < 2*BallHalfSize 或 allow 為空，退回預設目標 ===
    if len(allow) == 0:
        return [default_goal_point[0], default_goal_point[1]]

    has_gap_ge_2ball = any(
        (allow[i+1] - allow[i]) >= 2 * BallHalfSize
        for i in range(0, len(allow), 2)
    )
    if not has_gap_ge_2ball:
        return [default_goal_point[0], default_goal_point[1]]

    # === 從剩下的 allow 中選出最好的射門目標 ===
    dist = None
    best_y = None
    for i in range(0, len(allow), 2):
        seg_w = allow[i+1] - allow[i]
        if seg_w >= KICKABLE_RANGE:
            mid_y = int((allow[i] + allow[i+1]) / 2)
            d = abs(ball_p[1] - mid_y)
            if dist is None or d < dist:
                dist = d
                best_y = mid_y

    # 若沒有任何一段 ≥ KICKABLE_RANGE，也退回預設目標
    if best_y is None:
        return [default_goal_point[0], default_goal_point[1]]

    # 成功選出 y
    goal_point[1] = best_y
    return goal_point

def move_to_point(main_state, sub_state, function_angle, min_angle_threshold,
                           target_point, robot_pos, distance_threshold,side = "right"):
    """
    控制分階段行為（轉向 → 前進 → 狀態切換）
"
    參數說明：
    ----------
    main_state : int
        外部主控制狀態變數（例如 fb_st）
    sub_state : int
        次階段控制變數（例如 fb_st_2_st）
    function_angle : float
        機器人朝向與目標方向的角度誤差
    min_angle_threshold : float
        可接受的最小角度誤差閾值，超過需要轉向
    target_point : list [x, y]
        目標點座標
    robot : object
        機器人物件，需包含 robot.pos = [x, y]
    distance_threshold : float
        抵達目標的距離容許範圍

    回傳：
    ----------
    action : list
        動作指令（例如 ['N1','N1','N1'] 或 rotate_func(angle)）
    main_state : int
        更新後的主狀態
    sub_state : int
        更新後的次狀態
    """
    global ball_pos
    #main_state = change_st_when_moving(main_state,ball_pos,player_pos,"right")
    # === 階段 0：先轉向 ===
    if(main_state == 2 or main_state == 3):
        if sub_state == 0:
            if abs(function_angle) > min_angle_threshold:
                # 角度偏差過大 → 執行轉向命令
                action = turn(function_angle)
                return action, main_state, sub_state
            else:
                # 已對準方向 → 進入下一階段
                sub_state += 1
                print("sub_state:",sub_state)
                #time.sleep(30)
                return ['N1', 'N1', 'N1'], main_state, sub_state

        # === 階段 1：前進至目標點 ===
        elif sub_state == 1:
            # 計算與目標距離
            distance_to_target = math.hypot(
                target_point[x] - robot_pos[x],
                target_point[y] - robot_pos[y]
            )
            print("distance_to_target",distance_to_target)
            if distance_to_target > distance_threshold:
                # 距離仍大 → 持續前進
                if(abs(function_angle) > min_angle_threshold):
                    sub_state = 0
                if(side == "right"):
                    if(player_dir[y] < 0):
                        return ['d1', 'N1', 'N1'], main_state, sub_state
                    if(player_dir[y] > 0):
                        return ['a1', 'N1', 'N1'], main_state, sub_state
                if(side == "left"):
                    if(player_dir[y] < 0):
                        return ['a1', 'N1', 'N1'], main_state, sub_state
                    if(player_dir[y] > 0):
                        return ['d1', 'N1', 'N1'], main_state, sub_state
                return ['N1', 'N1', 'N1'], main_state, sub_state
                    
            else:
                # 已接近目標 → 進入主狀態下一階段
                main_state += 1
                return ['N1', 'N1', 'N1'], main_state, sub_state
    else:
        return ['N1', 'N1', 'N1'], main_state, sub_state

def det_bpos_forb(ball_p,player_p,goald_ir):#依據球門方向的不同判斷球在球員前面還是後面
    if(goald_ir == "left"):
        if(ball_p[x] > player_p[x]):
            return "behind"
        elif(ball_p[x] < player_p[x]):
            return "front"
        else:
            return "same_x"
    else:
        if(ball_p[x] < player_p[x]):
            return "behind"
        elif(ball_p[x] > player_p[x]):
            return "front"
        else:
            return "same_x"
        
def is_enemy_blocking(enemy_list, goal_point, ready_point, threshold):
    """
    檢查是否有任一敵人擋在 ready_point → goal_point 這條射線附近。
    若敵人距離線段小於 threshold，回傳 True。

    參數：
    ----------
    enemy_list : list of [x, y]
        所有敵人位置，例如 [[x1, y1], [x2, y2], ...]
    goal_point : [x, y]
        射門目標（球門）
    ready_point : [x, y]
        開球起點（球所在位置）
    threshold : float
        若敵人距離射線小於此值，代表擋線

    回傳：
    ----------
    True  -> 有敵人擋線
    False -> 路徑安全
    """

    # 射線向量平方長度
    x1, y1 = ready_point
    x2, y2 = goal_point
    line_len_sq = (x2 - x1)**2 + (y2 - y1)**2

    # 遍歷所有敵人
    for enemy in enemy_list:
        x0, y0 = enemy

        if line_len_sq == 0:
            d = math.hypot(x0 - x1, y0 - y1)
        else:
            # 計算敵人到線段的投影點比例 t
            t = ((x0 - x1)*(x2 - x1) + (y0 - y1)*(y2 - y1)) / line_len_sq
            t = max(0, min(1, t))  # 限制在線段範圍內

            # 找出線上最近點
            proj_x = x1 + t * (x2 - x1)
            proj_y = y1 + t * (y2 - y1)

            # 計算敵人到該點距離
            d = math.hypot(x0 - proj_x, y0 - proj_y)

        # 若小於 threshold，代表擋線
        if d < threshold:
            return True

    # 沒有任何敵人擋線
    return False

def point_position(p1, p2, p3):
    """
    判斷 p3 相對於通過 p1、p2 的直線的位置，
    根據幾何上「上方 / 下方」的定義（不受斜率正負影響）。
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    # 處理垂直線
    if x2 == x1:
        if x3 > x1:
            return "above"
        elif x3 < x1:
            return "below"
        else:
            return "on the line"

    # 用直線方程 y = mx + b
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    y_line = slope * x3 + intercept

    # 幾何上比較 y 高低
    if y3 > y_line:
        return "above"
    elif y3 < y_line:
        return "below"
    else:
        return "on the line"
    
def normalize_angle(angle):
    """
    把角度規範到 (-180, 180) 區間。
    """
    angle = (angle + 180) % 360 - 180
    return angle


def _push_turn_cmd(cmd):
    if cmd is None:
        return
    _turn_cmd_hist.append(cmd)
    if len(_turn_cmd_hist) > _turn_cmd_max:
        _turn_cmd_hist.pop(0)


def _is_turn_alternating():
    if len(_turn_cmd_hist) < _turn_cmd_max:
        return False
    for idx in range(1, len(_turn_cmd_hist)):
        if _turn_cmd_hist[idx] == _turn_cmd_hist[idx - 1]:
            return False
    return True


def _side_vec_hits_goal(pos, dir_vec):
    if not pos or not dir_vec or len(pos) < 2 or len(dir_vec) < 2:
        return False
    try:
        px, py = float(pos[0]), float(pos[1])
        goal_x = float(BOUNDARY[2][0]) if len(BOUNDARY) > 2 else 360.0
        y1 = float(BOUNDARY[2][1]) if len(BOUNDARY) > 2 else 70.0
        y2 = float(BOUNDARY[5][1]) if len(BOUNDARY) > 5 else 190.0
        y_min, y_max = (y1, y2) if y1 <= y2 else (y2, y1)
    except Exception:
        return False

    for rot in (90, 270):
        side_vec = vec.rotate_vector(dir_vec, rot)
        if not side_vec or len(side_vec) < 2:
            continue
        vx, vy = side_vec[0], side_vec[1]
        if abs(vx) < 1e-6:
            continue
        t = (goal_x - px) / vx
        if t <= 0:
            continue
        y_hit = py + vy * t
        if y_min <= y_hit <= y_max:
            return True
    return False

def player_action(player_dir, goal_dir):
    """
    根據玩家方向與球門方向回傳對應的指令。
    
    參數:
        player_dir: (dx, dy) 玩家方向向量
        goal_dir:   'right' 或 'left'
    
    回傳:
        list of str，例如 ['j1','N1','N1']
    """
    dy = player_dir[y]
    
    # 對應右側球門的情況
    if goal_dir == 'right':
        if dy < 0:
            return ['j1', 'N1', 'N1']
        else:
            return ['h1', 'N1', 'N1']
    
    # 對應左側球門的情況（反轉）
    elif goal_dir == 'left':
        if dy < 0:
            return ['h1', 'N1', 'N1']
        else:
            return ['j1', 'N1', 'N1']
    
    else:
        raise ValueError("goal_dir 必須是 'right' 或 'left'")
    
def change_st_when_moving(main_st,ball_p,player_pos,side):
    if(main_st == 2):
        if(side == "right"):
            if(ball_p > player_pos):
                main_st = 6
                return main_st
            else: 
                return main_st
        if(side == "left"):
            if(ball_p < player_pos):
                main_st = 6
                return main_st
            else: 
                return main_st
            
    if(main_st == 3):
        if(side == "right"):
            if(ball_p <= player_pos):
                main_st = 6
                return main_st
            else: 
                return main_st
        if(side == "left"):
            if(ball_p >= player_pos):
                main_st = 6
                return main_st
            else: 
                return main_st
