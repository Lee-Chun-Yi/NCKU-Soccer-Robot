import math
import cv2
import random
from time import sleep

# Const
# BOUNDARY = [(20, 0), (360, 0), (360, 50), (380, 50), (380, 130), (360, 130), (360, 180), (20, 180),(20,130), (0, 130),
#             (0, 50), (20, 50)]
#*****
BOUNDARY = [(0, 0), (360, 0), (360, 70), (375, 70), (375, 290), (360, 190), (360, 260), (0, 260),(0,190), (-15, 190),
        (-15, 70), (0, 70)] # 從左上角開始順時針繞一圈，注意球門x是負的
# BOUNDARY = [(16, 0), (370, 0), (370, 65), (387, 65), (387, 186), (370, 186), (370, 250),(16,250), (16, 186),
#             (0, 186), (0, 65),(16,65)]
# ROB_RANG = 30
# KICKABLE_RANGE = 40
# ROUGH_RANG = 50
# ERROR_DISTANCE = 10
# ROTATE_ANGLE = 0.30
# SAFE_DIST = 43  # RoboRad/2 + ball radius
# WAY_ANGLE = {'F': 0, 'L': -math.pi / 2, 'R': math.pi / 2}
# Robot = ['N1','N1','N1']
# MOVE = {
#     'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [18, 10]},
#     'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [80, 8]},
#     'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [80, 8]},
#     'B': {'Norm': 's1', 'Bound': [None, 10]}
# }
ROB_RANG = 15

KICKABLE_RANGE = 20//2
# ROUGH_RANG = 50//2
ROUGH_RANG = 30

# ERROR_DISTANCE = 10//2
ERROR_DISTANCE = 8
ROTATE_ANGLE = 0.25 #****從0.25調到0.4
# SAFE_DIST = 43//4  # RoboRad/2 + ball radius
# SAFE_DIST = 16  # RoboRad/2 + ball radius #### SOUND GOOD
SAFE_DIST = 13  # RoboRad/2 + ball radius4.5

WAY_ANGLE = {'F': 0, 'L': -math.pi / 2, 'R': math.pi / 2}
MOVE = {
    'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [6, 3]},
    'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [25, 5]},
    'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [25, 7]},
    'B': {'Norm': 's1', 'Bound': [None, 2]}
}


# Condition
ALLOW_MOVE_WAY = ['L', 'R']
# Field Variable
BOUNDARY = []  # [(x, y), ...]
CENTER = (0, 0)
ori_center = []
ga_x = ga_y = 0

# Ball, Robot Parameter

ball_p = [0, 0]
player_id = 0
oriplayer_p = [[0, 0],[0,0],[0,0]]
oriplayer_d = [[0, 0],[0,0],[0,0]]
oppo_id = 3
oppo_p = [[0, 0],[0,0],[0,0]]
oppo_p1 = [0, 0]
oppo_p2 = [0, 0]
# Anticipate point

stage = 0
kick_goal = [0, 0]
kick_dir = [0.0, 0.0]
first_arri = [0, 0]
kick_point = [0, 0]
kick_p = [0, 0]
kick_d = [0.0, 0.0]
kick_way = ""
move_way = ""
move_dir = [0.0, 0.0]
toward_dir = [0.0, 0.0]
kick_flag = False
kicked = False
runaway = 1
robot1_kicked =False
fix_problem = 4
SECOND_ERROR_DIST = 30
count = 0 
count_2 = 0
count_3 = 0
count_4 = 0
runaway = 0
select = ""
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
    global BOUNDARY, ori_center
    ori_center = center
    for pos in boundary:
        BOUNDARY.append((pos[0], pos[1]))

    pass


def Initialize():
    """
    Description:
        Initialuze the strategy.
        This function will be called by the simulator before a simulation is started.
    """
    # Your code
    print("Your Player is Robot", player_id)
    print("Obstacle is Robot", oppo_id)
    global stage,job,Robot,runaway,count_2,count_3
    stage = 1
    job = 2
    Robot = ['N1','N1','N1']
    global kick_goal
    global goal,shown
    global count,BOUNDARY,count_4
    shown = 0
    goal = 1
    runaway = 1
    # BOUNDARY = [(20, 0), (360, 0), (360, 50), (380, 50), (380, 130), (360, 130), (360, 180), (20, 180),(20,130), (0, 130),
    #         (0, 50), (20, 50)]
    BOUNDARY = [(16, 0), (370, 0), (370, 65), (387, 65), (387, 186), (370, 186), (370, 250),(16,250), (16, 186),
            (0, 186), (0, 65),(16,65)]
    kick_x = 0.75*abs(BOUNDARY[0][0] - BOUNDARY[1][0]) + BOUNDARY[0][0]
    kick_y = (BOUNDARY[1][1] + BOUNDARY[6][1])//2
    kick_goal = [kick_x,kick_y]
    count = 0
    count_2 = 0
    count_3 = 0
    count_4 = 0
    runaway = 0
    # output stratgy
    with open('strategy.txt', 'a') as f:
        f.write('\n')
        f.write('New strategy(pos3 3): ')
    pass


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
    if stage > 1:
        cv2.circle(frame, (int(first_arri[0]), int(first_arri[1])), 3, (237, 183, 217), -1)
        # cv2.line(frame, (ball_p[0], ball_p[1]), (kick_goal[0], kick_goal[1]), (0, 0, 255), 2)
        # cv2.circle(frame, (int(player_p[0]), int(player_p[1])), 3, (0, 0, 255), -1)
    if stage > 2:
        cv2.circle(frame, (int(kick_point[0]), int(kick_point[1])), 3, (232, 122, 63), -1)

    if kick_goal:
        cv2.circle(frame, (int(kick_goal[0]), int(kick_goal[1])), 3, (232, 122, 63), -1)
    return frame


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
    global oriplayer_d, oriplayer_p, player_id, oppo_id, oppo_p1, oppo_p2, ball_p
    i = 0
    for i in range(3):
        oriplayer_d[i] = teamD[i]
        oriplayer_p[i] = teamP[i]
    oppo_p[0] = oppoP[oppo_id - 3]  # 為什麼‘-3？’ 為了歸0？(以得出list中的第一個的值)因為一開始opp_id是3
    oppo_p[1] = oppoP[oppo_id - 3 + 1]
    ball_p = ballP
    #****
    print("[DEBUG] oriplayer_d:", oriplayer_d)
    print("[DEBUG] oriplayer_p:", oriplayer_p)
    print("[DEBUG] oppo_p:", oppo_p)
    print("[DEBUG] ball_p:", ball_p)

    pass

##################################################
def strategy():

    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """
    player_id = 0 #player_id和哪台機器人的關係是？ 還是沒有關係？
    global stage
    global job
    global Robot,kick_goal,fix_problem,count,runaway,count_2,count_3,count_4
    Robot = ['N1','N1','N1']
    # 看敵方的位置決定往上走還是往下走
    block = []
    block.append(oppo_p[0][1] - ROB_RANG)  # oppo_p[1]是robo0的y
    block.append(oppo_p[0][1] + ROB_RANG)  # ROB_RANG的意思？
    block.append(oppo_p[1][1] - ROB_RANG)  # oppo_p[1]是robo0的y
    block.append(oppo_p[1][1] + ROB_RANG)  # ROB_RANG的意思？
    block.sort()
    print("block",block)
    space1 = abs(block[0] - BOUNDARY[3][1])
    space2 = abs(block[3] - BOUNDARY[4][1])
    print("kick_goal",kick_goal)
    # 計算上面空間大還是下面空間大
    if space1 > space2 :
        select = "top"

    else :
        select = "bottom"

    # Your code
    print("stage", stage)
    ###############################
    # fix_problem = 4
    # job = 2
    # sleep(6)      
    # if fix_problem == 1 : 
    #     if count < 150:
    #         count += 1
    #         return ['N1',"g1","N1"]
#修正一些機器人初始的問題
    if fix_problem == 1 : 
        if count < 10:
            count += 1
            return ['N1',"W1","N1"]
        
        else : 
            fix_problem = 2
     
    if fix_problem == 2 :
        if count_2 < 4:
            count_2 += 1
            return ['N1','u1','N1']
        
        else:
            fix_problem = 3
        
    if fix_problem == 3 :
        if count_3 < 500:
            count_3 += 1
            return ['N1','h2','N1']
        else:
            if count_4 < 3 :
                count_4 += 1
                print("First Job Problem complete kick_goal = ",kick_goal)
                return ['N1','x1','N1']
            else :
                fix_problem = 4
#正常比賽的狀況
    if job == 2:
        print("Second job start")
        player_id = 0
#Stage 1 — moverobot：定位好機器人要到球的後面
#arrivepoint() → 計算一個目標點 first_arri，從球後方30cm出發。

#Stage 2 — findball：走到 first_arri
#控制機器人往目標前進（用轉向、前進），而且還要防止直接撞到球（SAFE_DIST）。

#Stage 3 — rotatedirection：調整方向
#讓機器人朝著預期要踢的方向（kick_goal）轉向。

#Stage 4 — 開始踢球
#靠近球後，發送踢球指令（例如 'h1' 或 'j1'）。

#進stage 1 
    if stage != 4:
        #轉圈圈進到這裡
        return moverobot(player_id,kick_goal)

    if stage == 4:
        global goal
        global kick_way
        # print('play',oriplayer_p[0],'opp1',oppo_p[0],oppo_p[1])
        global move_way
        # print("XXX",move_way)
        print(job)
        if job == 1 and player_id == 2:
            print("First job done",kick_way)
            global kicked
            if (kicked):
                print("GOOD JOB")
                global runaway
                if runaway < 20:
                    runaway += 1
                    if kick_way == 'L':
                        return ['N1', 'N1', 'd1']
                    elif kick_way == 'R':
                        return ['N1', 'N1', 'a1']
                stage = 1
                job = 2
                # global kick_goal
                kick_goal[0] = int(0.75 * (BOUNDARY[1][0] - BOUNDARY[0][0]) + BOUNDARY[0][0])
                print(0.75 * (BOUNDARY[1][0] - BOUNDARY[0][0]),BOUNDARY[0][0])
                if select == "top":
                    kick_goal[1] = int(block[0] + BOUNDARY[0][1])//2
                elif select == "bottom" :
                    kick_goal[1] = int(block[3] + BOUNDARY[6][1])//2
                print("First recalculate kick_goal",kick_goal)        
                # kick_goal[1] = (BOUNDARY[2][0] + BOUNDARY[1][0]) // 2
                
                print("Nice",stage,job)
                kicked = False
            if(not kicked):
                return ['N1', 'N1', MOVE[kick_way]['Kick']]
        if job == 2:
            print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
            stage = 1
            print(ball_p)
            # block = []
            # block.append(oppo_p[0][1] - ROB_RANG)  # oppo_p[1]是robo0的y
            # block.append(oppo_p[0][1] + ROB_RANG)  # ROB_RANG的意思？
            # block.append(oppo_p[1][1] - ROB_RANG)  # oppo_p[1]是robo0的y
            # block.append(oppo_p[1][1] + ROB_RANG)  # ROB_RANG的意思？
            # block.sort()
            # print("block",block)
            if select == "top":
                if(ball_p[1]) > block[0] and ball_p[0] < oppo_p[0][0]:
                    kick_goal[1] = (ball_p[1] + BOUNDARY[0][1])//2
                    print("Resetting kick_goal",kick_goal)

                if (ball_p[1] < BOUNDARY[0][1] + ROB_RANG):
                    print("Rule 2")
                    kick_goal[1] = ball_p[1]
                    kick_goal[0] = ball_p[0] - ROB_RANG

            elif select == 'bottom':
                if(ball_p[1]) > block[3] and ball_p[0] < oppo_p[0][0]:
                    kick_goal[1] = (ball_p[1] + BOUNDARY[4][1])//2
                    print("Resetting kick_goal",kick_goal)

                if (ball_p[1] > BOUNDARY[6][1] + ROB_RANG):
                    print("Rule 2")
                    kick_goal[1] = ball_p[1]
                    kick_goal[0] = ball_p[0] - ROB_RANG
            dist_with_second_pt = math.hypot(oriplayer_p[0][0] - kick_goal[0], oriplayer_p[0][1] - kick_goal[1])
            # if (( 570 < ball_p[0] < 630  and 130 < ball_p[1] < 190) or ball_p[0] > kick_goal[0] or dist_with_second_pt < SECOND_ERROR_DIST) and goal == 1 :
            if (ball_p[0] > kick_goal[0] or dist_with_second_pt < SECOND_ERROR_DIST) and goal == 1 :
                print("Rule 3 enter")
                kick_goal[0] = BOUNDARY[1][0] # bdy20=870
                allow = [BOUNDARY[2][1], BOUNDARY[5][1]] \
                    if BOUNDARY[2][1] < BOUNDARY[5][1] else [BOUNDARY[5][1], BOUNDARY[2][1]]
                # if(robot1_kicked and kick_goal < 830):
                #     kick_goal[0] = 600 + countki

                print("blocl", block)
                kick_goal[1] = int((block[2] - block[1]) / 2 + block[1])
                # stage = 1
                goal = 2
            if goal == 2 and oriplayer_p[0][0] <= BOUNDARY[1][0] - 30:
                # arrivepoint(player_id,kick_goal)
                kick_goal[0] = BOUNDARY[1][0]
                arrivepoint(player_id, kick_goal)

            return [MOVE[kick_way]['Kick'], 'N1', 'N1']

        return ['N1', 'N1', 'N1']

    return ['N1', 'N1', 'N1']


def get_sent_cmd(sentcmd, update):
    """
    Description:
        Simulator will pass the received strategy and a sending state
    Parameter:
        param1: list[str] -> received command
        param2: bool -> sent or not
    """
    # Your code
    if update:
        print('sent: ', sentcmd[0][0])
        if sentcmd[0][0] == 'N' and stage == 4:
            print('===N recieved==')
            global kick_flag
            kick_flag = True
        if sentcmd[2][0] == 'j' or sentcmd[2][0] == 'h':
            global kicked
            print("DONE")
            kicked = True
        if sentcmd[0][0] == 'j' or sentcmd[0][0] == 'h':
            global robot1_kicked
            print("DONE")
            robot1_kicked = True
    pass


def _unit_vector(start, end):
    vector = [e - s for s, e in zip(start, end)]  # 應該是後減前
    length = math.hypot(vector[0], vector[1])
    uniVector = [comp / length for comp in vector]
    # print(vector,uniVector,length)
    return uniVector


def _dot(x, y):
    """Dot product as sum of list comprehension doing element-wise multiplication"""
    return sum(x_i * y_i for x_i, y_i in zip(x, y))


def _rotate(vector, angle):
    rot_vector = [0.0, 0.0]
    rot_vector[0] = (math.cos(angle) * vector[0]) - (math.sin(angle) * vector[1])
    rot_vector[1] = (math.sin(angle) * vector[0]) + (math.cos(angle) * vector[1])
    return rot_vector


def _angle(a, b):
    cross = a[0] * b[1] - a[1] * b[0]
    return math.asin(cross)
########################################################
def moverobot(player_id, kick_goal ):
    global kick_dir, first_arri,stage
    if stage == 1:
        print("@@@@@@@@@@@@@@@@@@@@@@@in stage 1@@@@@@@@@@@@@@@@@@@@@@@")
        # global kick_goal, kick_dir, first_arri
        # kick_goal = [635,370]
        first_arri = arrivepoint(player_id, kick_goal)
        kick_dir = _unit_vector(ball_p, kick_goal)
        # print('Goal(x y)', kick_goal[0], kick_goal[1])
        stage += 1
    elif stage == 2:
        #轉圈圈進到這裡
        print("@@@@@@@@@@@@@@@@@@@@@@@in stage 2@@@@@@@@@@@@@@@@@@@@@@@")
        action = findball(player_id, first_arri)
        if action == "next stage":
            stage = 3
            return ['N1', 'N1', 'N1']
        else:
            return action
    elif stage == 3:
        action = rotatedirection(player_id, kick_dir)
        if action == "next stage":
            stage = 4
            return ['N1', 'N1', 'N1']
        else:
            return action
    return ['N1', 'N1', 'N1']

#****
def findball(player_id, first_arri):
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]

    # 決定方向（L 或 R）
    move_dir = _unit_vector(player_p, first_arri)
    product = -1
    for way in ALLOW_MOVE_WAY:
        temp_product = _dot(move_dir, _rotate(player_d, WAY_ANGLE[way]))
        if temp_product > product:
            global move_way
            product = temp_product
            move_way = way

    dist = math.hypot(player_p[0] - first_arri[0], player_p[1] - first_arri[1])
    dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
    
    if dist_ball > SAFE_DIST + 8:
    #if dist >= ERROR_DISTANCE and dist_ball > SAFE_DIST + 8:
        # 【改】重新用現在球的位置算 move_dir
        move_dir = _unit_vector(player_p, ball_p)
        dirction = _rotate(player_d, WAY_ANGLE[move_way])
        angle = _angle(move_dir, dirction)

        print(f"[DEBUG] Angle to ball: {math.degrees(angle):.2f} degrees")

        rotate_flag = False

        if angle > 2 * ROTATE_ANGLE:
            Robot[player_id] = 'Q1'
            rotate_flag = True
        elif angle < -2 * ROTATE_ANGLE:
            Robot[player_id] = 'E1'
            rotate_flag = True
        elif angle > ROTATE_ANGLE:
            Robot[player_id] = 'q1'
            rotate_flag = True
        elif angle < -ROTATE_ANGLE:
            Robot[player_id] = 'e1'
            rotate_flag = True

        if rotate_flag:
            print("[INFO] Rotating to align to ball.")
            return [Robot[0], Robot[1], Robot[2]]

        if dist >= MOVE[move_way]['Bound'][0]:
            Robot[player_id] = MOVE[move_way]['Fast']
            print("[INFO] Move fast towards ball.")
            return [Robot[0], Robot[1], Robot[2]]
        elif dist >= MOVE[move_way]['Bound'][1]:
            Robot[player_id] = MOVE[move_way]['Norm']
            print("[INFO] Move slow towards ball.")
            return [Robot[0], Robot[1], Robot[2]]

    print("[INFO] Reached ball or very close. Next stage.")
    return "next stage"


# def findball(player_id,first_arri):
#     player_p = oriplayer_p[player_id]
#     player_d = oriplayer_d[player_id]
#     move_dir = _unit_vector(player_p, first_arri)
#     product = -1
#     for way in ALLOW_MOVE_WAY:  # 判斷轉左後較近還是轉右後較靠近
#         temp_product = _dot(move_dir, _rotate(player_d, WAY_ANGLE[way]))
#         print(way, ':', temp_product, move_dir, _rotate(player_d, WAY_ANGLE[way]))
#         if temp_product > product:
#             global move_way
#             product = temp_product
#             move_way = way
#     # print('move:', move_way)

#     move_dir = _unit_vector(player_p, first_arri)
#     dist = math.hypot(player_p[0] - first_arri[0], player_p[1] - first_arri[1])
#     dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
#     # print('arri-player-dist', first_arri, player_p, dist)
#     if dist >= ERROR_DISTANCE and dist_ball > SAFE_DIST + 8:  # ERROR dist 應是為了避免要去的地方太近，出現error，dist_ball應該是為了避免不小心碰到球
#         dirction = _rotate(player_d, WAY_ANGLE[move_way])

#         # print("move_direction", dirction)
#         # print("rotate", _rotate(player_d, WAY_ANGLE[move_way]))
#         angle = _angle(move_dir, dirction)
#         if angle > 0 and angle > 2 * ROTATE_ANGLE:
#             # print("Q")
#             Robot[player_id] = 'Q1'
#             print("###############################findball")
#             #一直原地轉圈是因為進到這裡來
#             return [Robot[0], Robot[1], Robot[2]]
#         elif angle < 0 and angle < -2 * ROTATE_ANGLE:
#             # print("E")
#             Robot[player_id] = 'E1'
#             return [Robot[0], Robot[1], Robot[2]]
#         elif angle > 0 and angle > ROTATE_ANGLE:
#             # print("Q1")
#             Robot[player_id] = 'q1'
#             return [Robot[0], Robot[1], Robot[2]]
#         elif angle < 0 and angle < -ROTATE_ANGLE:
#             # print("E1")
#             Robot[player_id] = 'e1'
#             return [Robot[0], Robot[1], Robot[2]]
#         '''
#         MOVE
#         '''
#         # MOVE = {
#         #     'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [18, 10]},
#         #     'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [80, 8]},
#         #     'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [80, 8]},
#         #     'B': {'Norm': 's1', 'Bound': [None, 10]}
#         # }
#         if dist >= MOVE[move_way]['Bound'][0]:  # 超過80的話走快點
#             # print("F")
#             # return ['N1', 'N1', 'N1']
#             Robot[player_id] = MOVE[move_way]['Fast']
#             return [Robot[0], Robot[1], Robot[2]]
#             # return [MOVE[move_way]['Fast'], 'N1', 'N1']
#         elif dist >= MOVE[move_way]['Bound'][1]:
#             # print("S")
#             # return ['N1', 'N1', 'N1']
#             Robot[player_id] = MOVE[move_way]['Norm']
#             return [Robot[0], Robot[1], Robot[2]]
#             # return [MOVE[move_way]['Norm'], 'N1', 'N1']
#     # print('dist, dist_b:', int(dist), int(dist_ball))
#     # print('arri-player-dist', first_arri, player_p)
#     # print('========================================')
#     # 如果可以順利跑到這裡則就可以脫離轉圈圈，順利跨一步
#     return "next stage"

def arrivepoint(player_id,kick_goal):
    global first_arri
    final_first_arri = [0,0]
    ball_pnext=[0,0]
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    kick_dir = _unit_vector(ball_p, kick_goal)
    first_arri = [int(ball_p[i] - int(kick_dir[i] * ROUGH_RANG)) for i in range(2)]
    if(first_arri[1]>=440-25):#446是邊界，這個是保護機制，目前沒作用
        ball_pnext[0] = ball_p[0]+15
        ball_pnext[1] = ball_p[1]
        kick_dir = _unit_vector(ball_p, (ball_pnext))
        final_first_arri = [int(ball_p[i] - int(kick_dir[i] * ROUGH_RANG)) for i in range(2)]
    else:
        final_first_arri = first_arri
    global kick_point
    kick_point = [int(ball_p[i] - kick_dir[i] * SAFE_DIST) for i in range(2)]
    # move_dir = _unit_vector(player_p, final_first_arri)
    # print("pt",final_first_arri)
    return final_first_arri

def rotatedirection(player_id,kick_dir):
    # Choose the way of kick
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    global kick_way
    global move_way
    # return ['N1', 'N1', 'N1']
    if kick_way == "":
        # print('S3 arri-player-dist', first_arri, player_p)
        product = -1
        for way in ALLOW_MOVE_WAY:
            # print("rotate",_rotate(player_d, WAY_ANGLE[way]))
            temp_product = _dot(kick_dir, _rotate(player_d, WAY_ANGLE[way]))
            # print(way, ':', temp_product, kick_dir, _rotate(player_d, WAY_ANGLE[way]))
            if temp_product > product:
                product = temp_product
                kick_way = way
        # print("kick way", kick_way)
    dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
    dirction = _rotate(player_d, WAY_ANGLE[kick_way])
    # print("dirction", dirction)
    angle = _angle(kick_dir, dirction)
    # print('dist_b:', dist_ball)
    # print('ang:', angle * 180 / math.pi)
    if angle > 0 and angle > 2 * ROTATE_ANGLE:
        # print('turn big left')
        Robot[player_id] = 'Q1'
        return [Robot[0], Robot[1], Robot[2]]
        #print("###############################rotatedirection")
        # return ['Q1', 'N1', 'N1']
    elif angle < 0 and angle < -2 * ROTATE_ANGLE:
        # print('turn big  right')
        Robot[player_id] = 'E1'
        return [Robot[0], Robot[1], Robot[2]]
        # return ['E1', 'N1', 'N1']
    elif angle > 0 and angle > ROTATE_ANGLE:
        # print('turn left')
        Robot[player_id] = 'q1'
        return [Robot[0], Robot[1], Robot[2]]
        # return ['q1', 'N1', 'N1']
    elif angle < 0 and angle < -ROTATE_ANGLE:
        # print('turn right')
        Robot[player_id] = 'e1'
        return [Robot[0], Robot[1], Robot[2]]


    WAYS = ['F', 'R', 'B', 'L']
    for i in [1, 2, 3, 0]:
        move_way = WAYS[(WAYS.index(kick_way)+i) % 4]
        # print("kick_way",kick_way," ",WAYS.index(kick_way),"i==",i,move_way)
        temp_dir = _rotate(dirction, math.pi/2*i)
        diff_vec = [k - p for k, p in zip(kick_point, player_p)]
        product = _dot(temp_dir, diff_vec)
        print(move_way, ':', product)

        if product > MOVE[move_way]['Bound'][1]:
            # print('move:', MOVE[move_way]['Norm'])
            Robot[player_id] = MOVE[move_way]['Norm']
            return [Robot[0], Robot[1], Robot[2]]


    return "next stage"