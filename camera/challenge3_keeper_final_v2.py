import math
import cv2
import random
from time import sleep

# Const
ROB_RANG = 30
KICKABLE_RANGE = 40
ROUGH_RANG = 50
ERROR_DISTANCE = 10
ROTATE_ANGLE = 0.30
SAFE_DIST = 43  # RoboRad/2 + ball radius
WAY_ANGLE = {'F': 0, 'L': -math.pi / 2, 'R': math.pi / 2}
Robot = ['N1','N1','N1']
MOVE = {
    'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [18, 10]},
    'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [80, 8]},
    'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [80, 8]},
    'B': {'Norm': 's1', 'Bound': [None, 10]}
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
player_id = 1
oriplayer_p = [[0, 0],[0,0],[0,0]]
oriplayer_d = [[0, 0],[0,0],[0,0]]
oppo_id = 3
oppo_p = [[0, 0],[0,0],[0,0]]
kick_dir = [0.0, 0.0]

kick_way = ""
move_way = ""

#robot_kick

keeper_step = 1
keeper_kicked = False
width = 40
long = 60
keeper_mode = 1
BOUNDARY_goal = 110
Goal_area =[0,160,350,175,345]

#region clac
target = [[380,150],[380,250],[380,350]]
recordtarget = [0,0,0]
kick_target = [0,0]

#exception handle####原為嘗試解決一些例外情況，故暫留
#
# cannotmove = 0
# pst_lasttime = [0,0]

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
    global Robot,keeper_step,keeper_mode
    global cannotmove,pst_lasttime
    Robot = ['N1','N1','N1']

    with open('strategy.txt', 'a') as f:
        f.write('\n')
        f.write('New strategy(pos3 3): ')

    #robot_kick
    keeper_step = 1
    keeper_mode = 1


    #handle exception####原為嘗試解決一些例外情況，故暫留
    # global cannotmove, pst_lasttime
    # cannotmove = 0
    # pst_lasttime = [0,0]
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
    # global goal
    # # Your code
    # # cv2.circle(frame, (int(goal[0]), int(goal[1])), 3, (237, 183, 217), -1)
    # if stage > 1:
    #     cv2.circle(frame, (int(first_arri[0]), int(first_arri[1])), 3, (237, 183, 217), -1)
    #     # cv2.line(frame, (ball_p[0], ball_p[1]), (kick_goal[0], kick_goal[1]), (0, 0, 255), 2)
    #     # cv2.circle(frame, (int(player_p[0]), int(player_p[1])), 3, (0, 0, 255), -1)
    # if stage > 2:
    #     cv2.circle(frame, (int(kick_point[0]), int(kick_point[1])), 3, (232, 122, 63), -1)
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
    global oriplayer_d, oriplayer_p, player_id, oppo_id, ball_p
    # i = 0
    for i in range(3):
        oriplayer_d[i] = teamD[i]
        oriplayer_p[i] = teamP[i]
    oppo_p[0] = oppoP[oppo_id - 3]  # 為什麼‘-3？’ 為了歸0？(以得出list中的第一個的值)因為一開始opp_id是3
    oppo_p[1] = oppoP[oppo_id - 3 + 1]
    oppo_p[2] = oppoP[oppo_id - 3 + 2]
    ball_p = ballP

    pass
####原為嘗試解決一些例外情況，故暫留
# def exception(player_id):
#     player_p = oriplayer_p[player_id]
#     player_d = oriplayer_d[player_id]
#
#
#     if abs(pst_lasttime[0] - player_p[0]) <= 3 and abs(pst_lasttime[1] - player_p[1]) <= 3:
#         cannotmove += 1
#
#     if cannotmove == 20:
#         print("try to escape")
#         return go_back_origin(player_id)
#
# def go_back_origin(player_id):
#     global cannotmove
#
#     cannotmove = 0
#     player_p = oriplayer_p[player_id]
#     if player_p[0] >= 180 + 10 or player_p[0] <= 180 - 10:
#         if player_p[0] >= 180:
#             Robot[player_id] = 's1'
#             return [Robot[0], Robot[1], Robot[2]]
#         elif player_p[0] < 180:
#             Robot[player_id] = 'w1'
#             return [Robot[0], Robot[1], Robot[2]]
#     if player_p[0] >= 260 + 50  or player_p[0] <= 260 - 50 :
#         if player_p[1] <= 260 - 40:
#             Robot[player_id] = 'd1'
#             return [Robot[0], Robot[1], Robot[2]]
#
#         elif player_p[1] > 260 + 40:
#             Robot[player_id] = 'a1'
#             return [Robot[0], Robot[1], Robot[2]]
#
#     return ['N1','N1','N1']

def strategy():
    # global goal
    global keeper_step,keeper_kicked,keeper_mode
    # global cannotmove,pst_lasttime####原為嘗試解決一些例外情況，故暫留
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """
    player_id = 1
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]

    ####原為嘗試解決一些例外情況，故暫留
    # if abs(ball_p[0] - player_p[0]) >= 40 and abs(ball_p[1] - player_p[1]) >= 40:
    #     if abs(pst_lasttime[0] - player_p[0]) <= 3 and abs(pst_lasttime[1] - player_p[1]) <= 3:
    #         cannotmove += 1
    #
    #     if cannotmove == 20:
    #         print("try to escape")
    #         return go_back_origin(player_id)

    # pst_lasttime = player_p
    #####################################################

    if ((player_p[1] - 10 < ball_p[1] <player_p[1] + 10) and abs(ball_p[0]-oriplayer_p[1][0]) <= 45) or keeper_mode == 2:#10和45是測試後得出的數值，為了確保在機器人中間後，在進入踢球模式
        keeper_mode = 2
        if abs(ball_p[1]-player_p[1]) >= long + round(15/2) +30 or abs(ball_p[0]-player_p[0]) >= width + round(15/2) + 30:#如果在踢的過程中出現球突然離遠的情況，就進入這裡

            global keeper_step
            keeper_step = 4
            print("too far")
            return keeper_kick(player_id)
        print("Kick mode")
        return keeper_kick(player_id)
    elif keeper_mode == 1 :

        print("keeper", keeper_step)
        if abs(player_p[0] - 185)>=10 and (ball_p[0] - player_p[0]) >= 50 :#如敵人在goal area糾纏就先不回歸原位
            print("Return to original line")
            if player_p[0] >= 180:
                Robot[player_id] = 's1'
                return [Robot[0], Robot[1], Robot[2]]
            elif player_p[0] < 180:
                Robot[player_id] = 'w1'
                return [Robot[0], Robot[1], Robot[2]]

        elif (ball_p[0]-player_p[0]) < width/2 + round(15/2):#球離太近
            Robot[player_id] = 's1'
            return [Robot[0], Robot[1], Robot[2]]

        else:
            print("Move mode")
            return keeper_movement(player_id)

    return ['N1','N1','N1']


def keeper_movement(player_id):
    player_p = oriplayer_p[player_id]
    plater_d = oriplayer_d[player_id]
    enemy = [0, 0]

    distwithkeeper = 9999999
    for i in range(3):
        #找出離龍門最靠近的敵人與其座標:enemy
        tempdistwithkeeper = math.hypot(abs(oppo_p[i][0] - 180),abs(oppo_p[i][1] - 260))  # 827,259/180,260是goal area外那條線的中心
        print(i, tempdistwithkeeper)
        if tempdistwithkeeper < distwithkeeper:
            enemy = oppo_p[i]
            distwithkeeper = tempdistwithkeeper
            print("Change!", enemy)
    #找出球和敵人之間的角度
    dist = [0, 0]
    dist = [abs(enemy[0] - ball_p[0]), abs(enemy[1] - ball_p[1])]
    angle = math.atan(dist[1] / dist[0])

    distkeeper = [0, 0]
    #用三角函數找出goal（用找出的角度作延伸(相似三角形))
    distkeeper[0] = abs(enemy[0] - player_p[0])
    distkeeper[1] = math.tan(angle) * distkeeper[0]
    if ball_p[1] - enemy[1] >= 0:
        goal = [distkeeper[0], distkeeper[1] + enemy[1]]
    elif ball_p[1] - enemy[1] < 0:
        goal = [distkeeper[0], enemy[1] - distkeeper[1]]
    print(oppo_p, "enemy", enemy, "ballp", ball_p, "goal", goal, distkeeper)


    if ball_p[0] < 500 and Goal_area [1] - 10 < ball_p[1] < Goal_area[2] +10 :#500中點,-10 +10是為了稍微擴大範圍,Goal_area[1],[2]為龍門的y-axis範圍
        goal[1] = ball_p[1]


    if goal[1] - 10 <= player_p[1] <= goal[1] + 10 and Goal_area [1] - 10  <= goal[1] <= Goal_area[2] +10:#如守門員在目標附近 且 該目標在龍門範圍內
        return ['N1', 'N1', 'N1']
    #根據目標和守門員的相對位置來判別
    elif goal[1] >= player_p[1] and Goal_area [1] - 10  <= goal[1] <= Goal_area[2] +10:

        print("It is time to defense，RIGHT")
        Robot[player_id] = 'd1'
        if abs(goal[1] - player_p[1]) >= 80:
            Robot[player_id] = 'D1'
        return [Robot[0], Robot[1], Robot[2]]

    elif goal[1] < player_p[1] and Goal_area [1] - 10  <= goal[1] <= Goal_area[2] +10:
        print("It is time to defense,LEFT")
        Robot[player_id] = 'a1'
        if abs(goal[1] - player_p[1]) >= 80:
            Robot[player_id] = 'A1'

        return [Robot[0], Robot[1], Robot[2]]

    else:#如果沒事情做就歸位到260附近
        if player_p[1] <= 260 - 40:
            Robot[player_id] = 'd1'
            return [Robot[0], Robot[1], Robot[2]]

        elif player_p[1] > 260 + 40:
            Robot[player_id] = 'a1'
            return [Robot[0], Robot[1], Robot[2]]

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
        # if sentcmd[0][0] == 'N' and stage == 4:
        #     print('===N recieved==')
        #     global kick_flag
        #     kick_flag = True
        # if sentcmd[2][0] == 'j' or sentcmd[2][0] == 'h':
        #     global kicked
        #     print("DONE")
        #     kicked = True
        # if sentcmd[0][0] == 'j' or sentcmd[0][0] == 'h':
        #     global robot1_kicked
        #     print("DONE")
        #     robot1_kicked = True
        #讓程式知道球踢出去了
        if sentcmd[1][0] == 'i' or sentcmd[1][0] == 'u':
            global keeper_kicked
            print("GOalKeeper DONE")

            keeper_kicked = True
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



def keeper_kick(player_id):
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    global kick_way,keeper_step,kick_dir,Robot,keeper_kicked,keeper_mode,BOUNDARY_goal
    # global move_way
    global kick_target

    kick_target = oriplayer_p[0]#測試用
    kick_target = robot_target()
    print("target==",kick_target)
    boundary = robot_boundary(player_id, kick_target)
    if keeper_step == 1:#旋轉

        kick_dir = _unit_vector(ball_p, kick_target)
        dirction = player_d
        # print("dirction", dirction)
        angle = _angle(kick_dir, dirction)

        print("stage", keeper_step)
        # 如果將要旋轉的角度會超過110（邊界)
        for i in range(len(boundary)):

            if boundary[i][0]  < BOUNDARY_goal: #如果在右邊就是大於那裡的BOUNDARY_goal

                if player_p[1] <= ball_p[1]:
                    kick_way = 'R'
                else:
                    kick_way = 'L'
                keeper_step = 3 #直接跳過轉角度步驟，只往前踢

                return [Robot[0], Robot[1], Robot[2]]
        #如果旋轉後任意一點(四個頂點)和球的距離太小，就後退
        for i in range (len(boundary)):
            if abs(ball_p[0] - boundary[i][0]) <  10:
                Robot[player_id] = 's1'
                print('go back',abs(ball_p[0] - boundary[i][0]),boundary[i][0],ball_p[0])
                print(player_id)
                return [Robot[0], Robot[1], Robot[2]]

        if angle > 0 and angle > 2 * ROTATE_ANGLE:
            # print('turn big left')
            Robot[player_id] = 'Q1'
            return [Robot[0], Robot[1], Robot[2]]
            # return ['Q1', 'N1', 'N1']
        elif angle < 0 and angle < -2 * ROTATE_ANGLE:
            # print('turn big  right')
            Robot[player_id] = 'E1'
            # print(player_id)
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
        # move_step = 0
        keeper_step = 2
        print("keeper",keeper_step)
    if keeper_step == 2 :#將守門員移動到適合踢的位置

        print("stage", keeper_step)
        boundary.append(player_p)
        print("BOUNDARY NOW ",boundary)
        #計算相對位置以知道向左向右
        if kick_target[1] >= oriplayer_p[1][1]:
            print("move left,kick right")

            kick_way = 'R'
            kick_region = (boundary[3][1] + boundary[4][1])//2#右下和中點的中間
            if abs(ball_p[1] - kick_region) < 15:#如果在適合踢的區域內
            # if abs(ball_p[1] - kick_region) < 15 or (abs(boundary[1][1] - Goal_area[3])<5 or abs(boundary[3][1] - Goal_area[4])<5) :#or後面那項是為了讓它不會一直撞邊界
                Robot[player_id] = 'N1'
                print('alr can kick', kick_region, ball_p)
                # print(player_id)
                keeper_step = 3
                # print(Robot[0], Robot[1], Robot[2])
                return [Robot[0], Robot[1], Robot[2]]
            else:
            # kick_way = 'R'
                Robot[player_id] = 'a1'
                # print('A1')
                # print(player_id)
                print('Adjusting position', kick_region, ball_p)
                # print(Robot[0],Robot[1],Robot[2])
                return [Robot[0], Robot[1], Robot[2]]

        elif kick_target[1] < oriplayer_p[1][1] :

            print("move right,kick left")

            kick_way = 'L'
            kick_region = (boundary[1][1] + boundary[4][1]) // 2
            if abs(ball_p[1] - kick_region) < 15 :
                Robot[player_id] = 'N1'
                keeper_step = 3
                print('alr can kick',kick_region,ball_p)
                # print(player_id)
                # print(Robot[0], Robot[1], Robot[2])
                return [Robot[0], Robot[1], Robot[2]]
            else:
                Robot[player_id] = 'd1'
                # print('D1')
                print('Adjusting position', kick_region, ball_p)
                # print(player_id)
                return [Robot[0], Robot[1], Robot[2]]


    if keeper_step == 3:#踢球
        print("stage",keeper_step)
        if   kick_way == 'L' and keeper_kicked == False:
            Robot[player_id] = 'u1'
            # print("U")
            return [Robot[0], Robot[1], Robot[2]]
        elif kick_way == 'R' and keeper_kicked == False:
            Robot[player_id] = 'i1'
            # print("I")
            return [Robot[0], Robot[1], Robot[2]]
        if keeper_kicked:
            keeper_step = 4
            print("Kick alr")
            keeper_kicked = False

            return['N1','N1','N1']
    if keeper_step == 4:#轉回原角度

        dirction = player_d
        # print("dirction", dirction)
        angle = _angle([1,0], dirction)

        print("stage of return", keeper_step)

        if angle > 0 and angle > 2 * ROTATE_ANGLE:
            # print('turn big left')
            Robot[player_id] = 'Q1'
            return [Robot[0], Robot[1], Robot[2]]

        elif angle < 0 and angle < -2 * ROTATE_ANGLE:
            # print('turn big  right')
            Robot[player_id] = 'E1'

            return [Robot[0], Robot[1], Robot[2]]

        elif angle > 0 and angle > ROTATE_ANGLE:
            # print('turn left')
            Robot[player_id] = 'q1'
            return [Robot[0], Robot[1], Robot[2]]

        elif angle < 0 and angle < -ROTATE_ANGLE:
            # print('turn right')
            Robot[player_id] = 'e1'
            return [Robot[0], Robot[1], Robot[2]]

        #轉完後回歸原狀
        keeper_step = 1
        keeper_mode = 1
        # print("keeper", keeper_step)

    return ['N1','N1','N1']



def robot_boundary(player_id,goal):
    global long,width
    global kick_dir
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_p[player_id]
    origin = [0, 0] #機器人原點座標
    # print("newpst", origin)
    anglex = -_angleforpoint(oriplayer_p[1], goal)  # 需要的角度是逆時針的，所以加個負號
    # anglex = angle

    blk2 = [[0, 0], [0, 0], [0, 0], [0, 0]]
    blk2[0][0] = origin[0] - width / 2
    blk2[0][1] = origin[1] + long / 2
    blk2[1][0] = origin[0] + width / 2
    blk2[1][1] = origin[1] + long / 2
    blk2[2][0] = origin[0] - width / 2
    blk2[2][1] = origin[1] - long / 2
    blk2[3][0] = origin[0] + width / 2
    blk2[3][1] = origin[1] - long / 2
    #以機器人原點進行計算
    for i in range(len(blk2)):
        blk2[i] = _rotate(blk2[i], anglex)
    print("blk2final with robot as origin", blk2)
    #換回原坐標
    #X
    for i in range(len(blk2)):
        blk2[i][0] = round(blk2[i][0] + player_p[0])
    #Y
    for i in range(len(blk2)):
        blk2[i][1] = round(abs(0 - player_p[1] + blk2[i][1]))
    print('ang:', anglex * 180 / math.pi)
    print("blk2after change to original",blk2)
    print('ang:', anglex * 180 / math.pi)
    return blk2

def _angleforpoint(a,b):

    angle = math.atan2(b[1]-a[1],b[0]-a[0])
    return angle

def robot_target():
    global target
    recordtarget = [0,0,0]
    #分為三個區域，區域內每多一個敵方機器人就+1
    for m in range(3):
        if oppo_p[m][0] < 500:
            if 70 < oppo_p[m][1] <= (target[0][1] + target[1][1]) // 2:
                recordtarget[0] += 1

            elif (target[0][1] + target[1][1]) // 2 < oppo_p[m][1] <= (target[1][1] + target[2][1]) // 2:
                recordtarget[1] += 1

            elif (target[1][1] + target[2][1]) // 2 < oppo_p[m][1] <= 450:
                recordtarget[2] += 1

    #找出最少機器人的區域
    min_enemy = target[recordtarget.index(min(recordtarget))]

    print("region", recordtarget.index(min(recordtarget)),"num of enemy", min(recordtarget))

    return min_enemy