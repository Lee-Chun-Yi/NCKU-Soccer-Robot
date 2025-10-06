import math
import cv2
from time import sleep

# Const
# ROB_RANG = 35//2
ROB_RANG = 15

KICKABLE_RANGE = 20//2
# ROUGH_RANG = 50//2
ROUGH_RANG = 30

# ERROR_DISTANCE = 10//2
ERROR_DISTANCE = 12
# ROTATE_ANGLE = 0.18
ROTATE_ANGLE = 0.22
# SAFE_DIST = 43//4  # RoboRad/2 + ball radius
# SAFE_DIST = 16  # RoboRad/2 + ball radius #### SOUND GOOD
SAFE_DIST = 13 + 2.5  # RoboRad/2 + ball radius

WAY_ANGLE = {'F': 0, 'L': -math.pi / 2, 'R': math.pi / 2}
MOVE = {
    'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [6, 3]},
    'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [35, 5]},
    'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [35, 7]},
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
player_p = [0, 0]
player_d = [0.0, 0.0]
oppo_id = 3
oppo_p = [0, 0]

# Anticipate point

stage = 1
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
# BOUNDARY = [(20, 0), (360, 0), (360, 50), (380, 50), (380, 130), (360, 130), (360, 180), (20, 180),(20,130), (0, 130),
#             (0, 50), (20, 50)]
BOUNDARY = [(16, 0), (370, 0), (370, 65), (387, 65), (387, 186), (370, 186), (370, 250),(16,250), (16, 186),
            (0, 186), (0, 65),(16,65)]
CENTER = [0, 0]
FRAME_WIDTH, FRAME_HEIGHT = 1920 / 2, 1280 / 2
OFFSET = [0, 0]  # Offset is to calibrate the total field position
SCALE = 1

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
        BOUNDARY.append([pos[0], pos[1]])
    # print(BOUNDARY)
    pass


def Initialize():
    """
    Description:
        Initialuze the strategy.
        This function will be called by the simulator before a simulation is started.
    """
    # Your code
    print("Your Player is Robot", player_id)
    # print("B21", BOUNDARY[2][1], " B51", BOUNDARY[5][1])
    print("Obstacle is Robot", oppo_id)
    print(BOUNDARY)
    print("B21", BOUNDARY[2][1], " B51", BOUNDARY[5][1])
    print(oppo_p[0])
    global stage
    stage = 1
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
    global kick_goal
    # Your code
    if stage > 1:
        cv2.circle(frame, (int(first_arri[0]), int(first_arri[1])), 3, (237, 183, 217), -1)
        cv2.line(frame, (ball_p[0], ball_p[1]), (kick_goal[0], kick_goal[1]), (0, 0, 255), 2)
        cv2.circle(frame, (int(player_p[0]), int(player_p[1])), 3, (0, 0, 255), -1)
    if stage > 2:
        cv2.circle(frame, (int(kick_point[0]), int(kick_point[1])), 3, (232, 122, 63), -1)
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
    global player_d, player_p, player_id, oppo_id, oppo_p, ball_p
    player_d = teamD[0]
    player_p = teamP[0]
    oppo_p = oppoP[oppo_id - 3]  # 為什麼‘-3？’ 為了歸0？(以得出list中的第一個的值)因為一開始opp_id是3
    print("o",oppo_p)
    print("D",teamD,"p",teamP,"OP", oppoP,"BP",ballP)

    ball_p = ballP

    pass
check = 0

def strategy():
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """
    global stage
    global kick_goal
    global check
    # check = 0
    print("===================",oppo_p[1])
    # Your code
    # ball_p = [246,124]
    if stage == 1:
        # Find position of goal
        # print
        kick_goal[0] = BOUNDARY[2][0]#bdy20=870
        allow = [BOUNDARY[2][1], BOUNDARY[5][1]] \
            if BOUNDARY[2][1] < BOUNDARY[5][1] else [BOUNDARY[5][1], BOUNDARY[2][1]]
        print("allow=",allow)
        block = []
        block.append(oppo_p[1] - ROB_RANG)#oppo_p[1]是robo0的y
        block.append(oppo_p[1] + ROB_RANG)  # ROB_RANG的意思？
        print("block=",block)

        if block[0] > allow[0]:#allow[0]=170 [1]=350(170,350為goal的框)
            if block[1] < allow[-1]:
                allow.insert(1, block[0])  # 在allow的allow[1]、2的位置插入（原先位置的向右邊移動）這兩個
                allow.insert(2, block[1])
                print("allow",allow)
            else:
                allow[1] = block[0]  # 直接替換掉allow[1]欄位
                print("allow",allow)
        else:
            allow[0] = block[1]
            print("allow",allow)
        dist = None
        for i in range(0, len(allow), 2):
            if allow[i + 1] - allow[i] > KICKABLE_RANGE:
                kick_goal_temp = int((allow[i] + allow[i + 1]) / 2)
                if dist is None or dist > abs(ball_p[1] - kick_goal_temp):#none代表不含任何值
                    dist = abs(ball_p[1] - kick_goal_temp)
                    kick_goal[1] = kick_goal_temp
        # print("allow",allow,kick_goal_temp)
        print("player",player_p,"player_d",player_d,"oppop",oppo_p,"ball",ball_p)
        print('Goal(x y)', kick_goal[0], kick_goal[1])
        global kick_dir
        kick_dir = _unit_vector(ball_p, kick_goal)
        print(kick_dir)
        global first_arri, kick_point
        first_arri = [int(ball_p[i] - kick_dir[i] * ROUGH_RANG) for i in range(2)]
        kick_point = [int(ball_p[i] - kick_dir[i] * SAFE_DIST) for i in range(2)]
        # Find the way of move
        move_dir = _unit_vector(player_p, first_arri)
        product = -1
        for way in ALLOW_MOVE_WAY:
            temp_product = _dot(move_dir, _rotate(player_d, WAY_ANGLE[way]))
            print(way, ':', temp_product, move_dir, _rotate(player_d, WAY_ANGLE[way]))
            if temp_product > product:
                global move_way
                product = temp_product
                move_way = way
        print('move:', move_way)
        stage += 1
        print('stage:', stage)
    elif stage == 2:
        print('2nd: stage')
        move_dir = _unit_vector(player_p, first_arri)
        dist = math.hypot(player_p[0] - first_arri[0], player_p[1] - first_arri[1])
        dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
        print('arri-player-dist', first_arri, player_p, dist)
        if dist >= ERROR_DISTANCE and dist_ball > SAFE_DIST + 5 :
            dirction = _rotate(player_d, WAY_ANGLE[move_way])
            angle = _angle(move_dir, dirction)
            if angle > 0 and angle > 3 * ROTATE_ANGLE:
                return ['Q1', 'N1', 'N1']
            elif angle < 0 and angle < -3 * ROTATE_ANGLE:
                return ['E1', 'N1', 'N1']
            elif angle > 0 and angle >1.5* ROTATE_ANGLE:
                return ['q1', 'N1', 'N1']
            elif angle < 0 and angle < 1.5* -ROTATE_ANGLE:
                return ['e1', 'N1', 'N1']
            '''
            MOVE
            '''
            if dist >= MOVE[move_way]['Bound'][0]:
                return [MOVE[move_way]['Fast'], 'N1', 'N1']
            elif dist >= MOVE[move_way]['Bound'][1]:
                return [MOVE[move_way]['Norm'], 'N1', 'N1']
        print('dist, dist_b:', int(dist), int(dist_ball))
        print('arri-player-dist', first_arri, player_p)
        print('========================================')
        stage += 1
        print('stage:', stage)
    elif stage == 3:
        print('3rd: stage')
        global kick_way
        # Choose the way of kick
        if kick_way == "":
            print('S3 arri-player-dist', first_arri, player_p)
            product = -1
            for way in ALLOW_MOVE_WAY:
                temp_product = _dot(kick_dir, _rotate(player_d, WAY_ANGLE[way]))
                print(way, ':', temp_product, kick_dir, _rotate(player_d, WAY_ANGLE[way]))
                if temp_product > product:
                    # global kick_way
                    product = temp_product
                    kick_way = way
            print("kick way", kick_way)
        dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
        dirction = _rotate(player_d, WAY_ANGLE[kick_way])
        angle = _angle(kick_dir, dirction)
        print('dist_b:', dist_ball)
        print('ang:', angle * 180 / math.pi)
        if angle > 0 and angle > 2.5 * ROTATE_ANGLE:
            print('turn big left')
            return ['Q1', 'N1', 'N1']
        elif angle < 0 and angle < -2.5 * ROTATE_ANGLE:
            print('turn big  right')
            return ['E1', 'N1', 'N1']
        elif angle > 0 and angle > 0.5* ROTATE_ANGLE:
            print('turn left')
            return ['q1', 'N1', 'N1']
        elif angle < 0 and angle <0.5* -ROTATE_ANGLE:
            print('turn right')
            return ['e1', 'N1', 'N1']
        WAYS = ['F', 'R', 'B', 'L']
        for i in [1, 2, 3, 0]:
            move_way = WAYS[(WAYS.index(kick_way) + i) % 4]
            temp_dir = _rotate(dirction, math.pi / 2 * i)
            diff_vec = [k - p for k, p in zip(kick_point, player_p)]
            product = _dot(temp_dir, diff_vec)
            print(move_way, ':', product)
            if product > MOVE[move_way]['Bound'][1]:
                print('move:', MOVE[move_way]['Norm'])
                return [MOVE[move_way]['Norm'], 'N1', 'N1']
        if check == 3:
            stage += 1
        else:
            sleep(1)    
            check += 1
    dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
    print('player, kick ball', player_p, kick_point, dist_ball)
    if kick_flag: #and not (kicked)
        print('===kicked===', MOVE[kick_way]['Kick'])
        #sleep(1)  # For check
        return [MOVE[kick_way]['Kick'], 'N1', 'N1']
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
        if sentcmd[0][0] == 'j' or sentcmd[0][0] == 'h':
            global kicked
            sleep(0.5)
            kicked = True
    pass


def _unit_vector(start, end):
    vector = [e - s for s, e in zip(start, end)]
    length = math.hypot(vector[0], vector[1])
    uniVector = [comp / length for comp in vector]
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
