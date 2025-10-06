import math
import cv2
import random
from time import sleep

# Const
ROB_RANG = 15
KICKABLE_RANGE = 20//2
ROUGH_RANG = 30
ERROR_DISTANCE = 8
ROTATE_ANGLE = 0.3
SAFE_DIST = 20  # RoboRad/2 + ball radius
WAY_ANGLE = {'F': 0, 'L': -math.pi / 2, 'R': math.pi / 2}
Robot = ['N1','N1','N1']
# MOVE = {
#     'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [18, 10]},
#     'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [80, 8]},
#     'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [80, 8]},
#     'B': {'Norm': 's1', 'Bound': [None, 10]}
# }
MOVE = {
    'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [6, 3]},
    'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [25, 5]},
    'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [25, 7]},
    'B': {'Norm': 's1', 'Bound': [None, 2]}
}
# Con
# Condition
ALLOW_MOVE_WAY = ['L', 'R']
# Field Variable
BOUNDARY = []  # [(x, y), ...]
CENTER = (0, 0)
# ori_center = []
ga_x = ga_y = 0

# Ball, Robot Parameter

ball_p = [0, 0]
player_id = 2
oriplayer_p = [[0, 0],[0,0],[0,0]]
oriplayer_d = [[0, 0],[0,0],[0,0]]
oppo_id = 3
oppo_p = [[0, 0],[0,0],[0,0]]
# Anticipate point

stage = 0
kick_goal = [650,350]
kick_dir = [0.0, 0.0]
first_arri = [0, 0]
kick_point = [0, 0]
# kick_p = [0, 0]
# kick_d = [0.0, 0.0]
kick_way = ""
move_way = ""
move_dir = [0.0, 0.0]
# toward_dir = [0.0, 0.0]
kick_flag = False
kicked = False
robot1_kicked =False
stopit = 0
kick_goal_list = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
block= [0,0,0,0]
runaway = 1

#boundary
# width = 40 + 10
# long = 60  + 20
width = 12
long = 20

oriteam_boundary = [0,0,0]
oriteam_boundary[0] = [[0, 0], [0, 0], [0, 0], [0, 0]]
oriteam_boundary[1] = [[0, 0], [0, 0], [0, 0], [0, 0]]
oriteam_boundary[2] = [[0, 0], [0, 0], [0, 0], [0, 0]]

orienemy_boundary = [0,0,0]
orienemy_boundary[0] = [[0, 0], [0, 0], [0, 0], [0, 0]]
orienemy_boundary[1] = [[0, 0], [0, 0], [0, 0], [0, 0]]
orienemy_boundary[2] = [[0, 0], [0, 0], [0, 0], [0, 0]]

team_boundary = [0,0,0]
team_boundary[0] = [[0, 0], [0, 0], [0, 0], [0, 0]]
team_boundary[1] = [[0, 0], [0, 0], [0, 0], [0, 0]]
team_boundary[2] = [[0, 0], [0, 0], [0, 0], [0, 0]]

blk2 = [[0, 0], [0, 0], [0, 0], [0, 0]]
blk3 = [[0, 0], [0, 0], [0, 0], [0, 0]]
angle = 0
anglenew = 0
directionold = [[0, 0], [0, 0], [0, 0]]

####detect collide

not_touchable_region =[]
control_display = 1

###final control
cha1_stage = 1
x_nextpoint_index = 50 #原45//120
y_target_index = 20 #原30//45
x_target_index = 36 #原60//80
SAFE_DIST_robot0_1 = 14#40
SAFE_DIST_robot0_2 = 8#25
runaway_index = 3
kick_goal_first = [650,350]#650,350




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
    global stage,Robot,runaway,block
    # cha1_stage = 1
    stage = 1
    # job = 1
    Robot = ['N1','N1','N1']
    # global kick_goal,firsttime
    # global goal,shown
    # shown = 0
    # goal = 1
    runaway = 1
    # kick_goal = [650, 350]
    # firsttime = 1
    # motion = 1
    block = [0,0,0,0]
    # output stratgy
    with open('strategy.txt', 'a') as f:
        f.write('\n')
        f.write('New strategy(pos3 3): ')



    origin = [0,0]
    for i in range(3):
        directionold[i] = [0,0]

    for i in range(len(oriteam_boundary)):
        oriteam_boundary[i][0][0] = origin[0] - width / 2
        oriteam_boundary[i][0][1] = origin[1] + long / 2
        oriteam_boundary[i][1][0] = origin[0] + width / 2
        oriteam_boundary[i][1][1] = origin[1] + long / 2
        oriteam_boundary[i][2][0] = origin[0] + width / 2
        oriteam_boundary[i][2][1] = origin[1] - long / 2
        oriteam_boundary[i][3][0] = origin[0] - width / 2
        oriteam_boundary[i][3][1] = origin[1] - long / 2

    global orienemy_boundary
    for i in range(len(oriteam_boundary)):#這段可不用，因為在這時候oppo_p數值還沒更新
        if oppo_p[i] != [0,0]:
            print("enemy_boundary")
            orienemy_boundary[i][0][0] = oppo_p[i][0] - width / 2
            orienemy_boundary[i][0][1] =  oppo_p[i][1] + long / 2
            orienemy_boundary[i][1][0] =  oppo_p[i][0] + width / 2
            orienemy_boundary[i][1][1] =  oppo_p[i][1] + long / 2
            orienemy_boundary[i][2][0] =  oppo_p[i][0] + width / 2
            orienemy_boundary[i][2][1] =  oppo_p[i][1] - long / 2
            orienemy_boundary[i][3][0] =  oppo_p[i][0] - width / 2
            orienemy_boundary[i][3][1] =  oppo_p[i][1] - long / 2


    # collide = detect_collide(1)
    # #detect collide
    global control_display
    control_display = 1
    global cha1_stage
    cha1_stage = 1
    global kicked
    kicked = False
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

    global control_display

    # Your code
    if (kick_goal):

        cv2.circle(frame, (int(kick_goal[0]), int(kick_goal[1])), 3, (0, 0, 255), -1)

    if(kick_goal_list):
        for x in range(len(kick_goal_list)):
            cv2.circle(frame, (int(kick_goal_list[x][0]), int(kick_goal_list[x][1])), 3, (232, 122, 63), -1)

    if stage > 1:
        cv2.circle(frame, (int(first_arri[0]), int(first_arri[1])), 3, (237, 183, 217), -1)
        # cv2.line(frame, (ball_p[0], ball_p[1]), (kick_goal[0], kick_goal[1]), (0, 0, 255), 2)
        # cv2.circle(frame, (int(player_p[0]), int(player_p[1])), 3, (0, 0, 255), -1)
    if stage > 2:
        cv2.circle(frame, (int(kick_point[0]), int(kick_point[1])), 3, (232, 122, 63), -1)

    if (kick_goal):
        cv2.circle(frame, (int(kick_goal[0]), int(kick_goal[1])), 3, (0, 0, 255), -1)

    for x in range(len(team_boundary)):
        if team_boundary[x] != [[0, 0], [0, 0], [0, 0], [0, 0]]:
            # cv2.circle(frame, (int(kick_point[0]), int(kick_point[1])), 3, (232, 122, 63), -1)
            for i in range(4):
                cv2.circle(frame, (int(team_boundary[x][i][0]), (int(team_boundary[x][i][1]))), 3, (237, 183, 217), -1)

    for x in range(len(orienemy_boundary)):
        if orienemy_boundary[x] != [[0, 0], [0, 0], [0, 0], [0, 0]]:
            # cv2.circle(frame, (int(kick_point[0]), int(kick_point[1])), 3, (232, 122, 63), -1)
            for i in range(4):
                cv2.circle(frame, (int(orienemy_boundary[x][i][0]), (int(orienemy_boundary[x][i][1]))), 3, (237, 183, 217), -1)

    #detect collide
    # if(control_display==1):
    #     for i in range(len(not_touchable_region)):
    #         cv2.circle(frame, (int(not_touchable_region[i][0]), (int(not_touchable_region[i][1]))), 3, (237, 183, 217), -1)
    # control_display = 1

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
    global oriplayer_d, oriplayer_p, player_id, oppo_id,ball_p,oppo_p
    i = 0
    # for i in range(len(teamD)):
    #     oriplayer_d[i] = teamD[i]
    #     oriplayer_p[i] = teamP[i]
    # oriplayer_p[0] = teamP[0]
    # oriplayer_p[1] = teamP[2]
    # oriplayer_d[0] = teamD[0]
    # oriplayer_d[2] = teamD[2]
    #真實用
    # print("Challenge1接受到訊號","Robot0==",teamP[0],"Robot1==",teamP[1],"Ball_p",ballP,"Op0",oppo_p[0],"Op1",oppo_p[1])
    # print("OOOOOOOOOOOOOOOOOOO",oriplayer_d)
    oriplayer_p[0] = teamP[0]
    oriplayer_p[1] = teamP[1]
    oriplayer_d[0] = teamD[0]
    oriplayer_d[1] = teamD[1]
    oppo_p[0] = oppoP[oppo_id - 3]  # 為什麼‘-3？’ 為了歸0？(以得出list中的第一個的值)因為一開始opp_id是3
    oppo_p[1] = oppoP[oppo_id - 3 + 1]
    ball_p = ballP

    # print("Challenge1接受到訊號","Robot0==",teamP[0],"Robot1==",teamP[1],"Ball_p",ballP,"Op0",oppo_p[0],"Op1",oppo_p[1])
    # print("angle0",teamD[0],"angle1",teamD[1])
    pass


def strategy():
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """
    player_id = 1
    global stage
    global Robot
    global  kick_goal, stopit, kick_goal_list, block
    global kick_way
    global cha1_stage,x_nextpoint_index,x_target_index,y_target_index
    Robot = ['N1','N1','N1']
    # Your code
    #boundary_
    # boundary = robot_self_boundary(0, 0)
    alter_boundary = robot_self_boundary(1, 0)

    # collide detect
    # temp = detect_collide(0)
    # if temp != "No Matter":
    #     return temp
    #
    # temp = detect_collide(0)
    #
    # if temp != "No Matter":
    #     return temp
    ###############################

    if cha1_stage == 1:
        print("Cha1_stage==",cha1_stage)
        global orienemy_boundary
        # print(BOUNDARY)
        for i in range(len(orienemy_boundary)):
            #計算敵人四點座標
            if oppo_p[i] != [0, 0]:
                orienemy_boundary[i][0][0] = oppo_p[i][0] - width / 2
                orienemy_boundary[i][0][1] = oppo_p[i][1] + long / 2
                orienemy_boundary[i][1][0] = oppo_p[i][0] + width / 2
                orienemy_boundary[i][1][1] = oppo_p[i][1] + long / 2
                orienemy_boundary[i][2][0] = oppo_p[i][0] + width / 2
                orienemy_boundary[i][2][1] = oppo_p[i][1] - long / 2
                orienemy_boundary[i][3][0] = oppo_p[i][0] - width / 2
                orienemy_boundary[i][3][1] = oppo_p[i][1] - long / 2
        #預設給機器人2的點
        global ori_center
        kick_goal = kick_goal_first#650,350
        return moverobot(player_id, kick_goal)
    elif cha1_stage == 2:
        player_p = oriplayer_p[player_id]
        player_d = oriplayer_d[player_id]
        dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
        global SAFE_DIST_robot0_1,SAFE_DIST_robot0_2
        if dist_ball > SAFE_DIST_robot0_1:
            Robot[player_id] = 'w1'
            return [Robot[0], Robot[1], Robot[2]]
        if dist_ball < SAFE_DIST_robot0_2:
            Robot[player_id] = 's1'
            return [Robot[0], Robot[1], Robot[2]]

        cha1_stage = 3

    elif cha1_stage == 3:
        global kicked
        if (kicked):
            cha1_stage = 4
            return ['N1', 'N1', 'N1']

        player_p = oriplayer_p[1]
        player_d = oriplayer_d[1]
        global team_boundary,kick_way
        boundary = []
        boundary = team_boundary[player_id]
        ptleft = abs(boundary[1][1] - ball_p[1])
        ptright = abs(boundary[2][1] - ball_p[1])
        if ptleft <= ptright:
            kick_way = "L"
        else:
            kick_way = "R"

        if kick_way == "L":
            if boundary[1][1] + 3 <= ball_p[1] <= player_p[1] -3:
                # cha1_stage = 4
                print("Left kick")
                Robot[player_id] = 'u1'
                return [Robot[0], Robot[1], Robot[2]]
            else:
                Robot[player_id] = 'd1'
                return [Robot[0], Robot[1], Robot[2]]
        elif kick_way == "R":
            if player_p[1] + 3 <= ball_p[1] <= boundary[2][1] - 3:
                # cha1_stage = 4
                print("Right kick")
                Robot[player_id] = 'i1'
                return [Robot[0], Robot[1], Robot[2]]
            else:
                Robot[player_id] = 'a1'
                return [Robot[0], Robot[1], Robot[2]]

    # elif cha1_stage == 4:





    # elif cha1_stage == 2:
    #     print("First job done", kick_way)
    #     print("Cha1_stage==", cha1_stage)
    #     # print("kick0",kicked)
    #     global kicked
    #     print("kick0",kicked)
    #     if (kicked):
    #         print("GOOD JOB")
    #         global runaway
    #         #撤退用
    #         if runaway < 3:
    #             runaway += 1
    #             if kick_way == 'L':
    #                 return ['N1', 'D1', 'N1']
    #             elif kick_way == 'R':
    #                 return ['N1', 'A1', 'N1']
    #
    #         # kicked = False
    #         cha1_stage = 3
    #     if (not kicked):
    #         # return ['N1',  MOVE[kick_way]['Kick'], 'N1']
    #         return ['N1', 'N1', 'N1']
    elif cha1_stage == 4:
        global runaway
        player_d = oriplayer_d[1]
        if runaway < runaway_index:

            if abs(player_d[0] - 1) < 0.05 or abs(player_d[1] - 0) < 0.05:
                runaway += 1
                Robot[player_id] = 's1'
                return [Robot[0], Robot[1], Robot[2]]

            if kick_way == 'L':
                runaway += 1
                return ['N1', 'A1', 'N1']
            elif kick_way == 'R':
                runaway += 1
                return ['N1', 'D1', 'N1']

        player_id = 0
        global block
        print("Cha1_stage==", cha1_stage)
        #計算將要射門的目標位置
        block = []
        block.append(oppo_p[0][1] - ROB_RANG)  # oppo_p[1]是robo0的y
        block.append(oppo_p[0][1] + ROB_RANG)  # ROB_RANG的意思？
        block.append(oppo_p[1][1] - ROB_RANG)  # oppo_p[1]是robo0的y
        block.append(oppo_p[1][1] + ROB_RANG)  # ROB_RANG的意思？
        block.sort()
        print("block", block)

        # targetx = 990
        targetx = BOUNDARY[4][0] + BOUNDARY[4][0]//10
        targety = (block[2] - block[1]) / 2 + block[1]

        i = int(abs(targetx - ball_p[0]) // 60)
        m = int(abs(targety - ball_p[1]) // 30)
        print("i==", i, "m==", m)

        kick_goal[0] = ball_p[0] + x_target_index
        kick_goal[1] = targety

        #如果m 或i=0，就直接去下一階段，一般只有m會等於0，這反而是最佳位置
        if m != 0 and i != 0:
            for x in range(0, i):
                kick_goal_list[x][0] = int(ball_p[0] + x_target_index * (x + 1))

            for x in range(0, m):
                if targety < ball_p[1]:
                    kick_goal_list[x][1] = int(ball_p[1] - y_target_index * (x + 1))
                else:
                    kick_goal_list[x][1] = int(ball_p[1] + y_target_index * (x + 1))
            #多出來的位置以targety進行填補
            for x in range(len(kick_goal_list)):
                if kick_goal_list[x][1] == 0:
                    kick_goal_list[x][1] = targety
            # 多出來的位置以targetx進行填補
            for x in range(len(kick_goal_list)):
                if kick_goal_list[x][0] == 0:
                    kick_goal_list[x][0] = targetx

        print(" kick goal list==", kick_goal_list)

        cha1_stage = 5

    elif cha1_stage == 5:
        #適合一直線運動之位置
        print("Cha1_stage==", cha1_stage)
        if block[1] + long/2 + 10 <= ball_p[1] <= block[2] - long/2 - 10  and ball_p[0] > 300 and stopit == 0:
            # i = (900 - ball_p[0]) // 60
            i = (BOUNDARY[4][0] - ball_p[0]) // x_target_index
            print("CHANGE　EVERYTHING，一直線運動", i)
            stopit = 1
            # firsttime = 3
            kick_goal_list = [[0, 0]]
            for x in range(0, i):
                kick_goal_list.append([ball_p[0] + x_target_index * (x + 1), ball_p[1]])

            # kick_goal_list.append([990, ball_p[1]])
            kick_goal_list.append([BOUNDARY[4][0] + BOUNDARY[4][0] // 10, ball_p[1]])
            return ['N1','N1','N1']

        print("Final kick_goal_list",kick_goal_list)

        for i in range(len(kick_goal_list)):
            # 選擇距離45以外的目標
            # if kick_goal_list[i][0] - ball_p[0] > 45 and kick_goal_list[0] != 990:
            if kick_goal_list[i][0] - ball_p[0] > x_nextpoint_index and kick_goal_list[0] != BOUNDARY[4][0] + BOUNDARY[4][0] // 10:
                kick_goal = kick_goal_list[i]
                print(" alr change to next goal")
                break

        global robot1_kicked

        if robot1_kicked :
            robot1_kicked = False

        print("KICK_GOAL_NEXT ==", kick_goal)
        player_id = 0
        return move_robotreal(player_id, kick_goal)

    return ['N1', 'N1', 'N1']

countofkicked =0
def get_sent_cmd(sentcmd, update):
    """
    Description:
        Simulator will pass the received strategy and a sending state
    Parameter:
        param1: list[str] -> received command
        param2: bool -> sent or not
    """
    # Your code
    global countofkicked
    if update:
        print('sent: ', sentcmd[0][0])
        if sentcmd[0][0] == 'N' and stage == 4:
            print('===N recieved==')
            global kick_flag
            kick_flag = True
        if sentcmd[1][0] == 'u' or sentcmd[1][0] == 'i' and update == True:
            global kicked
            countofkicked += 1
            # print("DONE")
            if countofkicked == 1:
                kicked = True
                countofkicked = 0
        if sentcmd[0][0] == 'j' or sentcmd[0][0] == 'h':
            global robot1_kicked
            # print("DONE")
            robot1_kicked = True
    pass


def _unit_vector(start, end):
    vector = [e - s for s, e in zip(start, end)]  # 應該是後減前
    length = math.hypot(vector[0], vector[1])
    if length == 0:
        print("Warning, expection!!!")
        length = 1

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

def move_robotreal(player_id, kick_goal):

    global kick_dir, first_arri,stage,kick_way,step,arrived,count
    player_p = oriplayer_p[player_id]
    # print("ball==", ball_p,"kick_goal==",kick_goal[1],"player_pst",player_p[1])

    if stage == 1:
        print("move_stage==", stage)
        first_arri = arrivepoint(player_id,kick_goal)

        kick_dir = _unit_vector(ball_p, kick_goal)

        stage += 1
    elif stage == 2:

        print("move_stage==",stage,"first arrived",first_arri)
        action = findball(player_id, first_arri)
        if action == "next stage":
            stage = 3
            return ['N1', 'N1', 'N1']
        else:
            return action
    elif stage == 3:

        print("move_stage==",stage)
        action = rotatedirection(player_id, kick_dir)
        if action == "next stage":
            stage = 4
            return ['N1', 'N1', 'N1']
        else:
            return action

    elif stage == 4:
        # run(0)
        print("move_stage==",stage)
        if kick_way == 'L':
            Robot[player_id] = 'h1'
            stage = 1
            return [Robot[0], Robot[1], Robot[2]]
        if kick_way == 'R':
            Robot[player_id] = 'j1'
            stage = 1
            return [Robot[0], Robot[1], Robot[2]]

    return ['N1', 'N1', 'N1']



def moverobot(player_id, kick_goal ):
    global kick_dir, first_arri,stage
    global cha1_stage
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    if stage == 1:
        print("next stage,move_stage==",stage)
        dist = math.hypot(oriplayer_p[1][0] - ball_p[0], oriplayer_p[1][1] - ball_p[1])
        if dist < SAFE_DIST:
            return ['N1', 's1', 'N1']
        first_arri = arrivepoint(player_id, kick_goal)
        kick_dir = _unit_vector(ball_p, kick_goal)

        stage += 1
    elif stage == 2:
        if abs(first_arri[0] - player_p[0]) >= 10:
            if first_arri[0] >= player_p[0] :
                Robot[player_id] = 'w1'
                return [Robot[0], Robot[1], Robot[2]]
            elif first_arri[0] < player_p[0]:
                Robot[player_id] = 's1'
                return [Robot[0], Robot[1], Robot[2]]
        if abs(first_arri[1] - player_p[1]) >= 10:
            if first_arri[1] >= player_p[1]:
                Robot[player_id] = 'd1'
                return [Robot[0], Robot[1], Robot[2]]
            elif first_arri[1] < player_p[1]:
                Robot[player_id] = 'a1'
                return [Robot[0], Robot[1], Robot[2]]

        stage +=1

    elif stage == 3:
        print("stage==", stage)
        action = rotatedirection_robot0(player_id, kick_dir)
        if action == "next stage":
            stage = 4
            print("next stage,move_stage==",stage)
            cha1_stage = 2
            return ['N1', 'N1', 'N1']
        else:
            return action

    return ['N1', 'N1', 'N1']


def findball(player_id,first_arri):
    global move_way
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    move_dir = _unit_vector(player_p, first_arri)
    product = -1
    for way in ALLOW_MOVE_WAY:  # 判斷轉左後較近還是轉右後較靠近
        temp_product = _dot(move_dir, _rotate(player_d, WAY_ANGLE[way]))
        # print(way, ':', temp_product, move_dir, _rotate(player_d, WAY_ANGLE[way]))
        if temp_product > product:

            product = temp_product
            move_way = way
    # print('move:', move_way)

    move_dir = _unit_vector(player_p, first_arri)
    dist = math.hypot(player_p[0] - first_arri[0], player_p[1] - first_arri[1])
    dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
    # print('arri-player-dist', first_arri, player_p, dist)
    if dist >= ERROR_DISTANCE and dist_ball > SAFE_DIST + 8:  # ERROR dist 應是為了避免要去的地方太近，出現error，dist_ball應該是為了避免不小心碰到球
        dirction = _rotate(player_d, WAY_ANGLE[move_way])

        # print("move_direction", dirction)
        # print("rotate", _rotate(player_d, WAY_ANGLE[move_way]))
        angle = _angle(move_dir, dirction)
        if angle > 0 and angle > 2 * ROTATE_ANGLE:
            Robot[player_id] = 'Q1'
            return [Robot[0], Robot[1], Robot[2]]
        elif angle < 0 and angle < -2 * ROTATE_ANGLE:

            Robot[player_id] = 'E1'
            return [Robot[0], Robot[1], Robot[2]]
        elif angle > 0 and angle > ROTATE_ANGLE:

            Robot[player_id] = 'q1'
            return [Robot[0], Robot[1], Robot[2]]
        elif angle < 0 and angle < -ROTATE_ANGLE:

            Robot[player_id] = 'e1'
            return [Robot[0], Robot[1], Robot[2]]
        '''
        MOVE
        '''
        # MOVE = {
        #     'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [18, 10]},
        #     'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [80, 8]},
        #     'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [80, 8]},
        #     'B': {'Norm': 's1', 'Bound': [None, 10]}
        # }
        if dist >= MOVE[move_way]['Bound'][0]:  # 超過80的話走快點

            # return ['N1', 'N1', 'N1']
            Robot[player_id] = MOVE[move_way]['Fast']
            return [Robot[0], Robot[1], Robot[2]]
            # return [MOVE[move_way]['Fast'], 'N1', 'N1']
        elif dist >= MOVE[move_way]['Bound'][1]:

            # return ['N1', 'N1', 'N1']
            Robot[player_id] = MOVE[move_way]['Norm']
            return [Robot[0], Robot[1], Robot[2]]

    return "next stage"

def arrivepoint(player_id,kick_goal):
    global first_arri
    final_first_arri = [0,0]
    ball_pnext=[0,0]
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    kick_dir = _unit_vector(ball_p, kick_goal)
    first_arri = [int(ball_p[i] - int(kick_dir[i] * ROUGH_RANG)) for i in range(2)]

    global kick_point
    kick_point = [int(ball_p[i] - kick_dir[i] * SAFE_DIST) for i in range(2)]

    return first_arri

def rotatedirection_robot0(player_id,kick_dir):
    # Choose the way of kick
    # print("ball==",ball_p,first_arri)
    # if abs(ball_p[1] - first_arri[1]) <= 10:
    #     return "next stage"
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    global kick_way
    global move_way
    # return ['N1', 'N1', 'N1']
    # if kick_way == "":
        # print('S3 arri-player-dist', first_arri, player_p)
        # product = -1
        # for way in ALLOW_MOVE_WAY:
        #     # print("rotate",_rotate(player_d, WAY_ANGLE[way]))
        #     temp_product = _dot(kick_dir, _rotate(player_d, WAY_ANGLE[way]))
        #     # print(way, ':', temp_product, kick_dir, _rotate(player_d, WAY_ANGLE[way]))
        #     if temp_product > product:
        #         product = temp_product
        #         kick_way = way
        # print("kick way", kick_way)
    dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
    dirction = player_d

    angle = _angle(kick_dir,dirction)

    if angle > 0 and angle > 2 * ROTATE_ANGLE:
        # print('turn big left')
        Robot[player_id] = 'Q1'
        return [Robot[0], Robot[1], Robot[2]]
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


    # WAYS = ['F', 'R', 'B', 'L']
    # for i in [1, 2, 3, 0]:
    #     move_way = WAYS[(WAYS.index(kick_way)+i) % 4]
    #     # print("kick_way",kick_way," ",WAYS.index(kick_way),"i==",i,move_way)
    #     temp_dir = _rotate(dirction, math.pi/2*i)
    #     diff_vec = [k - p for k, p in zip(kick_point, player_p)]
    #     product = _dot(temp_dir, diff_vec)
    #
    #
    #     if product > MOVE[move_way]['Bound'][1]:
    #         # print('move:', MOVE[move_way]['Norm'])
    #         Robot[player_id] = MOVE[move_way]['Norm']
    #         return [Robot[0], Robot[1], Robot[2]]


    return "next stage"

def rotatedirection(player_id,kick_dir):
    # Choose the way of kick
    # print("ball==",ball_p,first_arri)
    # if abs(ball_p[1] - first_arri[1]) <= 10:
    #     return "next stage"
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

    angle = _angle(kick_dir,dirction)

    if angle > 0 and angle > 2 * ROTATE_ANGLE:
        # print('turn big left')
        Robot[player_id] = 'Q1'
        return [Robot[0], Robot[1], Robot[2]]
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


        if product > MOVE[move_way]['Bound'][1]:
            # print('move:', MOVE[move_way]['Norm'])
            Robot[player_id] = MOVE[move_way]['Norm']
            return [Robot[0], Robot[1], Robot[2]]


    return "next stage"



#boundary

def robot_self_boundary(player_id,angle_try):
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]
    global anglenew , directionold
    # print(player_d)
    boundary = 0
    if directionold[player_id] != [0, 0] and directionold[player_id] != player_d:
        # kick_dir = _unit_vector(directionold, player_d)
        anglenew = _angle(player_d, directionold[player_id])

        if angle_try!=0:
            anglenew = angle_try

        # print("rotateangle", anglenew)
        boundary = robot_boundary(player_id)
    else:
        anglenew = 0
        boundary = robot_boundary(player_id)
        # anglenew = 0
    directionold[player_id] = player_d
    return boundary

def robot_boundary(player_id):
    global long,width,blk2,anglenew,blk3
    global kick_dir
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_p[player_id]

    anglex = anglenew
    #以機器人本身作為坐標
    for i in range(len(oriteam_boundary[player_id])):
        oriteam_boundary[player_id][i] = _rotate(oriteam_boundary[player_id][i], anglex)
    # print("blk2final with robot as origin", oriteam_boundary[player_id])

    ##換回原先坐標
    team_boundary[player_id] = [[0, 0], [0, 0], [0, 0], [0, 0]]
    for i in range(len(oriteam_boundary[player_id])):
        # print(player_p,"*******************************************************")
        team_boundary[player_id][i][0] = round(oriteam_boundary[player_id][i][0] + player_p[0])

    for i in range(len(oriteam_boundary[player_id])):
        team_boundary[player_id][i][1] = round(abs(0 - player_p[1] + oriteam_boundary[player_id][i][1]))

    # print("blk2after change to original",team_boundary[player_id])
    # print('ang:', anglex * 180 / math.pi)
    return team_boundary[player_id]

def detect_collide(player_id):
    global not_touchable_region,long,width
    global orienemy_boundary
    global kick_way,kick_goal

    boundary = robot_self_boundary(0,0)
    player_p = oriplayer_p[player_id]
    player_d = oriplayer_d[player_id]

    block = []
    block.append(oppo_p[0][1] - ROB_RANG)  # oppo_p[1]是robo0的y
    block.append(oppo_p[0][1] + ROB_RANG)  # ROB_RANG的意思？
    block.append(oppo_p[1][1] - ROB_RANG)  # oppo_p[1]是robo0的y
    block.append(oppo_p[1][1] + ROB_RANG)  # ROB_RANG的意思？
    block.sort()
    #
    # print("me",boundary,"eme1",orienemy_boundary[0],"eme2",orienemy_boundary[1])
    #
    #
    # print("ans",boundary,block, orienemy_boundary[0][0][0], orienemy_boundary[0][1][0])

    if boundary[1][1] <= boundary[0][1]:
        move_way = "R"
    elif boundary[1][1] > boundary[0][1]:
        move_way = "L"
    # print("boundary==", boundary)
    # print("move-way",move_way)

    for e in range(2):
        tmp_collide = (collide(boundary,orienemy_boundary[e]))
        # print(tmp_collide)

        if (tmp_collide)== 999:
            pass
        else:
            print("Collided , TRY TO MOVE XDDDDDD")
            tmp_boundary = [[0,0],[0,0],[0,0],[0,0]]

            if move_way =="L":
                print("move_way==L")
                # print("bnd",boundary,"ene",orienemy_boundary[e])
                for i in range(len(boundary)):
                    tmp_boundary[i][1] = boundary[i][1] - 5
                    tmp_boundary[i][0] = boundary[i][0]


                # print("result", collide(tmp_boundary, orienemy_boundary[e]),tmp_boundary)
                if collide(tmp_boundary,orienemy_boundary[e])==999:
                    # print("result",collide(tmp_boundary,orienemy_boundary[e]))
                    print("up s")
                    kick_goal[1] = kick_goal[1] + 5
                    arrivepoint(0,kick_goal)
                    return ['s1', 'N1', 'N1']

                for i in range(len(boundary)):
                    tmp_boundary[i][1] = boundary[i][1] + 5
                    tmp_boundary[i][0] = boundary[i][0]

                # print("result", collide(tmp_boundary, orienemy_boundary[e]),tmp_boundary)
                if collide(tmp_boundary,orienemy_boundary[e])==999:
                    # print("result", collide(tmp_boundary, orienemy_boundary[e]))
                    print("down w")
                    kick_goal[1] = kick_goal[1] - 5
                    arrivepoint(0, kick_goal)
                    # first_arri[1] = first_arri[1] + 10
                    return ['w1', 'N1', 'N1']

                for i in range(len(boundary)):
                    tmp_boundary[i][0] = boundary[i][0] - 5
                    tmp_boundary[i][1] = boundary[i][1]
                if collide(tmp_boundary,orienemy_boundary[e])==999:
                    print("left d")
                    # kick_goal[0] = kick_goal[0] + 5
                    # arrivepoint(0, kick_goal)

                    return ['d1', 'N1', 'N1']

                for i in range(len(boundary)):
                    tmp_boundary[i][0] = boundary[i][0] + 5
                    tmp_boundary[i][1] = boundary[i][1]
                if collide(tmp_boundary,orienemy_boundary[e])==999:
                    print("right a")
                    # kick_goal[0] = kick_goal[0] - 5
                    # arrivepoint(0, kick_goal)

                    return ['a1', 'N1', 'N1']



            if move_way == "R":
                print("move_way==R")
                # print("bnd", boundary, "ene", orienemy_boundary[e])
                for i in range(len(boundary)):
                    tmp_boundary[i][1] = boundary[i][1] - 5
                    tmp_boundary[i][0] = boundary[i][0]

                # print("result", collide(tmp_boundary, orienemy_boundary[e]),tmp_boundary)

                if collide(tmp_boundary, orienemy_boundary[e]) == 999:

                    print("up w")
                    # first_arri[1] = first_arri[1] - 10
                    kick_goal[1] = kick_goal[1] + 5
                    arrivepoint(0, kick_goal)
                    return ['w1', 'N1', 'N1']

                for i in range(len(boundary)):
                    tmp_boundary[i][1] = boundary[i][1] + 5
                    tmp_boundary[i][0] = boundary[i][0]
                # print("result", collide(tmp_boundary, orienemy_boundary[e]),tmp_boundary)
                if collide(tmp_boundary, orienemy_boundary[e]) == 999:
                    print("down s")
                    kick_goal[1] = kick_goal[1] - 5
                    arrivepoint(0, kick_goal)
                    # first_arri[1] = first_arri[1] + 10
                    return ['s1', 'N1', 'N1']

                tmp_boundary = [[0, 0], [0, 0], [0, 0], [0, 0]]
                for i in range(len(boundary)):
                    tmp_boundary[i][0] = boundary[i][0] - 5
                    tmp_boundary[i][1] = boundary[i][1]
                if collide(tmp_boundary, orienemy_boundary[e]) == 999:
                    print("left a")
                    # kick_goal[0] = kick_goal[0] + 5
                    # arrivepoint(0, kick_goal)
                    return ['a1', 'N1', 'N1']

                for i in range(len(boundary)):
                    tmp_boundary[i][0] = boundary[i][0] + 5
                    tmp_boundary[i][1] = boundary[i][1]
                if collide(tmp_boundary, orienemy_boundary[e]) == 999:
                    print("right d")
                    # kick_goal[0] = kick_goal[0] - 5
                    # arrivepoint(0, kick_goal)
                    return ['d1', 'N1', 'N1']

    return "No Matter"


def collide(boundary_player,boundary_enemy):
    player_line = [0,0,0,0]
    player_line[0] = [[0,0],[0,0]]
    player_line[1] = [[0,0],[0,0]]
    player_line[2] = [[0,0],[0,0]]
    player_line[3] = [[0,0],[0,0]]
    enemy_line = [0, 0, 0, 0]
    enemy_line[0] = [[0, 0], [0, 0]]
    enemy_line[1] = [[0, 0], [0, 0]]
    enemy_line[2] = [[0, 0], [0, 0]]
    enemy_line[3] = [[0, 0], [0, 0]]

    for i in range(3):
        player_line[i] = [boundary_player[i][0],boundary_player[i][1]],[boundary_player[i+1][0],boundary_player[i+1][1]]
    player_line[3] = [boundary_player[3][0], boundary_player[3][1]], [boundary_player[0][0],boundary_player[0][1]]

    for i in range(3):
        enemy_line[i] = [boundary_enemy[i][0], boundary_enemy[i][1]], [boundary_enemy[i + 1][0],boundary_enemy[i + 1][1]]
    enemy_line[3] = [boundary_enemy[3][0], boundary_enemy[3][1]], [boundary_enemy[0][0], boundary_enemy[0][1]]
    count = 0
    for m in range(4):
        for i in range(4):
            result = line_intersection(player_line[i],enemy_line[m])
            if(result !="lines do not intersect"):
                print("intersect at ",result)
                count+=1
    if count == 0:
        return 999




def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return "lines do not intersect"

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    tmpx_line = [0, 0]
    tmpy_line = [0,0]
    tmpx_line2 = [0, 0]
    tmpy_line2 = [0, 0]

    if line1[0][1] >= line1[1][1]:
        tmpy_line[1] = line1[0][1]
        tmpy_line[0] = line1[1][1]
    else:
        tmpy_line[0] = line1[0][1]
        tmpy_line[1] = line1[1][1]

    if line1[0][0] >= line1[1][0]:
        tmpx_line[1] = line1[0][0]
        tmpx_line[0] = line1[1][0]
    else:
        tmpx_line[0] = line1[0][0]
        tmpx_line[1] = line1[1][0]

    if line2[0][1] >= line2[1][1]:
        tmpy_line2[1] = line2[0][1]
        tmpy_line2[0] = line2[1][1]
    else:
        tmpy_line2[0] = line2[0][1]
        tmpy_line2[1] = line2[1][1]


    if line2[0][0] >= line2[1][0]:
        tmpx_line2[1] = line2[0][0]
        tmpx_line2[0] = line2[1][0]
    else:
        tmpx_line2[0] = line2[0][0]
        tmpx_line2[1] = line2[1][0]


    if tmpy_line[0]  <= y <= tmpy_line[1] and tmpx_line[0]  <= x <= tmpx_line[1] and tmpy_line2[0]  <= y <= tmpy_line2[1] and tmpx_line2[0]  <= x <= tmpx_line2[1]:
        # print("NO")
        return x, y
    else:
        return "lines do not intersect"
