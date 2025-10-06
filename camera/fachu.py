import math
import constant as CONST
from enum import Enum
from time import sleep

# Parameter needed to adjust
ID_IN_USE = [3, 3]

# Field Parameter
CM_TO_PIX = 3.7
BOUNDARY = []
CENTER = [505, 260]
PENALTY = 0
SIDE = 1  # right is our field
FB_X = 0
GOAL = []

# Const
WIDTH = 83  # 27cm = 83pixel
DEPTH = 43  # 14cm = 43 pixel
DIAGONAL = ((WIDTH ** 2) + (DEPTH ** 2)) ** 0.5  # Robot Size

ROB_RANG = 25
KICKABLE_RANGE = 40
ROUGH_RANG = 60
ERROR_DISTANCE = 10
ROTATE_ANGLE = 0.26
SAFE_DIST = 30  # front kick
SAFE_DIST2 = 45  # side kick
WAY_ANGLE = {'F': 0, 'L': -math.pi / 2, 'R': math.pi / 2}
MOVE = {
    'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [18, 10]},
    'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [80, 8]},
    'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [80, 8]},
    'B': {'Norm': 's1', 'Bound': [None, 10]}
}
# Condition
ALLOW_MOVE_WAY = ['L', 'R']

#
stage = 0
robots = []
enemies = []
ball = None
ROB_RANG = 40  # 12cm = 40pixel
kick_goal = [0, 0]
shoot_dir = [0.0, 0.0]  # ball to goal vector
kick_pos = [0, 0]
move_dir = [0.0, 0.0]  # robot will move along this dir
kick_dir = [0.0, 0.0]
kick_point = [0, 0]
kick_way = ""
move_way = ""
kick_flag1 = False
kick_flag2 = False
kicked = False

count = 0 #把罰球寫死的
job = ['N1', 'N1', 'N1']
# fachu_list = ['d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1','d1']


#####可以用的策略######


#####7號######
#fachu_list = ['W1','W1','d1','d1','w1','i1'] # 往左踢（7號 放中間 腳對齊線）
# fachu_list = ['W1','W1','W1','e1','e1','e1','e1','e1','e1','w1','u1'] # 往右踢（7號 放中間 腳對齊線）#回到噴氣前

#####3號######
#fachu_list = ['W1','W1','q1','q1','q1','q1','d1','d1','w1','i1'] # 往左踢（3號 放中間 腳對齊線）
#fachu_list = ['W1','W1','W1','W1','j1','j1'] # 往右踢（3號 放中間 腳對齊線）


#####4號######
fachu_list = ['N1','N1','N1','N1','N1','N1','N1','N1','N1','N1','N1','f1','N1','N1','N1','N1','N1','N1','N1','N1','N1'] # 往右撲
# fachu_list = ['N1','N1','N1','N1','N1','f1','N1','N1','N1','N1','N1','N1','N1','N1','N1']  # 往左撲


#####絕望罰球######
#fachu_list = ['e1','e1','e1','e1','e1','e1','A1','A1','a1','h1','h1','h1','h1','h1'] #轉轉轉踢（放線後）
#fachu_list = ['W1','W1','W1','u1']

fachu_id = 2


def strategy_update_field(side, boundary, center, pk_x, fb_x, fb_y, penalty_y, ga_x, ga_y):
    """
    Description:
        Pass field information into strategy system.
        This will be called only once by the simulator.
    Parameter:
        param1: 0/1 -> 1 if left attack right, -1 if right attack left
        param2: list[list(int)] -> 12 boundary points of the field
        param3: list[int] -> center of the field
        param4: list[list(int)] -> 4 penalty corner
        param5: int -> x coordinate of free ball point w.r.t center
    """
    # Your code
    global BOUNDARY, CENTER, PENALTY, SIDE, FB_X, GOAL
    for pos in boundary:
        BOUNDARY.append((pos[0], pos[1]))
    CENTER = center
    PENALTY = penalty_y
    # for simulator
    SIDE = -1 * side
    FB_X = fb_x
    # x = 3 if a==2 else 0
    GOAL = [BOUNDARY[11], BOUNDARY[8]] if SIDE == 1 else [BOUNDARY[2], BOUNDARY[5]]
    print(GOAL)


def Initialize():
    """
    Description:
        Initialise the strategy.
        This function will be called by the simulator before a simulation is started.
    """
    print('initialize STA')
    global robots, enemies, ball, stage
    stage = 1
    for i in range(2):
        robots.append(Robot(ID_IN_USE[i]))
    for i in range(2):
        enemies.append([])
    ball = Ball()
    print('ball:', type(ball))


def draw_on_simulator(frame):
    """
    Description:
        Draw whatever you want on the simulator.
        Before the simulator update window, it will call this function and you can just draw anything you want.
        This function will be called every time the simulator is going to update frame.
        This function will be called everytime the simulator is going to update frame.
    Parameter:
        param1: numpy array -> the frame that will be displayed
    Return:
        retva1: numpy array -> the frame that will be displayed
    """
    # Your code
    return frame


def Update_Robo_Info(teamD, teamP, oppoP, ballP, receive_count):
    """
    Description:
        Pass robot and ball info into strategy.
        This function will be called before every time the simulator ask for strategy
    Parameter:
        param1: list[list[float]] -> [x,y] for our teammate robot direction
        param2: list[list[int]] -> [x,y] for our teammate robot position
        param3: list[list[int]] -> [x,y] for opponent's robot position
        param4: list[int] -> [x,y] for ball position
    """
    # Your code
    global robots, ball, enemies ,count

    count = receive_count

    for i in range(2):

        if len(teamP[i]) > 0:
            if teamP[i] is not None:
                robots[i].pos = teamP[i]
        if len(teamD[i]) > 0:
            if teamD[i] is not None:
                robots[i].dir = teamD[i]
        if len(oppoP) > 0:
            if oppoP[i] is not None:
                enemies[i] = oppoP[i]
    if ballP is not None:
        ball.pos = ballP


def strategy():
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: str -> command for this robot
    """
    global count ,fachu_list ,job ,fachu_id
    job = ['N1', 'N1', 'N1']

    if(0<= count < len(fachu_list)):
        job[fachu_id] = fachu_list[count]
    else:
        job= ['N1', 'N1', 'N1']

    return job




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
        print('sent: ', sentcmd[1])
        if sentcmd[1] == 'N2' and stage == 8:
            print('===N2 recieved==')
            global kick_flag2
            kick_flag2 = True
        if sentcmd[1] == 'j1' or sentcmd[1] == 'h1':
            global kicked
            kicked = True


class Robot():
    """
    Attributes:
        ID: An int stands for the robot's ID(1-7)
        pos: An list[x,y] stands for the robot's position
        dir: list[x,y] -> The direction the robot faces, stored in unit vector
        role: A Role(Enum) represents the robot's role
        job: A Job(Enum) -> the move the robots are going to execute
        MOTION: motion contants from constant.py
        BODY: robot size from constant.py
    """

    def __init__(self, ID):
        self.ID = ID
        self.pos = [-1, -1]
        self.dir = [0, 0]
        self.role = ''
        self.job = Job.NONE
        # self.MOTION = CONST.getMotion(ID)
        # self.BODY = CONST.getBody()
        self.aim_pos = [-1, -1]


class Role(Enum):
    NONE = 0
    MAIN = 1  # attacker or defender
    SUP = 2  # supporter
    GK = 3


class Job(Enum):
    NONE = 0
    MOVE = 1
    PASS = 2
    KICK = 3
    DRIBBLE = 4
    LEAVE = 5
    REST = 6


class Ball():
    def __init__(self):
        self.pos = [0, 0]
        # self.RADIUS = CONST.getRadius()


if __name__ == '__main__':
    pass


def _dot(x, y):
    """Dot product as sum of list comprehension doing element-wise multiplication"""
    return sum(x_i * y_i for x_i, y_i in zip(x, y))


def _unit_vector(start, end):
    vector = [e - s for s, e in zip(start, end)]
    length = math.hypot(vector[0], vector[1])
    uniVector = [comp / length for comp in vector]
    return uniVector


def _angle(a, b):
    dot_result = _dot(a, b)
    cos = dot_result / (math.hypot(a[0], a[1]) * math.hypot(b[0], b[1]))
    return math.acos(cos)


def _rotate(vector, angle):
    rot_vector = [0.0, 0.0]
    rot_vector[0] = (math.cos(angle) * vector[0]) - (math.sin(angle) * vector[1])
    rot_vector[1] = (math.sin(angle) * vector[0]) + (math.cos(angle) * vector[1])
    return rot_vector
