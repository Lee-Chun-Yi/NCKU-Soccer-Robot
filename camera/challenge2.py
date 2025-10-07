import math
import cv2
from time import sleep

# Const
BALL_RADIUS = 10  # Ball Size #diameter 6.5cm = 20 pixel
WIDTH = 83  # 27cm = 83pixel
DEPTH = 43  # 14cm = 43 pixel
DIAGONAL = ((WIDTH ** 2) + (DEPTH ** 2)) ** 0.5  # Robot Size

ROB_RANG = 25
KICKABLE_RANGE = 40
ROUGH_RANG = 60
ERROR_DISTANCE = 10
ROTATE_ANGLE = 0.25
SAFE_DIST = 49  # RoboRad/2 + ball radius
WAY_ANGLE = {'F': 0, 'L': -math.pi / 2, 'R': math.pi / 2}
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
player_id = 0
player_p = [0, 0]
player_d = [0.0, 0.0]
oppo_id = 3
oppo_p = [0, 0]

# Anticipate point

stage = 0
kick_goal = [0, 0]
up_down = 0  # decide defender's position. 0 is up 1 is down
shoot_dir = [0.0, 0.0]  # ball to goal vector
kick_pos = [0, 0]
move_dir = [0.0, 0.0]  # robot will move along this dir
kick_dir = [0.0, 0.0]
turned_angle = 0
first_arri = [0, 0]
kick_point = [0, 0]
kick_way = ""
move_way = ""
toward_dir = [0.0, 0.0]
kick_flag = False
kicked = False
change1 = False
change2 = False


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
    player_id = 0
    oppo_id = 3
    print("Your Player is Robot", player_id)
    print("Obstacle is Robot", oppo_id)
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
    global player_d, player_p, player_id, oppo_id, oppo_p, ball_p, change1, change2
    if not teamD[player_id]:
        player_d = teamD[player_id]
        change1 = True
    else:
        player_d = teamD[player_id]
        change1 = True

    if not teamP[player_id]:
        player_p = teamP[player_id]
        change2 = True
    else:
        player_p = teamP[player_id]
        change2 = True

    if oppoP[oppo_id - 3] is not None:
        oppo_p = oppoP[oppo_id - 3]
    if ballP is not None:
        ball_p = ballP

    print("Team_dir: ", player_d," Team_pos: ", player_p)


def strategy():
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """

    global stage, up_down, shoot_dir, kick_pos, move_dir, kick_point, turned_angle
    # Your code
    if stage == 1:
        # Find position of goal
        kick_goal[0] = BOUNDARY[2][0]  # kick_goal[0]=870 球門的x座標
        up_down = 0 if oppo_p[1] <= 260 else 1  # if 1 is below ceter,vice versa
        kick_point = [int(ball_p[i] - shoot_dir[i] * SAFE_DIST) for i in range(2)]
        if up_down == 0:
            kick_goal[1] = oppo_p[1] + ROUGH_RANG
        else:
            kick_goal[1] = oppo_p[1] - ROUGH_RANG
        shoot_dir = _unit_vector(ball_p, kick_goal)  # shoot direction so the same as rob direction
        kick_pos = [ball_p[0] - shoot_dir[0] * SAFE_DIST,
                    ball_p[1] - shoot_dir[1] * SAFE_DIST]  # the position will kick the ball
        # print('kickGoal(x y)', kick_goal)
        # print('shoot_dir', shoot_dir)
        # print('kick_pos', kick_pos)
        # print('Goal(x y)', kick_goal[0], kick_goal[1])
        stage += 1

    elif stage == 2:
        move_dir = _unit_vector(player_p, kick_pos) if change2 else move_dir
        print('move_dir', move_dir)
        print('player_d', player_d)
        turned_angle = _angle(move_dir,
                              player_d) - math.pi / 2 if change1 else turned_angle  # the angle robot should turn
        print('turned_angle', turned_angle)
        if turned_angle > 0 and turned_angle > 2 * ROTATE_ANGLE:
            return ['Q1', 'N1', 'N1']
        elif turned_angle < 0 and turned_angle < -2 * ROTATE_ANGLE:
            return ['E1', 'N1', 'N1']
        elif turned_angle > 0 and turned_angle > ROTATE_ANGLE:
            return ['q1', 'N1', 'N1']
        elif turned_angle < 0 and turned_angle < -ROTATE_ANGLE:
            return ['e1', 'N1', 'N1']

        # move!!!!!
        # Find the way of move
        # move_dir = _unit_vector(player_p, kick_pos)
        global move_way
        move_way = ALLOW_MOVE_WAY[0] if player_d[1] > 0 else ALLOW_MOVE_WAY[1]
        print('move:', move_way)
        dist = math.hypot(player_p[0] - kick_pos[0], player_p[1] - kick_pos[1])  # 勾股定理
        print('distance', dist, ' player:', player_p, '   ball:', kick_pos)
        if dist >= MOVE[move_way]['Bound'][0]:
            return [MOVE[move_way]['Fast'], 'N1', 'N1']
        elif dist >= MOVE[move_way]['Bound'][1]:
            return [MOVE[move_way]['Norm'], 'N1', 'N1']
        print('======================================== enter stage 3')
        stage += 1

    elif stage == 3:
        # Choose the way of kick
        global kick_way
        angle = _angle(shoot_dir, player_d) - math.pi / 2 if change1 else turned_angle
        print('ang:', angle * 180 / math.pi)
        if angle > 0 and angle > 2 * ROTATE_ANGLE:
            print('turn big left')
            return ['Q1', 'N1', 'N1']
        elif angle < 0 and angle < -2 * ROTATE_ANGLE:
            print('turn big  right')
            return ['E1', 'N1', 'N1']
        elif angle > 0 and angle > ROTATE_ANGLE:
            print('turn left')
            return ['q1', 'N1', 'N1']
        elif angle < 0 and angle < -ROTATE_ANGLE:
            print('turn right')
            return ['e1', 'N1', 'N1']

        product = -1
        for way in ALLOW_MOVE_WAY:
            temp_product = _dot(kick_dir, _rotate(player_d, WAY_ANGLE[way]))
            print(way, ':', temp_product, kick_dir, _rotate(player_d, WAY_ANGLE[way]))
            if temp_product > product:
                product = temp_product
                kick_way = way
        print("kick way", kick_way)
        stage += 1
    dist_ball = math.hypot(player_p[0] - ball_p[0], player_p[1] - ball_p[1])
    print('player, kick ball', player_p, kick_point, dist_ball)
    if kick_flag and not kicked:
        print('===kicked===', MOVE[kick_way]['Kick'])
        # sleep(1)  # For check
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
    dot_result = _dot(a, b)
    cos = dot_result / (math.hypot(a[0], a[1]) * math.hypot(b[0], b[1]))
    return math.acos(cos)
