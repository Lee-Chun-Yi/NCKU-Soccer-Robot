import cv2
import math
import time
import enum
import numpy as np

import challenge2 as st_A
import blank_strategy as st_B

# Virtual field parameter
#   Frame width is not field width, it is the size of simulation window
#   The size of the field is defined in BOUNDARY. You can use SCALE to adjust field size
BOUNDARY = []
CENTER = []
OFFSET = []  # Offset is to calibrate the total field position
SCALE = 1

# Game parameter
Side = -1  # 1:Left A  Right B  #-1:Left B  Right A
CMD_FPS = 2
CAM_FPS = 15

# Robot movement parameter
Robo_rotate_speed = 0.030  # radius change
'''
    the step size of a robot for each command
    [fast_for, for, back, move_left, move_right, fast_move_l, fast_move_r]
'''
Robo_move = [1.2, 0.66, 0.66, 0.53, 0.53, 5.3, 5.3]  # [4, 3, 3, 4, 4, 6, 6]

# Ball simulation parameter
Ball_bump_speed = 3  # Bump speed when robot walking
Ball_kicked_speed = 4  # Robot kick speed
Ball_n_acce = 0.1  # Ball moving deacceleration !!!!!!!!!!!!!!!!!!!!!!!!!!!!
Ball_passing_speed = 2  # Robot pass speed
Ball_rotbump_speed = 4  # Bump speed when robot turning

# Simulator debug option
Cursor_Position_Show = 0

# Simulator inner parameter
Ball_direction = [1.0, 0.0]
Ball_Position = [0, 0]
Ball_speed = 0
FRAME_FPS = 30
Kickable_distance = 70
mode = 1  # ['start', 'pause', 'set']
Mouse_Position = [0, 0]
Robo_command = ['N1', 'N1', 'N1', 'N1', 'N1', 'N1']
Robo_Direction = [[1.0, 0.0], [1.0, 0.0], [1.0, 0.0], [-1.0, 0.0], [-1.0, 0.0], [-1.0, 0.0]]
Robo_kick_angle_threshold = 60 * math.pi / 180
Robo_Position = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
Robo_rotate_rect = [None, None, None, None, None, None]
Robo_rotate_vert = [None, None, None, None, None, None]
Setup_Index = 0

# The simulation of object is simulated with update frequency @FRAME_FPS
# In practice, the camera has maximum update frequency of 30 fps. You can set this parameter with @CAM_FPS
# Also, the sending frequency in practice will not be 30 fps because it is too frequent for robot to reacat.
# YOu can modify the sending frequency in @CMD_FPS
# If you modify the value, you should modify object speed with appropriate scalar. Implement by yourself.
STRATEGY_UP, SEND_UP = int(FRAME_FPS / CAM_FPS), int(FRAME_FPS / CMD_FPS)


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


def get_strategy():
    """Simulator get strategy"""
    try:
        cmd_a = st_A.strategy()
        cmd_b = st_B.strategy()
    except AttributeError:
        return 0, 0, 0
    return 1, cmd_a, cmd_b


def init_strategy():
    """Initialize strategy"""
    global Robo_Position, Robo_Direction
    try:
        st_A.Initialize()
        st_B.Initialize()
    except AttributeError:
        print("error")


def parse_strategy_field(PK_x, FB_x, FB_y, Penalty_y):
    """Parse field parameter to strategy"""
    try:
        (sideA, sideB) = (1, -1) if Side == 1 else (-1, 1)
        st_A.strategy_update_field(sideA, BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y)
        st_B.strategy_update_field(sideB, BOUNDARY, CENTER, PK_x, FB_x, FB_y, Penalty_y, GA_x, GA_y)
    except AttributeError:
        pass


def return_sent_cmd(send_data, sent):
    """Pass sent command to strategy"""
    try:
        st_A.get_sent_cmd(send_data[0:3], sent)
        st_B.get_sent_cmd(send_data[3:6], sent)
    except AttributeError:
        pass


def upd_strategy_position():
    """Simulator Opject position update"""
    try:
        a_offset = 0
        b_offset = 0
        if Side == 1:
            a_offset = 0
            b_offset = 3
        else:
            a_offset = 3
            b_offset = 0
        st_A.Update_Robo_Info(Robo_Direction[b_offset:b_offset + 3], Robo_Position[b_offset:b_offset + 3],
                              Robo_Position[a_offset:a_offset + 3], Ball_Position)
        st_B.Update_Robo_Info(Robo_Direction[a_offset:a_offset + 3], Robo_Position[a_offset:a_offset + 3],
                              Robo_Position[b_offset:b_offset + 3], Ball_Position)  # 我們是b
    except AttributeError:
        pass
