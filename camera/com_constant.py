import enum

#################################################
#        NEED TO ADJUST BEFORE STARTED          #
#################################################

BALL_RADIUS = 10    # Ball Size #diameter 6.5cm = 20 pixel

# Robot Size
WIDTH = 83 #27cm = 83pixel
DEPTH = 43 #14cm = 43 pixel
DIAGONAL = ((WIDTH**2)+(DEPTH**2)) ** 0.5

# Robot step size
# [ BIG: for, back, left, right
#   SMALL: for, back, left, right]
#REFERENCE STEP = [ [16, 14, 70, 70],
#                   [ 8,  7,  4,  4] ]
STEP = [ [16, 14, 70, 70],
        [ 7,  7,  4,  4] ]

# One turn size
# [BIG, SMALL]
TURN = [360/18, 360/40]      

# In movement()
MOVING_TURN_RATIO = [1.2, 1.2, 1.5]     # When turning to ready position, use this ratio to decide if is facing it
KICK_TURN_RATIO = [1.0, 1.0, 1.2]       # When turning to attack, use this to decide whether it is good angle to attack
MOVING_DIS_RATIO = [3, 2]               # When moving to a position, use this ratio to decide if arrived
                                        # [BIG, SMALL]

############################# STRATEGY ##################################

FOOT_OFFSET = 0.185 * WIDTH   # The foot offset from robot center #5cm/27cm=0.185


# movement()
CMD_DIS_ERROR_THRES = 5
CMD_DIR_ERROR_THRES = 0.01


#################################################
#              END VARIABLE DEFINE              #
#################################################


#################################################
#                  COMMAND SET                  #
#################################################

CMD_SET = ['N', 'u', 'i', 'h', 'j', 'b', 'n', 'o',\
              'p', 'f', 'g', 'y', 'Y', 'r', 'R', 'z', 'x']

ACTION_SET = ['N1', 'u1', 'i1', 'h1', 'j1', 'b1', 'n1', 'o1',\
              'p1', 'f1', 'g1', 'y1', 'Y1', 'r1', 'R1', 'z1', 'x1',\
              'w1', 's1', 'a1', 'd1', 'W1', 'A1', 'D1',\
              'q1', 'e1', 'Q1', 'E1']

FOR_BACK_CMD = ['W', 'w', 's']

LE_RI_CMD = ['a', 'd', 'A', 'D']

TURNING_CMD = ['Q', 'E', 'q', 'e']

TURN_CMD = [['N', 'N', 'Q', 'E'],
            ['N', 'N', 'q', 'e']]

MOVE_CMD = [ ['W', 'N', 'A', 'D'],
             ['w', 's', 'a', 'd']]

KICK_CMD = [ ['N', 'N', 'u', 'i'],
             ['N', 'N', 'b', 'n'],
             ['h'],
             ['j'],
             ['o','p'] ]

NO_MOVE = 'N'

#################################################
#                      END                      #
#################################################

#################################################
#                   Define Enum                 #
#################################################

#Robo Num
class ROB(enum.IntEnum):
    ROB_0 = 0
    ROB_1 = 1
    ROB_2 = 2
globals().update(ROB.__members__)

#Robo Role
class ROLE(enum.IntEnum):
    PLAYER_1 = 0
    PLAYER_2 = 1
    KEEPER = 2
globals().update(ROLE.__members__)

# set x and y
class set(enum.IntEnum):
    x = 0
    y = 1
globals().update(set.__members__)

#Step Distance
class DIR(enum.IntEnum):
    FOR = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3
globals().update(DIR.__members__)
PASS = 4

class SIZE(enum.IntEnum):
    BIG = 0
    SMALL = 1
globals().update(SIZE.__members__)

#################################################
#                 END DEFINE                    #
#################################################

