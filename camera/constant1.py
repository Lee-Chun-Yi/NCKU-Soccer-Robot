import enum

#################################################
#        NEED TO ADJUST BEFORE STARTED          #
#################################################

BALL_MOVE_THRES = 5 #preBall_Ball_length > BALL_MOVE_THRES : the ball is moving
BALL_FROM_INIT_THRES = 10 #preBall_Ball_length > BALL_FROM_INIT_THRES : the initial ball is moving

# The threshold for the first robot to kick out the ball
FIRST_KICK_THRES = 35 #ori_Ball_length >= FIRST_KICK_THRES : game_state = 2  #6.5cm = 20pixel #10cm = 31

# Robot Size
CENTER_OFFSET = [0.4, 0.4, 0.32, 0.32] #front:5.6cm/14cm=0.4 #back:5.6cm/14cm=0.4 #right:8.7cm/27cm=0.32 #[0.4, 0.4, 0.32, 0.32]

TOLERANCE = 8 #Tolerate Distance By A Step

# In move_amount_adjust(deviation, command = CONST.NO_MOVE)
LE_RI_BIG_RATIO = 1.6
TURN_BIG_RATIO = 2.0

############################# STRATEGY #################################

# In Decide_Goal()
SHOOT_RATIO = 0.6 #close to goalup or goaldown   
GREEDY_RATIO = 4 #BALL_RADIUS

# In Strategy_Kicker(robot)
#STEP[BIG][LEFT]
MOVE_STYLE_RATIO = 1.0 
#DIAGONAL
RESET_STATE_RATIO = 1.2
AVOID_POS_RATIO = 0.7    # Ratio for avoid position, times diagonal
READY_POS_RATIO = 0.8   # Ratio for ready position, times diagonal
#BALL_RADIUS
KICK_RATIO = [1.5, 2.0, 1.0, 1.0, 0]     # [for, back, left, right, pass] shooting ratio
                                            # The shifting from robot center when kicking ball

#################################################
#              END VARIABLE DEFINE              #
#################################################

