import math
import time
import copy
import cv2
import enum
# import strategy.constant as CONST
from strategy.com_constant import *
from strategy.constant1 import *
# from strategy.movement import *
from strategy.vec_cal_func import *
import strategy.movement_1 as moveSep

ST_ID = 1
#################################################
#               TESTING VARIABLE                #
#################################################

TEST_KICK = 0           # Used if we want to has specific kick
                        # 0 => None
                        # 1 => Forward
                        # 2 => SIDE
                        # 3 => Backward

#################################################
#             END TESTING VARIABLE              #
#################################################

## Command format  
cmd = [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]] #Robot_Action = [position_x, position_y, facing_x, facing_y, action, renew]
pre_cmd = [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
final_cmd = ['N1','N1','N1']        # Final command given to the system
sent_cmd = ['N1','N1','N1']
action_option = [0,0,0]

#PlaySIDE
SIDE = 1
ATTACK_DIR = [1,0]      # Direction from our goal to their goal

# Robot Role
first = 0
second = 1
nobody = 2
front = 0
back = 1

#Field parameter

ball_pos = [0, 0]

BOUNDARY = [(140, 70), (870, 70), (870, 170), (900, 170), (900, 350), (870, 350), (870, 450), (140, 450), (140, 350), (110, 350), (110, 170), (140, 170)]
CENTER = [0, 0]
PK = [0, 0]
FK = [0, 0]
FB = [0, 0]
PENALTY_AREA = [0, 0]
GOAL_AREA = [0, 0]

#Strategy parameter
robo_dir = [[1.0, 0.0], [1.0, 0.0], [1.0, 0.0]]
robo_pos = [[0,0] , [0,0] , [0,0] , [0,0] , [0,0] , [0,0]]

#State
game_state = 1
move_state = [0,0,0]

#Boundary Parameter
boundary_ourgoalup = [0,0]
boundary_ourgoaldown = [0,0]
boundary_goalup = [0,0]
boundary_goaldown = [0,0]
boundary_left_x = 0
boundary_farleft_x = 0
boundary_right_x = 0
boundary_farright_x = 0
boundary_up_y = 0
boundary_down_y = 0

#Ball Movement
system_count = 0
pre_ball_move, ball_move, init_ball_move = 1, 1, 1
pre_ball_pos, init_ball_pos, original_ball_pos = [[0,0],[0,0],[0,0],[0,0],[0,0]] ,[0,0], [0,0]

#Determine oppo's position
oppo_pos = [[0,0],[0,0],[0,0]] #Forward(oppo0) - Midfielder(oppo1) - keeper(oppo2)

#Determine if robots are on the court
#our robot
team_alive = [0, 0, 0]
#oppo robot
oppo_alive = [0, 0, 0]
#Caculate Angle : The point [ball_pos[0],ball_pos[1]+1] is regarded as 0 degree 
#COUNTER-CLOCKWISE
goalup_angle_adj, goaldown_angle_adj = 0 , 0
front_upPos_angle_adj, front_downPos_angle_adj, back_upPos_angle_adj, back_downPos_angle_adj = 0, 0, 0, 0
front_upPos, front_downPos, back_upPos, back_downPos = [0,0], [0,0], [0,0], [0,0]

#Ball_Goal(robot)
ball_goal = [0,0]

###################################### Position_Adjustment() #####################################################
pos_adjust = 0
adjust_uxy = [0,0]

###################################### strategy_kicker(robot) ####################################################
move_style = [1,1,1] #1:FORWARD MOVE #2:SIDE MOVE
kick_style = [0,0,0] #0:Auto #1:Pass #2:Bump
kicker_state = [0,0,0]
kicker_goal = [0,0]
kicker_avoidPos, kicker_readyPos, kicker_kickPos = [0,0] ,[0,0], [0,0]
avoid_uxy = [0,0]

'''
#ACTION
'N' None (0)
'u' Left forward shoot (1)
'i'	Right forward shoot (2)
'h'	Left side shoot (3)
'j'	Right side shoot (4)	
'b'	Left back shoot	(5)
'n'	Right back shoot (6)	
'o'	Left pass (7)
'p'	Right pass (8)
'f' Left save ball (9)
'g' Right save ball (10)
'y'	Squat save ball (11)
'Y' Forward save ball (12)
'r'	Rest (13)
'R'	Stand up (14)
'z'	Start robot	(15)
'x'	Shut down robot (16)
#MOVE
'w'	Forward (17)	
's'	Backward (18)	
'a'	Move left (19)
'd'	Move right (20)	
'W'	Fast forward (21)	
'A'	Fast move left (22)	
'D'	Fast move right (23)	
'q'	Small Turn left (24)		
'e'	Small Turn right (25)	
'Q'	Turn left (26)	
'E'	Turn right (27)
'''

# A Fake movement to run stretagy without error
def movement(robot):
    global pre_cmd, final_cmd, sent_cmd
    global move_state
    final_cmd[robot], move_state[robot] = moveSep.movement(robot, robo_pos[robot], robo_dir[robot], cmd[robot], pre_cmd[robot], sent_cmd[robot], move_state[robot], ball_pos)
    pre_cmd[robot] = copy.copy(cmd[robot]) #copy data, this line has to be placed at last

def get_final_cmd(ret_cmd):
    global final_cmd
    final_cmd = ret_cmd.copy()

def get_sent_cmd(sentcmd, update):
    global sent_cmd
    if update:
        sent_cmd = sentcmd.copy()

def Initialize():
    global cmd, robo_dir, robo_pos
    cmd = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
    robo_dir = [[1.0, 0.0], [1.0, 0.0], [1.0, 0.0]]
    robo_pos = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
    moveSep.init(ST_ID)

def strategy_update_field(Play_SIDE, boundary, center, PK_x, FK_x, FB_y, Penalty_Y, GA_x, GA_y):
    global SIDE, BOUNDARY, CENTER, PK, FK, FB, PENALTY_AREA, GOAL_AREA
    
    SIDE = Play_SIDE

    BOUNDARY = boundary

    CENTER = center

    PK[x] = PK_x

    FK[x] = FK_x

    FB[x] = FK[x] 
    FB[y] = FB_y

    PENALTY_AREA[x] = PK[x]
    PENALTY_AREA[y] = Penalty_Y

    GOAL_AREA[x] = GA_x
    GOAL_AREA[y] = GA_y

def Update_Robo_Info(Team_D, Team_P, Oppo_P, Ball_P):
    global robo_dir, robo_pos, ball_pos
    Team_P.extend(Oppo_P)
    robo_dir = Team_D
    robo_pos = Team_P
    ball_pos = Ball_P    

def draw_on_simulator(frame):
    
    target = [int(kicker_goal[0]),int(kicker_goal[1])]
    cv2.circle(frame, (target[0],target[1]) , 15 , (0,250,250) , -1, 8, 0)
    cv2.putText(frame, str(target[0]) + ' ' + str(target[1]), (target[0],target[1]), cv2.FONT_HERSHEY_SIMPLEX,1, (0,250,250), 2, cv2.LINE_AA)
    
    target = [int(kicker_avoidPos[0]),int(kicker_avoidPos[1])]
    cv2.circle(frame, (target[0],target[1]) , 5 , (250,0,120) , -1, 8, 0)
    cv2.putText(frame, str(target[0]) + ' ' + str(target[1]), (target[0],target[1]), cv2.FONT_HERSHEY_SIMPLEX,1, (250,0,120), 2, cv2.LINE_AA)

    target = [int(kicker_readyPos[0]),int(kicker_readyPos[1])]
    cv2.circle(frame, (target[0],target[1]) , 5 , (250,60,127) , -1, 8, 0)
    cv2.putText(frame, str(target[0]) + ' ' + str(target[1]), (target[0],target[1]), cv2.FONT_HERSHEY_SIMPLEX,1, (250,60,127), 2, cv2.LINE_AA)
    
    target = [int(kicker_kickPos[0]),int(kicker_kickPos[1])]
    cv2.circle(frame, (target[0],target[1]) , 5 , (35,120,250) , -1, 8, 0)
    cv2.putText(frame, str(target[0]) + ' ' + str(target[1]), (target[0],target[1]), cv2.FONT_HERSHEY_SIMPLEX,1, (35,120,250), 2, cv2.LINE_AA)
    
    return frame

def fitness():

    ball_dir = [0, 0, 0]
    ball_dis = [0, 0, 0]
    angle_to_attack_dir = [0, 0, 0]
    fitness = [0, 0, 0]

    for rob in range(3):
        #ball_dir , length : robot -> ball
        ball_dir[rob] , ball_dis[rob] = vector_param(robo_pos[rob],ball_pos)
        #angle to attack dir : ATTACK_DIR -> robo_ball_uxy
        angle_to_attack_dir[rob] , _ = vector_angle(ATTACK_DIR,ball_dir[rob])

        # Consider the weighting for the distance
        if team_alive[rob] == 1:
            if angle_to_attack_dir[rob] < 90 :
                ball_dis[rob] = ball_dis[rob]
            else:
                ball_dis[rob] = ball_dis[rob] + (2**0.5) * DIAGONAL
        else:
            ball_dis[rob] = 0

    max_len = max(ball_dis[0],ball_dis[1],ball_dis[2])
    #print("len:",Team0_len,Team1_len,Team2_len,max_len)

    #Compute fit
    for rob in range(3):
        if team_alive[rob] == 1:    
            fitness[rob] = int(100 - 75 * ball_dis[rob] / max_len)
        else:
            fitness[rob] = 0
   
    # print("fitness:", fitness)
    return fitness

def decide_shoot_goal(robot,mode): #Oppo robots block goal? -1:none, 0:oppo0, 1:oppo1, 2:oppo2, 10:oppo1&0, 20:oppo2&0, 21:oppo2&1, 210:oppo2&1&0
    global action_option
    global kick_style
    
    print('robot:',robot)
    print('mode:',mode)

    #LOCAL VARIABLE    
    shoot_goal = [0,0]
    greedy = 0
    
    #ALGORITHM
    if mode == -1:

        goal_x = (boundary_goalup[0] + boundary_goaldown[0]) * 0.5
        if  ball_pos[1]  < CENTER[1]:
            goal_y = boundary_goalup[1] * 2/3 + boundary_goaldown[1] * 1/3
        else:
            goal_y = boundary_goalup[1] * 1/3 + boundary_goaldown[1] * 2/3

        if CENTER[1] - GOAL_AREA[y] <= ball_pos[1] <= CENTER[1] + GOAL_AREA[y]:
            goalmid_angle = (goalup_angle_adj + goaldown_angle_adj) * 0.5 #devide angle equally : goalup_angle , goaldown_angle
            ball_goalmid_uxy = rotate_vector([0,1],goalmid_angle)
            shoot_goal = [goal_x , ball_pos[1] + abs(goal_x - ball_pos[0]) * ball_goalmid_uxy[1]]
        else:    
            shoot_goal = [goal_x , goal_y]
        #print('angle zero')
    
    elif mode == 1 or mode == 2 :

        #Synchronize Information
        if mode == 1: #Front
            oppo_upPos_angle = front_upPos_angle_adj
            oppo_downPos_angle = front_downPos_angle_adj
            oppo_upPos = front_upPos
            oppo_downPos = front_downPos
        else: #mode == 2 #Back
            oppo_upPos_angle = back_upPos_angle_adj
            oppo_downPos_angle = back_downPos_angle_adj
            oppo_upPos = back_upPos
            oppo_downPos = back_downPos  
            
        oppo_goalup_angle = goalup_angle_adj - oppo_upPos_angle #angle : goalup_angle to oppo_angle  
        oppo_goaldown_angle = oppo_downPos_angle - goaldown_angle_adj #angle : goaldown_angle to oppo_angle
        
        print("Two angle:",oppo_goalup_angle,oppo_goaldown_angle)

        if oppo_goalup_angle >= oppo_goaldown_angle:  #shoot oppo to goalup
            #Information   
            ball_upPos_uxy , ball_upPos_length = vector_param(ball_pos,oppo_upPos) #uxy , length : ball -> oppo_upPos
            effective_length = ball_upPos_length * math.sin(oppo_goalup_angle * math.pi / 180)
            print("effective_length:",effective_length)
            if effective_length >= GREEDY_RATIO * BALL_RADIUS:
                shoot_goal =  [oppo_upPos[0] * (1-SHOOT_RATIO) + boundary_goalup[0] * SHOOT_RATIO , oppo_upPos[1] * (1-SHOOT_RATIO) + boundary_goalup[1] * SHOOT_RATIO]    
            else: 
                greedy = 1 
            #print('angleup')

        else: #oppo_goalup_angle < oppo_goaldown_angle #shoot oppo to goaldown
            #Information    
            ball_downPos_uxy , ball_downPos_length = vector_param(ball_pos,oppo_downPos) #uxy , length : ball -> oppo_downPos
            effective_length = ball_downPos_length * math.sin(oppo_goaldown_angle * math.pi / 180)
            print("effective_length:",effective_length)
            if effective_length >= GREEDY_RATIO * BALL_RADIUS:   
                shoot_goal =  [oppo_downPos[0] * (1-SHOOT_RATIO) + boundary_goaldown[0] * SHOOT_RATIO , oppo_downPos[1] * (1-SHOOT_RATIO) + boundary_goaldown[1] * SHOOT_RATIO]    
            else:
                greedy = 1                
            #print('angledown')

    else:  #mode == 3
           
        #Synchronize Information        
        if front_upPos_angle_adj >= back_upPos_angle_adj:
            #angle
            oppoup_upPos_angle = front_upPos_angle_adj
            oppoup_downPos_angle = front_downPos_angle_adj
            oppodown_upPos_angle = back_upPos_angle_adj
            oppodown_downPos_angle = back_downPos_angle_adj
            #position
            oppoup_upPos = front_upPos
            oppoup_downPos = front_downPos
            oppodown_upPos = back_upPos
            oppodown_downPos = back_downPos 
        else:
            #angle
            oppoup_upPos_angle = back_upPos_angle_adj
            oppoup_downPos_angle = back_downPos_angle_adj
            oppodown_upPos_angle = front_upPos_angle_adj
            oppodown_downPos_angle = front_downPos_angle_adj
            #position
            oppoup_upPos = back_upPos
            oppoup_downPos = back_downPos
            oppodown_upPos = front_upPos
            oppodown_downPos = front_downPos

        #Three angle
        angle1 = goalup_angle_adj - oppoup_upPos_angle
        angle2 = oppoup_downPos_angle - oppodown_upPos_angle
        angle3 = oppodown_downPos_angle - goaldown_angle_adj
        max_angle = max(angle1,angle2,angle3)
        print("Three angle:",angle1,angle2,angle3)

        if angle1 == max_angle: #angle1 is the largest angle
            #Information    
            ball_oppoup_upPos_uxy , ball_oppoup_upPos_length = vector_param(ball_pos,oppoup_upPos) #uxy , length : ball -> oppoup_upPos
            effective_length = ball_oppoup_upPos_length * math.sin(angle1 * math.pi / 180)
            print("effective_length:",effective_length)
            if effective_length >= GREEDY_RATIO * BALL_RADIUS:          
                shoot_goal =  [oppoup_upPos[0] * (1-SHOOT_RATIO) + boundary_goalup[0] * SHOOT_RATIO , oppoup_upPos[1] * (1-SHOOT_RATIO) + boundary_goalup[1] * SHOOT_RATIO]    
            else:
                greedy = 1
            #print('angle1')

        elif angle3 == max_angle: #angle3 is the largest angle
            #Information    
            ball_oppodown_downPos_uxy , ball_oppodown_downPos_length = vector_param(ball_pos,oppodown_downPos) #uxy , length : ball -> oppodown_downPos
            effective_length = ball_oppodown_downPos_length * math.sin(angle3 * math.pi / 180)
            print("effective_length:",effective_length)
            if effective_length >= GREEDY_RATIO * BALL_RADIUS:
                shoot_goal =  [oppodown_downPos[0] * (1-SHOOT_RATIO) + boundary_goaldown[0] * SHOOT_RATIO , oppodown_downPos[1] * (1-SHOOT_RATIO) + boundary_goaldown[1] * SHOOT_RATIO]
            else:
                greedy = 1
            #print('angle3') 
             
        else: #angle2 is the largest
            #Information
            if ball_pos[1] < oppo_pos[front][1]:
                ball_oppofront_uxy , ball_oppofront_length = vector_param(ball_pos,front_upPos) #uxy , length : ball -> front_upPos
            else:
                ball_oppofront_uxy , ball_oppofront_length = vector_param(ball_pos,front_downPos) #uxy , length : ball -> front_downPos
            effective_length = ball_oppofront_length * math.sin(angle2 * math.pi / 180)
            print("effective_length:",effective_length)
            if effective_length >= GREEDY_RATIO * BALL_RADIUS:
                shoot_goal =  [oppoup_downPos[0] * 0.5 + oppodown_upPos[0] * 0.5 , oppoup_downPos[1] * 0.5 + oppodown_upPos[1] * 0.5]
            else:
                greedy = 1
            #print('angle2')            
        
    #Greedy Solution
    if greedy == 1:
        #Goal
        if SIDE == 1:
            if ball_pos[1] < oppo_pos[front][1]:
                shoot_goal = [boundary_right_x,boundary_up_y]
            else:
                shoot_goal = [boundary_right_x,boundary_down_y]
        else:
            if ball_pos[1] < oppo_pos[front][1]:
                shoot_goal = [boundary_left_x,boundary_up_y]
            else:
                shoot_goal = [boundary_left_x,boundary_down_y]
        #Action
        kick_style[robot] = 2 #Bump

    return shoot_goal #return value to Positioning()

def Positioning(robot):
    global cmd
     
    #LOCAL VARIABLE
    pos_goal = [0,0] 
    
    #ALGORITHM    
    if goaldown_angle_adj <= front_upPos_angle_adj <= goalup_angle_adj or \
        goaldown_angle_adj <= front_downPos_angle_adj <= goalup_angle_adj: #FRONT BLOCK
        if goaldown_angle_adj <= back_upPos_angle_adj <= goalup_angle_adj or \
            goaldown_angle_adj <= back_downPos_angle_adj <= goalup_angle_adj: #BACK BLOCK                  
            pos_goal = decide_shoot_goal(robot,3)                    
        else: #BACK NO BLOCK
            pos_goal = decide_shoot_goal(robot,1)                    
    else: #FRONT NO BLOCK
        if goaldown_angle_adj <= back_upPos_angle_adj <= goalup_angle_adj or \
            goaldown_angle_adj <= back_downPos_angle_adj <= goalup_angle_adj: #BACK BLOCK
            pos_goal = decide_shoot_goal(robot,2)                 
        else: #BACK NO BLOCK
            pos_goal = decide_shoot_goal(robot,-1) 

    return pos_goal

def BallGoal(robot):
    global cmd
    global ball_goal
    global action_option
    global kick_style

    #Default Action
    kick_style[robot] = 2 #Bump

    #Default Goal
    ball_goal[0] = CENTER[0] + SIDE * FK[x]
    if oppo_pos[front][1] < CENTER[1]:
        ball_goal[1] = CENTER[1] + FB[y]
    else:
        ball_goal[1] = CENTER[1] - FB[y]        

    #ALGORITHM
    
    if robot == first:       
        pass

    else: #robot == second
        
        if SIDE == 1:            
            if ball_pos[0] < CENTER[0]:
                pass
            else:
                #Reset
                kick_style[robot] = 0
                #Goal
                ball_goal = Positioning(robot)                              

        else: #SIDE == -1            
            if CENTER[0] < ball_pos[0]:
                pass
            else:
                #Reset
                kick_style[robot] = 0
                #Goal
                ball_goal = Positioning(robot)                        

    return ball_goal   

def vertical_length(position):
    
    ################################## COMPUTE VERTICAL LENGTH ##################################
    
    #LOCAL VARIABLE
    point = 0
    min_length = 1000
    uxy = [0,0]    
    
    #SET
    POINT_SET = [11,0,1,5,6,7]
    
    #DETERMINE POS
    Pos = copy.copy(position)

    #ALGORITHM
    for i in range(len(POINT_SET)):
        num = POINT_SET[i]
        if (BOUNDARY[num%12][0] <= Pos[0] <= BOUNDARY[(num+1)%12][0]) or \
            (BOUNDARY[num%12][0] > Pos[0] > BOUNDARY[(num+1)%12][0]) or \
            (BOUNDARY[num%12][1] <= Pos[1] <= BOUNDARY[(num+1)%12][1]) or \
            (BOUNDARY[num%12][1] > Pos[1] > BOUNDARY[(num+1)%12][1]):
            #Determine verti_length
            B1_B2_uxy , B1_B2_length  = vector_param(BOUNDARY[num%12],BOUNDARY[(num+1)%12])
            B1_Pos_uxy , B1_Pos_length  = vector_param(BOUNDARY[num%12],Pos)
            verti_angle , verti_angle_full = vector_angle(B1_B2_uxy,B1_Pos_uxy)
            verti_length = B1_Pos_length * math.sin(verti_angle * math.pi / 180)
            #Determine min_length
            if verti_length < min_length:   
                point = num                 
                min_length = verti_length
                uxy = B1_B2_uxy 

    #Field Check
    if boundary_farleft_x <= Pos[0] <= boundary_farright_x and boundary_up_y <= Pos[1] <= boundary_down_y: #on the field
        pass
    else: #off the field
        min_length = -1                                      

    #print("point:",point,"min_length:",min_length,"uxy:",uxy)

    return point,min_length,uxy

def Position_Adjustment(robot):
    global pos_adjust
    global adjust_uxy

    ################################## READY POS ##################################
    
    #DETERMINE READY POS
    goal = BallGoal(robot)
    goal_uxy , goal_length = vector_param(ball_pos,goal)
    readyPos = [ int(ball_pos[0] + (READY_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS) * -goal_uxy[0]) , int(ball_pos[1] + (READY_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS) * -goal_uxy[1]) ]    

    #print("READY_POS_RATIO",READY_POS_RATIO,"readyPos:",readyPos)

    #COMPUTE VERTICAL LENGTH
    ready_point , ready_length , ready_uxy = vertical_length(readyPos)
    #print("ready_point:",ready_point,"ready_length:",ready_length,"ready_uxy:",ready_uxy)

    #CHANGE POS_ADJUST
    if ready_length < 0.7 * DEPTH:
        pos_adjust = 1
    else:
        pos_adjust = 0

    ################################## BALL POS ##################################

    if pos_adjust == 1:

        #COMPUTE VERTICAL LENGTH
        ball_point , ball_length , ball_uxy = vertical_length(ball_pos)
        #print("ball_point:",ball_point,"ball_length:",ball_length,"ball_uxy:",ball_uxy)

        #CHANGE POS_ADJUST
        if ball_length < 0.7 * DEPTH:
            pos_adjust = 2
            #COMPUTE ADJUST_UXY
            adjust_angle = 45 #Adjustable
            if 5 <= ball_point <= 7: #point : 5, 6, 7
                adjust_uxy = rotate_vector(ball_uxy,(-90 - SIDE * adjust_angle))               
            else: #point : 11, 0, 1
                adjust_uxy = rotate_vector(ball_uxy,(-90 + SIDE * adjust_angle))
        else:
            pos_adjust = 1

    #print("pos_adjust:",pos_adjust)

def strategy_kicker(robot):
    global cmd
    global action_option
    global move_style, kick_style
    global kicker_state
    global kicker_goal 
    global kicker_readyPos, kicker_kickPos, kicker_avoidPos
    global avoid_uxy
   
    ###### GOAL INFORMATION ######

    ########################################### RESET STATE ###########################################

    #Information
    kicker_ball_uxy , kicker_ball_length = vector_param(robo_pos[robot],ball_pos) #uxy , length : robot -> ball

    if kicker_state[robot] == 3:
        if kicker_ball_length < RESET_STATE_RATIO * DIAGONAL:
            pass
        else:
            kicker_state[robot] = 0            

    ########################################### GOAL INFORMATION ###########################################
    
    #Position Adjust
    Position_Adjustment(robot)
    
    #Determine goal
    if robot == first:
        kicker_goal = BallGoal(first)

    else: #robot == second
        if pos_adjust == 0:
            kicker_goal = BallGoal(second)

        elif pos_adjust == 1:
            if SIDE == 1:
                kicker_goal = [boundary_right_x,ball_pos[1]]
            else:
                kicker_goal = [boundary_left_x,ball_pos[1]]

        else: #pos_adjust == 2
            kicker_goal = [ball_pos[0] + 2 * BALL_RADIUS * adjust_uxy[0] , ball_pos[1] + 2 * BALL_RADIUS * adjust_uxy[1]]
                
    #Information    
    goal_uxy , goal_length = vector_param(ball_pos,kicker_goal) #uxy , length : ball -> goal   
    goal_kicker_angle , goal_kicker_angle_full = full_angle(kicker_goal,robo_pos[robot],ball_pos) #angle : goal -> robot (Compared to ball)     
    goal_robodir_angle , goal_robodir_angle_full = vector_angle(goal_uxy,robo_dir[robot]) #angle : goal_uxy -> robo_dir
    
    #Compute avoid_uxy
    if kicker_state[robot] < 2:
        if goal_kicker_angle_full < 180:
            avoid_uxy = rotate_vector(goal_uxy,-90)
        else:
            avoid_uxy = rotate_vector(goal_uxy,90)

    #Compute side_uxy
    if 0 < goal_robodir_angle_full < 180:
        side_uxy = rotate_vector(goal_uxy,-90)       
    else:
        side_uxy = rotate_vector(goal_uxy,90)    

    ############################################# DETERMINE STYLE #############################################
    
    #MOVE STYLE   
    if ball_move == 1 or init_ball_move == 1:
        if kicker_ball_length >= MOVE_STYLE_RATIO * STEP[BIG][LEFT]: #2:Side Move
            move_style[robot] = 2
        else: #1:Forward Move
            move_style[robot] = 1
    #print("move_style[robot]:",move_style[robot])

    ############################################# DETERMINE ACTION ############################################# 

    #KICK STYLE
    #0:Auto #1:Pass #2:Bump 
    
    if kick_style[robot] == 0: #0:Auto
        if kicker_state[robot] < 3:    
            if goal_robodir_angle <= 30: #Forward Shoot
                action_option[robot] = 1
            elif 30 < goal_robodir_angle <= 180: #Side Shoot
                action_option[robot] = 3
            else: #Back Shoot
                action_option[robot] = 5
    elif kick_style[robot] == 1: #1:Pass 
        action_option[robot] = 7
    else: #2:Bump
        action_option[robot] = 0            

    #TEST SPECIFIC KICK
    if TEST_KICK != 0:
        action_option[robot] = (TEST_KICK * 2) - 1   
    
    ####################################### DETERMINE LEFT OR RIGHT ACTION ###################################### 
    
    if 1 <= action_option[robot] <= 8:

        #LEFT SIDE
        left_uxy = rotate_vector(robo_dir[robot],-90)
        kicker_leftPos = [ robo_pos[robot][0] + FOOT_OFFSET * left_uxy[0] , robo_pos[robot][1] + FOOT_OFFSET * left_uxy[1] ]
        leftPos_ball_uxy , leftPos_ball_length = vector_param(kicker_leftPos,ball_pos) #uxy , length : leftPos -> ball
        
        #RIGHT SIDE
        right_uxy = rotate_vector(robo_dir[robot],90)
        kicker_rightPos = [ robo_pos[robot][0] + FOOT_OFFSET * right_uxy[0] , robo_pos[robot][1] + FOOT_OFFSET * right_uxy[1] ]
        rightPos_ball_uxy , rightPos_ball_length = vector_param(kicker_rightPos,ball_pos) #uxy , length : rightPos -> ball
        
        #DETERMINE LEFT OR RIGHT SIDE
        if leftPos_ball_length <= rightPos_ball_length: #Left
            if (action_option[robot] == 1) or (action_option[robot] == 2): #Forward Shoot
                action_option[robot] = 1
            elif (action_option[robot] == 3) or (action_option[robot] == 4): #Side Shoot
                action_option[robot] = 3
            elif (action_option[robot] == 5) or (action_option[robot] == 6): #Back Shoot
                action_option[robot] = 5
            elif (action_option[robot] == 7) or (action_option[robot] == 8): #Pass
                action_option[robot] = 7
            else:
                pass
        else: #Right
            if (action_option[robot] == 1) or (action_option[robot] == 2): #Forward Shoot
                action_option[robot] = 2
            elif (action_option[robot] == 3) or (action_option[robot] == 4): #Side Shoot
                action_option[robot] = 4
            elif (action_option[robot] == 5) or (action_option[robot] == 6): #Back Shoot
                action_option[robot] = 6
            elif (action_option[robot] == 7) or (action_option[robot] == 8): #Pass
                action_option[robot] = 8
            else:
                pass

        #print("left,right:",leftPos_ball_length,rightPos_ball_length)
        #print('action_option:', action_option[robot])
    
    ######################################## POSITION INFORMATION ########################################
    
    #Determine kickPos_dis
    if (action_option[robot] == 3) or (action_option[robot] == 4): #Side Shoot
        kickPos_dis = CENTER_OFFSET[LEFT] * WIDTH + KICK_RATIO[LEFT] * BALL_RADIUS + 1 * BALL_RADIUS
    elif (action_option[robot] == 5) or (action_option[robot] == 6): #Back Shoot
        kickPos_dis = CENTER_OFFSET[BACK] * DEPTH + KICK_RATIO[BACK] * BALL_RADIUS + 1 * BALL_RADIUS
    elif (action_option[robot] == 7) or (action_option[robot] == 8): #Pass
        kickPos_dis = CENTER_OFFSET[FOR] * DEPTH + KICK_RATIO[PASS] * BALL_RADIUS + 1 * BALL_RADIUS
    else: #Forward Shoot
        kickPos_dis = CENTER_OFFSET[FOR] * DEPTH + KICK_RATIO[FOR]* BALL_RADIUS + 1 * BALL_RADIUS

    #Determine kicker_thres
    if (action_option[robot] == 3) or (action_option[robot] == 4) or (action_option[robot] == 0): #Side Shoot or Bump
        kicker_thres = 0
    else:
        kicker_thres = 11#(0.185 * WIDTH) - STEP[SMALL][LEFT]
    
    #Compute avoidPos(90 degree)
    kicker_avoidPos = [ int(ball_pos[0] + (AVOID_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS) * avoid_uxy[0]) , int(ball_pos[1] + (AVOID_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS) * avoid_uxy[1]) ]        
    #Boundary Adjustment : avoidPos 
    if (kicker_avoidPos[1] <= boundary_up_y + 0.5 * DIAGONAL) or (kicker_avoidPos[1] >= boundary_down_y - 0.5 * DIAGONAL):
        if kicker_avoidPos[1] <= boundary_up_y + 0.5 * DIAGONAL:
            kicker_avoidPos = [ int(ball_pos[0]) , int(ball_pos[1] + (AVOID_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS)) ]    
        elif kicker_avoidPos[1] >= boundary_down_y - 0.5 * DIAGONAL:
            kicker_avoidPos = [ int(ball_pos[0]) , int(ball_pos[1] - (AVOID_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS)) ]    
        else:
            pass

    #Compute readyPos(180 degree)
    kicker_readyPos = [ int(ball_pos[0] + (READY_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS) * -goal_uxy[0]) , int(ball_pos[1] + (READY_POS_RATIO * DIAGONAL + 1 * BALL_RADIUS) * -goal_uxy[1]) ]    

    #Compute kickPos(180 degree)
    kickPos = [ int(ball_pos[0] + kickPos_dis * -goal_uxy[0]) , int(ball_pos[1] + kickPos_dis * -goal_uxy[1]) ]  
    kicker_kickPos = [ int( kickPos[0] + kicker_thres * avoid_uxy[0] ) , int(kickPos[1] + kicker_thres * avoid_uxy[1]) ]

    #Position Distance
    #uxy , length : robot -> position
    kicker_avoidPos_uxy , kicker_avoidPos_length = vector_param(robo_pos[robot],kicker_avoidPos)  
    kicker_readyPos_uxy , kicker_readyPos_length = vector_param(robo_pos[robot],kicker_readyPos)  
    kicker_kickPos_uxy , kicker_kickPos_length = vector_param(robo_pos[robot],kicker_kickPos)
    
    ################################################## MOVEMENT & ACTION  ##################################################
    
    if (goal_kicker_angle <= 90) and (kicker_state[robot] == 0): ###### avoidPos #######
        if kicker_avoidPos_length < MOVING_DIS_RATIO[BIG] * BALL_RADIUS:
            kicker_state[robot] = 1
        else:
            cmd[robot] = [ kicker_avoidPos[0] , kicker_avoidPos[1] , 0 , 0 , 0 , 6 ]
                            
    else:        
        if kicker_state[robot] == 0:
            if goal_kicker_angle > 150: #kickPos
                kicker_state[robot] = 2
            else: #readyPos
                kicker_state[robot] = 1
        
        elif kicker_state[robot] == 1: ###### readyPos ######
            if kicker_readyPos_length < MOVING_DIS_RATIO[BIG] * BALL_RADIUS or goal_kicker_angle > 150:
                kicker_state[robot] = 2
            else:
                cmd[robot] = [ kicker_readyPos[0] , kicker_readyPos[1] , 0 , 0 , 0 , 6 ]

        elif kicker_state[robot] == 2: ###### kickPos ######
            if kicker_kickPos_length < MOVING_DIS_RATIO[BIG] * BALL_RADIUS:
                kicker_state[robot] = 3
            else:
                if move_style[robot] == 1: #Forward Move
                    cmd[robot] = [ kicker_kickPos[0] , kicker_kickPos[1] , 0 , 0 , 0 , 1 ] 
                else: #Side Move
                    cmd[robot] = [ kicker_kickPos[0] , kicker_kickPos[1] , 0 , 0 , 0 , 6 ]

        else: #kicker_state[robot] == 3 #Action #kickPos
            if action_option[robot] == 0: #Bump
                cmd[robot] = [ ball_pos[0] , ball_pos[1] , side_uxy[0] , side_uxy[1]  , 0 , 5 ] 
            elif (action_option[robot] == 3) or (action_option[robot] == 4): #Side Kick             
                cmd[robot] = [ kicker_kickPos[0] , kicker_kickPos[1] , side_uxy[0] , side_uxy[1] , action_option[robot] , 5 ]    
            elif (action_option[robot] == 5) or (action_option[robot] == 6): #Back Kick
                cmd[robot] = [ kicker_kickPos[0] , kicker_kickPos[1] , -goal_uxy[0] , -goal_uxy[1] , action_option[robot] , 3 ]    
            else: #Front Kick
                cmd[robot] = [ kicker_kickPos[0] , kicker_kickPos[1] , goal_uxy[0] , goal_uxy[1] , action_option[robot] , 3 ]           

def Strategy_First():
    global cmd
    global game_state

    if game_state == 1:
        strategy_kicker(first)
        
    elif game_state == 2:
        first_Pos = [0,0]
        #y-axis        
        if robo_pos[first][1] <= robo_pos[second][1]:
            first_Pos[1] = boundary_up_y + DIAGONAL
        else:
            first_Pos[1] = boundary_down_y - DIAGONAL
        #x-axis       
        if SIDE == 1:
            first_Pos[0] = boundary_left_x + DIAGONAL
        else:
            first_Pos[0] = boundary_right_x - DIAGONAL

        first_Pos_uxy , first_Pos_length = vector_param(robo_pos[first],first_Pos) #uxy , length : robo -> first_Pos

        #Renew cmd 
        if first_Pos_length >= DIAGONAL:
            cmd[first] = [first_Pos[0],first_Pos[1],0,0,0,6] 
        else:
            game_state = 3

    else: #game_state == 3
        cmd[first] = [0,0,0,0,16,7]  #Shut down

def Strategy_Second():
    global cmd

    if game_state == 1:
        cmd[second] = [0,0,0,0,13,7] #Rest        
    else: #game_state >= 2
        strategy_kicker(second)   

def court():
    global ATTACK_DIR
    global boundary_ourgoalup, boundary_ourgoaldown, boundary_goalup , boundary_goaldown, boundary_left_x, boundary_farleft_x, boundary_right_x, boundary_farright_x, boundary_up_y, boundary_down_y
    global oppo_pos
    global team_alive, oppo_alive
    global goalup_angle_adj, goaldown_angle_adj
    global front_upPos_angle_adj, front_downPos_angle_adj, back_upPos_angle_adj, back_downPos_angle_adj
    global front_upPos, front_downPos, back_upPos, back_downPos
    
    #Boundary Parameter
    if SIDE == 1:
        ATTACK_DIR = [1,0]
        boundary_ourgoalup = BOUNDARY[11]
        boundary_ourgoaldown = BOUNDARY[8]
        boundary_goalup = BOUNDARY[2]
        boundary_goaldown = BOUNDARY[5]
    else: 
        ATTACK_DIR = [-1,0]
        boundary_ourgoalup = BOUNDARY[2]
        boundary_ourgoaldown = BOUNDARY[5]
        boundary_goalup = BOUNDARY[11]
        boundary_goaldown = BOUNDARY[8]

    boundary_farleft_x = (BOUNDARY[9][0]+BOUNDARY[10][0])*0.5
    boundary_farright_x = (BOUNDARY[3][0]+BOUNDARY[4][0])*0.5
    boundary_left_x = (BOUNDARY[0][0]+BOUNDARY[7][0])*0.5
    boundary_right_x = (BOUNDARY[1][0]+BOUNDARY[6][0])*0.5
    boundary_up_y = (BOUNDARY[0][1]+BOUNDARY[1][1])*0.5
    boundary_down_y = (BOUNDARY[6][1]+BOUNDARY[7][1])*0.5
    
    #oppo_pos:[Forward(oppo0),Midfielder(oppo1),keeper(oppo2none)]
    oppo_pos = [robo_pos[3].copy(), robo_pos[4].copy(), robo_pos[5].copy()]
    if SIDE == 1:        
        oppo_pos.sort(key=lambda x:x[0]) 
    else: 
        oppo_pos.sort(key=lambda x:x[0]) 
        oppo_pos[0] , oppo_pos[2] = oppo_pos[2] , oppo_pos[0]

    #Are six robots on the field? #1:on the field #0:off the field
    #our team
    if boundary_farleft_x <= robo_pos[0][0] <= boundary_farright_x and boundary_up_y <= robo_pos[0][1] <= boundary_down_y: 
        team_alive[0] = 1 
    else: 
        team_alive[0] = 0
    if boundary_farleft_x <= robo_pos[1][0] <= boundary_farright_x and boundary_up_y <= robo_pos[1][1] <= boundary_down_y:
        team_alive[1] = 1
    else:
        team_alive[1] = 0
    if boundary_farleft_x <= robo_pos[2][0] <= boundary_farright_x and boundary_up_y <= robo_pos[2][1] <= boundary_down_y:
        team_alive[2] = 1
    else:
        team_alive[2] = 0
    #oppo team
    if boundary_farleft_x <= oppo_pos[0][0] <= boundary_farright_x and boundary_up_y <= oppo_pos[0][1] <= boundary_down_y:
        oppo_alive[0] = 1 
    else:
        oppo_alive[0] = 0
    if boundary_farleft_x <= oppo_pos[1][0] <= boundary_farright_x and boundary_up_y <= oppo_pos[1][1] <= boundary_down_y:
        oppo_alive[1] = 1
    else:
        oppo_alive[1] = 0
    if boundary_farleft_x <= oppo_pos[2][0] <= boundary_farright_x and boundary_up_y <= oppo_pos[2][1] <= boundary_down_y:
        oppo_alive[2] = 1
    else:
        oppo_alive[2] = 0

    #Comupute front , oppodown 
    front_upPos = [oppo_pos[front][0] , oppo_pos[front][1] - 1/3 * WIDTH]
    front_downPos = [oppo_pos[front][0] , oppo_pos[front][1] + 1/3 * WIDTH]       
    back_upPos = [oppo_pos[back][0] , oppo_pos[back][1] - 1/3 * WIDTH]
    back_downPos = [oppo_pos[back][0] , oppo_pos[back][1] + 1/3 * WIDTH]

    #Caculate Angle : The point [ball_pos[0],ball_pos[1]+1] is regarded as 0 degree 
    #COUNTER-CLOCKWISE
    goalup_angle , goalup_angle_full = full_angle([ball_pos[0],ball_pos[1]+1],boundary_goalup,ball_pos)
    goaldown_angle , goaldown_angle_full = full_angle([ball_pos[0],ball_pos[1]+1],boundary_goaldown,ball_pos)
    front_upPos_angle , front_upPos_angle_full = full_angle([ball_pos[0],ball_pos[1]+1],front_upPos,ball_pos)
    front_downPos_angle ,front_downPos_angle_full = full_angle([ball_pos[0],ball_pos[1]+1],front_downPos,ball_pos)
    back_upPos_angle , back_upPos_angle_full = full_angle([ball_pos[0],ball_pos[1]+1],back_upPos,ball_pos)
    back_downPos_angle ,back_downPos_angle_full = full_angle([ball_pos[0],ball_pos[1]+1],back_downPos,ball_pos) 

    #Angle Adjustment
    if SIDE == 1:
        goalup_angle_adj = goalup_angle_full
        goaldown_angle_adj = goaldown_angle_full
        front_upPos_angle_adj = front_upPos_angle_full
        front_downPos_angle_adj = front_downPos_angle_full
        back_upPos_angle_adj = back_upPos_angle_full
        back_downPos_angle_adj = back_downPos_angle_full
    else:
        goalup_angle_adj = abs(360 - goalup_angle_full)
        goaldown_angle_adj = abs(360 - goaldown_angle_full)
        front_upPos_angle_adj = abs(360 - front_upPos_angle_full)
        front_downPos_angle_adj = abs(360 - front_downPos_angle_full)
        back_upPos_angle_adj = abs(360 - back_upPos_angle_full)
        back_downPos_angle_adj = abs(360 - back_downPos_angle_full)

def strategy():
    global cmd, final_cmd
    global first, second, nobody, front, back
    global system_count, ball_move, pre_ball_move, pre_ball_pos, original_ball_pos, init_ball_pos, init_ball_move
    global move_state, kicker_state
    global game_state
    global kick_style, action_option

    ########################################### Ball Movement ###########################################

    #Ball Move
    preBall_Ball_uxy , preBall_Ball_length = vector_param(pre_ball_pos[system_count%5],ball_pos)
    if preBall_Ball_length >= BALL_MOVE_THRES:
        ball_move = 1
    else: 
        ball_move = 0
    #print('ball_move:',ball_move)

    #Initial Ball Move
    if (pre_ball_move == 1 and ball_move == 0) or init_ball_move == 1 :
        init_ball_pos = copy.copy(ball_pos)

    init_Ball_uxy , init_Ball_length = vector_param(init_ball_pos, ball_pos)
    if init_Ball_length >= BALL_FROM_INIT_THRES:
        init_ball_move = 1
    else:
        init_ball_move = 0
    #print("init_ball_move",init_ball_move)

    #Reset All
    if ball_move == 1 or init_ball_move == 1:
        move_state = [0,0,0]
        kicker_state = [0,0,0]
        kick_style = [0,0,0]
        action_option = [0,0,0]

    #Change Game State 
    if system_count == 0:
        original_ball_pos = copy.copy(ball_pos)
    if game_state == 1:  
        ori_Ball_uxy , ori_Ball_length = vector_param(original_ball_pos,ball_pos)
        if ori_Ball_length >= FIRST_KICK_THRES:
            game_state = 2  
  
    ########################################### Renew Court Information ###########################################

    court()

    ########################################### Role Assignment ###########################################

    if system_count == 0:
        #our team
        fit = fitness()
        #print(fit[0],fit[1],fit[2])
        max_fit = max(fit[0],fit[1],fit[2])
        if fit[0] == max_fit and team_alive[0] == 1:
            first = 0
            if fit[1] >= fit[2] and team_alive[1] == 1:
                second = 1
                nobody = 2
            else:
                second = 2
                nobody = 1
        elif fit[1] == max_fit and team_alive[1] == 1:
            first = 1
            if fit[2] >= fit[0] and team_alive[2] == 1:
                second = 2
                nobody = 0
            else:
                second = 0
                nobody = 2
        else:
            first = 2
            if fit[0] >= fit[1] and team_alive[0] == 1:
                second = 0
                nobody = 1
            else:
                second = 1
                nobody = 0

        #oppo team
        if oppo_alive[0] == 1:
            front = 0
            if oppo_alive[1] == 1:
                back = 1
            else:
                back = 2
        elif oppo_alive[1] == 1:
            front = 1
            if oppo_alive[2] == 1:
                back = 2
            else:
                back = 0
        else:
            front = 2
            if oppo_alive[0] == 1:
                back = 0
            else:
                back = 1 

    ########################################### Execute Strategy ###########################################
    
    Strategy_First()
    Strategy_Second()
    
    ################################################ Execute Movement ################################################

    final_cmd = ['N1', 'N1', 'N1']
    movement(first)
    movement(second)        

    print("cmd[first]:",cmd[first],"final_cmd[first]:",final_cmd[first])
    print("cmd[second]:",cmd[second],"final_cmd[second]:",final_cmd[second])

    ########################################### Renew Information ###########################################

    pre_ball_pos[system_count%5] = copy.copy(ball_pos) 
    pre_ball_move = copy.copy(ball_move) 
    system_count += 1

    return final_cmd
