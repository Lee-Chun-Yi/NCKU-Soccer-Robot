import time
import copy
import importlib
import com_constant as COM_CONST
import constant1 as CONST
from vec_cal_func import *

#######################
#       CONSTANT      #
#######################
CHANGE_SIDE_TIME = 1.7    # second

####### Record if the robot is resting
robot_is_rest = [0, 0, 0]

####### Check is the left right kick is at time
last_kick_tic = [0, 0, 0]
alter_kick = [False, False, False]

def init(ID):
    global CONST, robot_is_rest, last_kick_tic, alter_kick
    CONST = importlib.import_module("strategy.constant"+str(ID))
    robot_is_rest = [0, 0, 0]
    alter_kick = [False, False, False]
    last_kick_tic[0] = time.time()
    last_kick_tic[1] = time.time()
    last_kick_tic[2] = time.time()
    
'''
Perpuse: Decide the attack command
@param robot: Robot ID
@param deviation: The deviation for the movement
@param command: Check which kind of command it is
@return: No return, but it change the global final_cmd directly
'''
def robot_action(robot, robo_pos, cmd, last_cmd, move_state):
    global robot_is_rest, last_kick_tic, alter_kick
    
    #Data
    destination = [cmd[0],cmd[1]]
    robo_des_uxy , robo_des_length = vector_param(robo_pos,destination) #uxy , length : robot -> destination

    #Renew final_cmd
    decision = COM_CONST.ACTION_SET[cmd[4]]

    # If the robot is told to rest
    if cmd[4] == 13:
        robot_is_rest[robot] = 1

    if decision[0] in COM_CONST.CMD_SET[1:8]:
        # if robot not kicking
        if not last_cmd[0] in COM_CONST.CMD_SET[1:8]:
            alter_kick[robot] = False
            last_kick_tic[robot] = time.time()

        tmp = time.time() - last_kick_tic[robot]
        # print("Time: ", tmp)
        if tmp >= CHANGE_SIDE_TIME:
            alter_kick[robot] = not alter_kick[robot]
            last_kick_tic[robot] = time.time()

        if alter_kick[robot] == 1:
            if decision == 'u1':
                decision = 'i1'
            elif decision == 'i1':
                decision = 'u1'        
            elif decision == 'b1':
                decision = 'n1'
            elif decision == 'n1':
                decision = 'b1'
            elif decision == 'o1':
                decision = 'p1'
            elif decision == 'p1':
                decision = 'o1'
            else:
                pass
        pass
    else:
        alter_kick[robot] = 0
        last_kick_tic[robot] = time.time()


    # if last_cmd != decision:
    #     last_kick_tic[robot] = time.time()
    # elif last_cmd == decision and (time.time() - last_kick_tic[robot] >= CHANGE_SIDE_TIME): #robot have already received the cmd   
    #     #Kick Another Foot
    #     if last_cmd == 'u1':
    #         decision = 'i1'
    #     elif last_cmd == 'i1':
    #         decision = 'u1'        
    #     elif last_cmd == 'b1':
    #         decision = 'n1'
    #     elif last_cmd == 'n1':
    #         decision = 'b1'
    #     elif last_cmd == 'o1':
    #         decision = 'p1'
    #     elif last_cmd == 'p1':
    #         decision = 'o1'
    #     else:
    #         pass
    #     last_kick_tic[robot] = time.time()
    
    move_state_tmp = move_state
    '''
    #Reset
    if robo_des_length >= CONST.TOLERANCE: 
        move_state_tmp = 0
    '''
    return decision, move_state_tmp

'''
Perpuse: Decide the command amount by using deviation
@param robot: Robot ID
@param deviation: The deviation for the movement
@param command: Check which kind of command it is
@return: No return, but it change the global final_cmd directly
'''
def move_amount_adjust(deviation, command = COM_CONST.NO_MOVE):
    decision = command[0]
    times = 1

    ###### Forward - Back Adjustment ######

    if decision in COM_CONST.FOR_BACK_CMD:
        #change num
        if 0 <= deviation < 0.5 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR]:
            decision = COM_CONST.NO_MOVE
        elif 0.5 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR] <= deviation < 1 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR]:
            times = 1
        elif 1 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR] <= deviation < 2 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR]:
            times = 1
        elif 2 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR] <= deviation < 3 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR]:
            times = 1
        elif 3 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR] <= deviation < 4 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR]:
            times = 2
        elif 4 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR] <= deviation < 5 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.FOR]:
            times = 3
        else:
            times = 4


    ###### Left - Right Adjustment ######
     
    elif decision in COM_CONST.LE_RI_CMD:
        if deviation <= CONST.LE_RI_BIG_RATIO * COM_CONST.STEP[COM_CONST.BIG][COM_CONST.LEFT]:
            #Change Cmd
            if decision == COM_CONST.MOVE_CMD[COM_CONST.BIG][COM_CONST.LEFT]:
                decision = COM_CONST.MOVE_CMD[COM_CONST.SMALL][COM_CONST.LEFT]
            elif decision == COM_CONST.MOVE_CMD[COM_CONST.BIG][COM_CONST.RIGHT]:
                decision = COM_CONST.MOVE_CMD[COM_CONST.SMALL][COM_CONST.RIGHT]
            #Change Num
            if 0 <= deviation < 0.5 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT]:
                decision = COM_CONST.NO_MOVE
            elif 0.5 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT] <= deviation < 1 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT]:
                times = 1
            elif 1 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT] <= deviation < 2 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT]:
                times = 1
            elif 2 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT] <= deviation < 3 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT]:
                times = 1
            elif 3 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT] <= deviation < 4 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT]:
                times = 2
            elif 4 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT] <= deviation < 5 * COM_CONST.STEP[COM_CONST.SMALL][COM_CONST.LEFT]:
                times = 3
            else:
                times = 4 
                     
        else:
            #Change Cmd
            if decision == COM_CONST.MOVE_CMD[COM_CONST.SMALL][COM_CONST.LEFT]:
                decision = COM_CONST.MOVE_CMD[COM_CONST.BIG][COM_CONST.LEFT]
            elif decision == COM_CONST.MOVE_CMD[COM_CONST.SMALL][COM_CONST.RIGHT]:
                decision = COM_CONST.MOVE_CMD[COM_CONST.BIG][COM_CONST.RIGHT]
            #Change Num    
            times = 1
           
    
    ###### Turn Adjustment ######

    elif decision in COM_CONST.TURNING_CMD:
        if deviation <= CONST.TURN_BIG_RATIO * COM_CONST.TURN[COM_CONST.BIG]:
            #Change Cmd
            if decision == COM_CONST.TURN_CMD[COM_CONST.BIG][COM_CONST.LEFT]:
                decision = COM_CONST.TURN_CMD[COM_CONST.SMALL][COM_CONST.LEFT]
            elif decision == COM_CONST.TURN_CMD[COM_CONST.BIG][COM_CONST.RIGHT]:
                decision = COM_CONST.TURN_CMD[COM_CONST.SMALL][COM_CONST.RIGHT]
            #Change Num
            if 0 <= deviation < 0.5 * COM_CONST.TURN[COM_CONST.SMALL]:
                decision = COM_CONST.NO_MOVE
            elif 0.5 * COM_CONST.TURN[COM_CONST.SMALL] <= deviation < 1 * COM_CONST.TURN[COM_CONST.SMALL]:
                times = 1
            elif 1 * COM_CONST.TURN[COM_CONST.SMALL] <= deviation < 2 * COM_CONST.TURN[COM_CONST.SMALL]:
                times = 1
            elif 2 * COM_CONST.TURN[COM_CONST.SMALL] <= deviation < 3 * COM_CONST.TURN[COM_CONST.SMALL]:
                times = 1
            elif 3 * COM_CONST.TURN[COM_CONST.SMALL] <= deviation < 4 * COM_CONST.TURN[COM_CONST.SMALL]:
                times = 2
            elif 4 * COM_CONST.TURN[COM_CONST.SMALL] <= deviation < 5 * COM_CONST.TURN[COM_CONST.SMALL]:
                times = 3
            else:
                times = 4           
        else:
            #Change Cmd
            if decision == COM_CONST.TURN_CMD[COM_CONST.SMALL][COM_CONST.LEFT]:
                decision = COM_CONST.TURN_CMD[COM_CONST.BIG][COM_CONST.LEFT]
            elif decision == COM_CONST.TURN_CMD[COM_CONST.SMALL][COM_CONST.RIGHT]:
                decision = COM_CONST.TURN_CMD[COM_CONST.BIG][COM_CONST.RIGHT]
            #Change Num
            if CONST.TURN_BIG_RATIO * COM_CONST.TURN[COM_CONST.BIG] < deviation <= (CONST.TURN_BIG_RATIO + 1) * COM_CONST.TURN[COM_CONST.BIG]:
                times = 1
            elif (CONST.TURN_BIG_RATIO + 1) * COM_CONST.TURN[COM_CONST.BIG] <= deviation < (CONST.TURN_BIG_RATIO + 2) * COM_CONST.TURN[COM_CONST.BIG]:
                times = 2
            elif (CONST.TURN_BIG_RATIO + 2) * COM_CONST.TURN[COM_CONST.BIG] <= deviation < (CONST.TURN_BIG_RATIO + 3) * COM_CONST.TURN[COM_CONST.BIG]:
                times = 3
            else:
                times = 4           
    
    return decision + str(times)

def nearly_same(cmd, pre_cmd):
    if abs(cmd[0] - pre_cmd[0]) > COM_CONST.CMD_DIS_ERROR_THRES:
        return False
    elif abs(cmd[1] - pre_cmd[1]) > COM_CONST.CMD_DIS_ERROR_THRES:
        return False
    elif abs(cmd[2] - pre_cmd[2]) > COM_CONST.CMD_DIR_ERROR_THRES:
        return False
    elif abs(cmd[3] - pre_cmd[3]) > COM_CONST.CMD_DIR_ERROR_THRES:
        return False
    elif cmd[4] != pre_cmd[4]:
        return False
    elif cmd[5] != pre_cmd[5]:
        return False
    
    return True

def movement(robot, robo_pos, robo_dir, cmd, pre_cmd, sent_cmd, move_state, ball_pos):
    global last_kick_tic, alter_kick

    robot_cmd = 'N1'
    ans_move_state = move_state

    #Data
    destination = [cmd[0],cmd[1]]
    direction = [cmd[2],cmd[3]]
    action = cmd[4]
    identity = cmd[5] #1:all(no direction) 1:striker,supporter(with direction) 3:keeper(with direction)

    #Information
    #uxy , length : robot -> destination
    robot_des_uxy , robot_des_length = vector_param(robo_pos,destination)

    #uxy , length : robot -> ball
    robot_ball_uxy , robot_ball_length = vector_param(robo_pos,ball_pos)

    #Angle : robot_des_uxy -> robo_dir
    des_robot_angle , des_robot_angle_full = vector_angle(robot_des_uxy,robo_dir)
    #print("des_robot_angle , des_robot_angle_full:",des_robot_angle , des_robot_angle_full)

    #Angle : robot_ball_uxy -> robo_dir
    ball_robot_angle , ball_robot_angle_full = vector_angle(robot_ball_uxy,robo_dir)
    #print("ball_robot_angle , ball_robot_angle_full:",ball_robot_angle , ball_robot_angle_full)

    #Angle : direction -> robo_dir
    dir_robot_angle , dir_robot_angle_full = vector_angle(direction,robo_dir)
    #print("dir_robot_angle , dir_robot_angle_full:",dir_robot_angle , dir_robot_angle_full)

    #Angle : direction -> robot_des_uxy
    dir_des_robot_angle , dir_des_robot_angle_full = vector_angle(direction,robot_des_uxy)
    #print("dir_des_robot_angle , dir_des_robot_angle_full:",dir_des_robot_angle , dir_des_robot_angle_full)


    ################################################# ALGORITHM #################################################

    if not nearly_same(cmd, pre_cmd): #change cmd, renew state
        ans_move_state = 0
    
    # pre_cmd = copy.copy(cmd) #copy data, this line has to be placed at last

    if identity == 1: #Move(Forward) -> Action
        if des_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Destination(move)
            if 0 <= des_robot_angle_full <= 180: #Turn right
                robot_cmd = move_amount_adjust(des_robot_angle,'e1')
            else: #Turn left
                robot_cmd = move_amount_adjust(des_robot_angle,'q1') 
        else: #Move
            if robot_des_length >= CONST.TOLERANCE: #Forward
                robot_cmd = move_amount_adjust(robot_des_length,'w1')      
            else: #Action                    
                robot_cmd, ans_move_state = robot_action(robot, robo_pos, cmd, sent_cmd, move_state) 

    elif identity == 2: #Move(Backward) -> Action
        if (180-des_robot_angle) >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Destination(move)
            if 0 <= des_robot_angle_full <= 180: #Turn Left
                robot_cmd = move_amount_adjust((180-des_robot_angle),'q1')
            else: #Turn Right
                robot_cmd = move_amount_adjust((180-des_robot_angle),'e1') 
        else: #Move
            if robot_des_length >= CONST.TOLERANCE: #Backward
                robot_cmd = move_amount_adjust(robot_des_length,'s1')      
            else: #Action                    
                robot_cmd, ans_move_state = robot_action(robot, robo_pos, cmd, sent_cmd, move_state) 
    
    elif identity == 3: #Aim direction -> Move(left-right->forward-backward) -> Aim direction ->　Action
        if ans_move_state == 0:
            if dir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Aim direction(move)
                if 0 <= dir_robot_angle_full <= 180: #Turn right
                    robot_cmd = move_amount_adjust(dir_robot_angle,'e1') 
                else: #Turn left
                    robot_cmd = move_amount_adjust(dir_robot_angle,'q1') 
            else: #Move
                #Comupte leri_length
                if dir_des_robot_angle > 90:
                    leri_angle = 180 - dir_des_robot_angle
                else:
                    leri_angle = dir_des_robot_angle
                leri_length = robot_des_length * math.sin(leri_angle * math.pi / 180)
                #print("3-leri_length:",leri_length,"3-leri_angle:",leri_angle)
                if leri_length >= CONST.TOLERANCE: #Left - Right
                    if 0 <= dir_des_robot_angle_full <= 180: #Move left
                        robot_cmd = move_amount_adjust(leri_length,'a1')
                    else: #Move right
                        robot_cmd = move_amount_adjust(leri_length,'d1')                                       
                else: #Forward - Backward
                    #Comupte forback_length
                    if dir_des_robot_angle > 90:
                        forback_angle = 180 - dir_des_robot_angle
                    else:
                        forback_angle = dir_des_robot_angle
                    forback_length = robot_des_length * math.cos(forback_angle * math.pi / 180)
                    #print("3-forback_length:",forback_length,"3-forback_angle:",forback_angle)
                    if forback_length >= CONST.TOLERANCE: #Forward - Backward
                        if dir_des_robot_angle < 90: #Forward
                            robot_cmd = move_amount_adjust(forback_length,'w1')
                        else: #Backward
                            robot_cmd = move_amount_adjust(forback_length,'s1')
                    else: #Can't move any more                    
                        ans_move_state = 1                        
        else: #ans_move_state == 1
            if dir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Aim direction(action)
                if 0 <= dir_robot_angle_full <= 180: #Turn right
                    robot_cmd = move_amount_adjust(dir_robot_angle,'e1')  
                else: #Turn left
                    robot_cmd = move_amount_adjust(dir_robot_angle,'q1')
            else: #Action
                robot_cmd, ans_move_state = robot_action(robot, robo_pos, cmd, sent_cmd, move_state) 

    elif identity == 4: #Keeper Mode Movement #Aim direction -> Move(left-right->forward-backward) -> Action 
        if dir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.KEEPER]: #Turn to : Aim direction(move)
            if 0 <= dir_robot_angle_full <= 180: #Turn right
                robot_cmd = move_amount_adjust(dir_robot_angle,'e1') 
            else: #Turn left
                robot_cmd = move_amount_adjust(dir_robot_angle,'q1') 
        else: #Move
            #Comupte leri_length
            if dir_des_robot_angle > 90:
                leri_angle = 180 - dir_des_robot_angle
            else:
                leri_angle = dir_des_robot_angle
            leri_length = robot_des_length * math.sin(leri_angle * math.pi / 180)
            #print("4-leri_length:",leri_length,"4-leri_angle:",leri_angle)
            if leri_length >= CONST.TOLERANCE: #Left - Right
                if 0 <= dir_des_robot_angle_full <= 180: #Move left
                    robot_cmd = move_amount_adjust(leri_length,'a1')
                else: #Move right
                    robot_cmd = move_amount_adjust(leri_length,'d1')                                       
            else: #Forward - Backward
                #Comupte forback_length
                if dir_des_robot_angle > 90:
                    forback_angle = 180 - dir_des_robot_angle
                else:
                    forback_angle = dir_des_robot_angle
                forback_length = robot_des_length * math.cos(forback_angle * math.pi / 180)
                #print("4-forback_length:",forback_length,"4-forback_angle:",forback_angle)
                if forback_length >= CONST.TOLERANCE: #Forward - Backward
                    if dir_des_robot_angle < 90: #Forward
                        robot_cmd = move_amount_adjust(forback_length,'w1')
                    else: #Backward
                        robot_cmd = move_amount_adjust(forback_length,'s1')
                else: #Action                    
                    robot_cmd, ans_move_state = robot_action(robot,  robo_pos, cmd, sent_cmd, move_state) 

    elif identity == 5: #Aim direction -> Move(forward-backward->left-right) -> Aim direction ->　Action
        if ans_move_state == 0:
            if dir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Aim direction(move)
                if 0 <= dir_robot_angle_full <= 180: #Turn right
                    robot_cmd = move_amount_adjust(dir_robot_angle,'e1') 
                else: #Turn left
                    robot_cmd = move_amount_adjust(dir_robot_angle,'q1')
            else: #Move
                #Comupte forback_length
                if dir_des_robot_angle > 90:
                    forback_angle = 180 - dir_des_robot_angle
                else:
                    forback_angle = dir_des_robot_angle
                forback_length = robot_des_length * math.cos(forback_angle * math.pi / 180)
                #print("5-forback_length:",forback_length,"5-forback_angle:",forback_angle)
                if forback_length >= CONST.TOLERANCE: #Forward - Backward  
                    if dir_des_robot_angle < 90: #Forward
                        robot_cmd = move_amount_adjust(forback_length,'w1')
                    else: #Backward
                        robot_cmd = move_amount_adjust(forback_length,'s1')
                else:                                    
                    #Comupte leri_length
                    if dir_des_robot_angle > 90:
                        leri_angle = 180 - dir_des_robot_angle
                    else:
                        leri_angle = dir_des_robot_angle
                    leri_length = robot_des_length * math.sin(leri_angle * math.pi / 180)
                    #print("5-leri_length:",leri_length,"5-leri_angle:",leri_angle)
                    if leri_length >= CONST.TOLERANCE: #Left - Right
                        if 0 <= dir_des_robot_angle_full <= 180: #Move left
                            robot_cmd = move_amount_adjust(leri_length,'a1')
                        else: #Move right
                            robot_cmd = move_amount_adjust(leri_length,'d1')                                       
                    else: #Can't move any more 
                        ans_move_state = 1
        else: #ans_move_state == 1
            if dir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Aim direction(action)
                #print("dir_robot_angle:",dir_robot_angle)
                if 0 <= dir_robot_angle_full <= 180: #Turn right
                    robot_cmd = move_amount_adjust(dir_robot_angle,'e1')  
                else: #Turn left
                    robot_cmd = move_amount_adjust(dir_robot_angle,'q1')  
            else: #Action
                #print("5-action")
                robot_cmd, ans_move_state = robot_action(robot,  robo_pos, cmd, sent_cmd, move_state) 

    elif identity == 6: #Move(left or right) -> Action
        #Compute newdir_uxy
        newdir_uxy = [0,0]
        if direction[0] == 1: #Left Side Move
            newdir_uxy = rotate_vector(robot_des_uxy,90)
        elif direction[1] == 1: #Right Side Move
            newdir_uxy = rotate_vector(robot_des_uxy,-90)
        else: #direction[0] == 0 and direction[1] == 0
            if 0 <= des_robot_angle_full <= 180: #Right Side Move
                newdir_uxy = rotate_vector(robot_des_uxy,-90)
            else: #Left Side Move
                newdir_uxy = rotate_vector(robot_des_uxy,90)
        #Angle : newdir_uxy -> robot_Direction[robot]
        newdir_robot_angle , newdir_robot_angle_full = vector_angle(newdir_uxy,robo_dir)

        if newdir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Destination(move)
            if 0 <= newdir_robot_angle_full <= 180: #Turn right
                robot_cmd = move_amount_adjust(newdir_robot_angle,'e1')
            else: #Turn left
                robot_cmd = move_amount_adjust(newdir_robot_angle,'q1') 
        else: #Move
            if robot_des_length >= CONST.TOLERANCE: #Left or Right
                if 0 <= des_robot_angle_full <= 180: #Move Right
                    robot_cmd = move_amount_adjust(robot_des_length,'d1') 
                else: #Move left
                    robot_cmd = move_amount_adjust(robot_des_length,'a1')      
            else: #Action
                robot_cmd, ans_move_state = robot_action(robot,  robo_pos, cmd, sent_cmd, move_state) 
        
    elif identity == 7: #Action
        robot_cmd, ans_move_state = robot_action(robot,  robo_pos, cmd, sent_cmd, move_state)

    elif identity == 8: #(Forward or Side) Move to destination -> (Forward or Side) Move to ball ->Action
        
        mode = cmd[4] #1:Forward + Forward #2:Forward + Side #3:Side + Forward #3:Side + Side

        if mode <= 2: #Forward Move to destination
            if ans_move_state == 0: 
                if des_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Destination(move)
                    if 0 <= des_robot_angle_full <= 180: #Turn right
                        robot_cmd = move_amount_adjust(des_robot_angle,'e1')
                    else: #Turn left
                        robot_cmd = move_amount_adjust(des_robot_angle,'q1') 
                else: #Move
                    if robot_des_length >= CONST.TOLERANCE: #Forward       
                        robot_cmd = move_amount_adjust(robot_des_length,'w1')
                    else: #Can't move any more                    
                        ans_move_state = 1
            elif ans_move_state == 1:
                if dir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Aim direction(action)
                    if 0 <= dir_robot_angle_full <= 180: #Turn right
                        robot_cmd = move_amount_adjust(dir_robot_angle,'e1') 
                    else: #Turn left
                        robot_cmd = move_amount_adjust(dir_robot_angle,'q1')
                else: #Can't move any more
                    ans_move_state = 2
            else: #ans_move_state == 2
                if mode == 1 or mode == 3: #Forward Move to ball
                    if ball_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Ball(move)
                        if 0 <= ball_robot_angle_full <= 180: #Turn right
                            robot_cmd = move_amount_adjust(ball_robot_angle,'e1')
                        else: #Turn left
                            robot_cmd = move_amount_adjust(ball_robot_angle,'q1') 
                    else: #Move
                        if robot_ball_length >= CONST.TOLERANCE: #Forward
                            robot_cmd = move_amount_adjust(robot_ball_length,'w1')      
                        else: #Action                    
                            robot_cmd, ans_move_state = robot_action(robot, robo_pos, cmd, sent_cmd, move_state) 
                else: #Side Move to ball
                    #Compute newdir_uxy
                    newdir_uxy = [0,0]
                    if 0 <= ball_robot_angle_full <= 180: #Right Side Move
                        newdir_uxy = rotate_vector(robot_ball_uxy,-90)
                    else: #Left Side Move
                        newdir_uxy = rotate_vector(robot_ball_uxy,90)
                    #Angle : newdir_uxy -> robot_Direction[robot]
                    newdir_robot_angle , newdir_robot_angle_full = vector_angle(newdir_uxy,robo_dir)

                    if newdir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Ball(move)
                        if 0 <= newdir_robot_angle_full <= 180: #Turn right
                            robot_cmd = move_amount_adjust(newdir_robot_angle,'e1')
                        else: #Turn left
                            robot_cmd = move_amount_adjust(newdir_robot_angle,'q1') 
                    else: #Move
                        if robot_ball_length >= CONST.TOLERANCE: #Left or Right
                            if 0 <= des_robot_angle_full <= 180: #Move Right
                                robot_cmd = move_amount_adjust(robot_ball_length,'d1') 
                            else: #Move left
                                robot_cmd = move_amount_adjust(robot_ball_length,'a1')      
                        else: #Action
                            robot_cmd, ans_move_state = robot_action(robot, robo_pos, cmd, sent_cmd, move_state)
        else: #Side Move to destination
            if ans_move_state == 0: 
                #Compute newdir_uxy
                newdir_uxy = [0,0]
                if 0 <= des_robot_angle_full <= 180: #Right Side Move
                    newdir_uxy = rotate_vector(robot_des_uxy,-90)
                else: #Left Side Move
                    newdir_uxy = rotate_vector(robot_des_uxy,90)
                #Angle : newdir_uxy -> robot_Direction[robot]
                newdir_robot_angle , newdir_robot_angle_full = vector_angle(newdir_uxy,robo_dir)

                if newdir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.MOVING_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Destination(move)
                    if 0 <= newdir_robot_angle_full <= 180: #Turn right
                        robot_cmd = move_amount_adjust(newdir_robot_angle,'e1')
                    else: #Turn left
                        robot_cmd = move_amount_adjust(newdir_robot_angle,'q1') 
                else: #Move
                    if robot_des_length >= CONST.TOLERANCE: #Left or Right
                        if 0 <= des_robot_angle_full <= 180: #Move Right
                            robot_cmd = move_amount_adjust(robot_des_length,'d1') 
                        else: #Move left
                            robot_cmd = move_amount_adjust(robot_des_length,'a1')      
                    else: #Can't move any more
                        ans_move_state = 1
            elif ans_move_state == 1:
                if dir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Aim direction(action)
                    if 0 <= dir_robot_angle_full <= 180: #Turn right
                        robot_cmd = move_amount_adjust(dir_robot_angle,'e1') 
                    else: #Turn left
                        robot_cmd = move_amount_adjust(dir_robot_angle,'q1')
                else: #Can't move any more
                    ans_move_state = 2
            else: #ans_move_state == 2
                if mode == 1 or mode == 3: #Forward Move to ball
                    if ball_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Ball(move)
                        if 0 <= ball_robot_angle_full <= 180: #Turn right
                            robot_cmd = move_amount_adjust(ball_robot_angle,'e1')
                        else: #Turn left
                            robot_cmd = move_amount_adjust(ball_robot_angle,'q1') 
                    else: #Move
                        if robot_ball_length >= CONST.TOLERANCE: #Forward
                            robot_cmd = move_amount_adjust(robot_ball_length,'w1')      
                        else: #Action                    
                            robot_cmd, ans_move_state = robot_action(robot, robo_pos, cmd, sent_cmd, move_state) 
                else: #Side Move to ball
                    #Compute newdir_uxy
                    newdir_uxy = [0,0]
                    if 0 <= ball_robot_angle_full <= 180: #Right Side Move
                        newdir_uxy = rotate_vector(robot_ball_uxy,-90)
                    else: #Left Side Move
                        newdir_uxy = rotate_vector(robot_ball_uxy,90)
                    #Angle : newdir_uxy -> robot_Direction[robot]
                    newdir_robot_angle , newdir_robot_angle_full = vector_angle(newdir_uxy,robo_dir)

                    if newdir_robot_angle >= COM_CONST.TURN[COM_CONST.SMALL] * COM_CONST.KICK_TURN_RATIO[COM_CONST.PLAYER_1]: #Turn to : Ball(move)
                        if 0 <= newdir_robot_angle_full <= 180: #Turn right
                            robot_cmd = move_amount_adjust(newdir_robot_angle,'e1')
                        else: #Turn left
                            robot_cmd = move_amount_adjust(newdir_robot_angle,'q1') 
                    else: #Move
                        if robot_ball_length >= CONST.TOLERANCE: #Left or Right
                            if 0 <= des_robot_angle_full <= 180: #Move Right
                                robot_cmd = move_amount_adjust(robot_ball_length,'d1') 
                            else: #Move left
                                robot_cmd = move_amount_adjust(robot_ball_length,'a1')      
                        else: #Action
                            robot_cmd, ans_move_state = robot_action(robot, robo_pos, cmd, sent_cmd, move_state)                    

    else: #Exception:None
        robot_cmd = 'N1'

    # if the robot has sit
    if robot_is_rest[robot] == 1 and (robot_cmd[0] != 'y' or \
        robot_cmd[0] != 'Y' or robot_cmd[0] != 'f' or robot_cmd[0] != 'g'):
        robot_is_rest[robot] = 0
    elif robot_is_rest[robot] == 1 and robot_cmd[0] != 'r':
        robot_cmd = 'R1'
        robot_is_rest[robot] = 0
    else: 
        pass

    # ans_move_state q= move_state
    return robot_cmd, ans_move_state
