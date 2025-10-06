from ast import If
import math
import cv2
import time 

# Const
kick_count = 0
ROB_RANG = 25
KICKABLE_RANGE = 30 #5/19
ROUGH_RANG = 30
ERROR_DISTANCE = 10
ROTATE_ANGLE = 0.25
SAFE_DIST = 15  # RoboRad/2 + ball radius 43
WAY_ANGLE = {'F': 0, 'L': -math.pi/2, 'R': math.pi/2}
MOVE = {
    'F': {'Fast': 'W1', 'Norm': 'w1', 'Bound': [18, 10]},
    'L': {'Fast': 'A1', 'Norm': 'a1', 'Kick': 'h1', 'Bound': [24, 13]},
    'R': {'Fast': 'D1', 'Norm': 'd1', 'Kick': 'j1', 'Bound': [24, 13]},
    'B': {'Norm': 's1', 'Bound': [None, 10]}
}
job = ['N1', 'N1', 'N1']
stage_each = [0, 0, 0]
try_get_away_count = 0 #避免卡住
# Condition
ALLOW_MOVE_WAY = ['L', 'R']
# Field Variable
BOUNDARY = []  # [(x, y), ...]
CENTER = (0, 0)
ori_center = []
ga_x = ga_y = 0

# Ball, Robot Parameter
ball_p = [0, 0]
ball_ori_x = 0#判斷踢到球了沒
player_id = [0,0,0]
player_p = [[0,0],[0,0],[0,0]]
player_d = [[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]]
oppo_id = [0,0,0]
oppo_p = [[0,0],[0,0],[0,0]]
dist_ball = [0,0,0]
near_ball_id = 0
far_ball_id = 0
near_ball_dist = 0
near_ourside_op_id = 0
oppo_p_x =[0, 0, 0] #判斷緊迫盯人對象
# Anticipate point

stage = 0
kick_goal = [0, 0]
kick_dir = [0.0, 0.0]
first_arri = [0, 0] #離球近的目標點
obst_arri = [0, 0] #離球遠的卡人點
away_arri = [0, 0] #離球遠的遠離離球近的目標點
back_arri = [0, 0]
back_arri2 = [0, 0]
kick_point = [0, 0]#離球近的踢球點
kick_p = [0, 0]
kick_d = [0.0, 0.0]
kick_way = ""
move_way = ""
move_way2 = ""#離球遠的移動使用
move_way_away = ""#分開時使用
back_way = ""#回防
back_way2 = ""#回防
move_dir = [0.0, 0.0]
move_dir2 = [0.0, 0.0]
back_dir = [0.0, 0.0]#回防
back_dir2 = [0.0, 0.0]#回防
away_dir = [0.0, 0.0] #分開機器人用的方向
kick_flag = False
kicked = False

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
    global BOUNDARY, ori_center, player_id, oppo_id
    ori_center = center
    if side == 1:
        player_id = [0,1,2]
        oppo_id = [3,4,5]
    elif side == -1:
        player_id = [3,4,5]
        oppo_id = [0,1,2]

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

    print("Your Player is Robot", player_id[0:3])
    print("Oppo Player is Robot", oppo_id[0:3])
    global stage 
    stage = 1

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
    
    cv2.circle(frame, (int(first_arri[0]), int(first_arri[1])), 3, (255, 133, 0), -1) 
    cv2.circle(frame, (int(kick_point[0]), int(kick_point[1])), 3, (232, 122, 63), -1) #藍色
    cv2.circle(frame, (int(obst_arri[0]), int(obst_arri[1])), 3, (237, 183, 217), -1)  #淡粉色
    cv2.circle(frame, (int(away_arri[0]), int(away_arri[1])), 3, (255, 0, 255), -1)  #深粉色

    
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
    #print('Updating Robot Info')
    global player_d, player_p, player_id, oppo_id, oppo_p, ball_p
    i = 0
    for i in range(3):
        player_d[i] = teamD[i]
        player_p[i] = teamP[i]
        print('  player',player_id[i],'is at ',player_p[i])
        oppo_p[i] = oppoP[i]
        print('    oppo',oppo_id[i],'is at ',oppo_p[i])
    ball_p = ballP
    pass


def strategy():
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """
    
    # print("player position:",[[int(round(x)) for x in pair] for pair in player_p])#確定一下進來的數據
    # print("ball position:",[int(round(x)) for x in ball_p])

    global stage
    #5/19 要不要考慮球的位置？
    if stage == 1:
        print('[stage] == 1')  # Find position of goal
        kick_goal[0] = BOUNDARY[2][0]
        allow = [BOUNDARY[2][1], BOUNDARY[5][1]] \
            if BOUNDARY[2][1] < BOUNDARY[5][1] else [BOUNDARY[5][1], BOUNDARY[2][1]]
        #print(allow)
        #print(oppo_p[2])
        block = []
        block.append(oppo_p[2][1] - ROB_RANG) #oppo keeper -->block
        block.append(oppo_p[2][1] + ROB_RANG) #5/19
        #print(block)
        if block[0] > allow[0]:
            if block[1] < allow[-1]:
                allow.insert(1, block[0])
                allow.insert(2, block[1])
            else:
                allow[1] = block[0]
        else:
            allow[0] = block[1]
        dist = None
        

        for i in range(0, len(allow), 2):
            #print(f"[DEBUG] check range: {allow[i]} to {allow[i+1]}")
            if allow[i+1] - allow[i] > KICKABLE_RANGE: # kickable_range = 30
                kick_goal_temp = int((allow[i] + allow[i+1]) / 2)
                #print(f"→ candidate goal y: {kick_goal_temp}")
                if dist is None or dist > abs(ball_p[1] - kick_goal_temp):
                    #print(f"→ better goal found, dist = {abs(ball_p[1] - kick_goal_temp)}")
                    dist = abs(ball_p[1] - kick_goal_temp)
                    kick_goal[1] = kick_goal_temp

        print('Goal is set as', kick_goal[0], kick_goal[1])#想把球射到球門裡的位置
        
        #找到誰離球近,敵人誰離我家近 #5/19 如果說場上只剩一個機器人這樣還守嗎
        # 有偵測到就守 沒有就不守 三個按照x座標排順序 -1000 -1000 360 這樣最後一個會是守門員
        # 守門員也一樣有就理它 沒有就不理它
        global near_ball_id, dist_ball, near_ball_dist, far_ball_id, near_ourside_op_id, oppo_p_x 
        for i in range(3):
            dist_ball[i] = math.hypot(player_p[i][0]-ball_p[0], player_p[i][1]-ball_p[1])
        #     print('dist to ball of player ', player_id[i], ' = ', dist_ball[i])
        #永遠都是2離球最近
        near_ball_id = 2
        near_ball_dist = dist_ball[near_ball_id]
        far_ball_id = (near_ball_id+2)%4
        for i in range(3):
            oppo_p_x[i] = oppo_p[i][0]
        near_ourside_op_id = oppo_p_x.index(min(oppo_p_x)) #離家裡最近（x最小）的是離球遠的機器人緊迫盯人的目標
        # print('most close to the ball is player ', near_ball_id, 'dist is ', near_ball_dist,'the other player is', far_ball_id)
        # print('the oppo most close to our side =', near_ourside_op_id)

        print('------------------ movement discussion ------------------')
        global kick_dir, first_arri, kick_point, move_dir, move_way

        kick_dir = _unit_vector(ball_p, kick_goal)
        first_arri = [int(ball_p[i]-kick_dir[i]*ROUGH_RANG) for i in range(2)]
        if first_arri[1] > BOUNDARY[6][1] :
            first_arri[1] = BOUNDARY[6][1]-ROB_RANG -10 
        elif first_arri[1]<  BOUNDARY[0][1] :
            first_arri[1] = BOUNDARY[0][1]+ROB_RANG + 10
        kick_point = [int(ball_p[i]-kick_dir[i]*SAFE_DIST) for i in range(2)]
        move_dir = _unit_vector(player_p[near_ball_id], first_arri)
        move_way = _find_move_way(move_dir,near_ball_id)      
        print('kick dir =', [round(x, 2) for x in kick_dir])
        print('kick_point = ', kick_point) #要踢球前站的位置
        print('first arri=  ', first_arri) #前往踢球點的第一個位置
        print('forward turn choose :', move_way)

        global obst_arri, move_dir2, move_way2 #離球遠的機器人緊迫盯人的目標位置, 移動方向, 移動方式
        obst_arri[0] = int(oppo_p[near_ourside_op_id][0]-2*SAFE_DIST)
        obst_arri[1] = int(oppo_p[near_ourside_op_id][1])
        move_dir2 = _unit_vector(player_p[far_ball_id], obst_arri)
        move_way2 = _find_move_way(move_dir2,far_ball_id)
        print('obst arri = ', obst_arri)
        print('backward turn choose :', move_way2)
        print('-------------------------------------------------------------')
        
        global stage_each, job, try_get_away_count, away_dir
        job = ['N1', 'N1', 'N1']
        stage_each = [0, 0, 0]
        dist = math.hypot(player_p[near_ball_id][0]-first_arri[0], player_p[near_ball_id][1]-first_arri[1])#dist = dist of robot to first arri
        print('[arri]-[player]-[dist]', first_arri, [int(x) for x in player_p[near_ball_id]], int(dist))

        #  #避免同隊卡住,目前只有0,2卡住有用
        # if math.hypot(player_p[0][0]-player_p[2][0], player_p[0][1]-player_p[2][1]) < 2.5*(SAFE_DIST+ERROR_DISTANCE ) :
        #     print('!!!!!!!!!!!!!!!!!! teamates are too close  !!!!!!!!!!!!!!!!!!')
        #     global away_arri
        #     away_dir = _unit_vector(player_p[near_ball_id], player_p[far_ball_id])
        #     away_arri = [int(player_p[far_ball_id][i]+away_dir[i]*(SAFE_DIST+ERROR_DISTANCE)) for i in range(2)]
        #     print('away dir =  ', away_dir)
        #     print('away arri = ', away_arri)
        #     global move_way_away
        #     move_way_away = _find_move_way(away_dir,far_ball_id)
        #     print('get away rob turn:', move_way_away)
        #     dist_away = math.hypot(player_p[far_ball_id][0]-away_arri[0], player_p[far_ball_id][1]-away_arri[1])#dist_away = dist of far_ball_robot to away_arri
        #     if dist_away >= ERROR_DISTANCE:
        #         dirction = _rotate(player_d[far_ball_id], WAY_ANGLE[move_way_away])
        #         angle = _angle(away_dir, dirction)
        #         print('--get away turn--')
        #         job[far_ball_id] = _turning(angle, far_ball_id)
        #         if job[far_ball_id] != 'N1':
        #             stage_each[far_ball_id] = 1
        #     '''
        #     MOVE
        #     '''
        #     if stage_each[far_ball_id] == 0 :
        #         print('--get away move--')
        #         #if dist_away >= MOVE[move_way_away]['Bound'][0]:
        #         print('[robot]',far_ball_id, '-->', MOVE[move_way_away]['Fast']) #避免離開太慢還是會卡到所以離開速度調成一定是快
        #         job[far_ball_id] = MOVE[move_way_away]['Fast']
        #         stage_each[far_ball_id] = 1

        #  #回防
        # if ball_p[0] < (BOUNDARY[10][0]*7/8+BOUNDARY[3][0]/8) : #如果球在全場的1/8以內就回來
        #     print('!!!!!!!!!!!!!!!!!! get back !!!!!!!!!!!!!!!!!!')
        #     global back_way, back_way2, back_arri, back_arri2, back_dir, back_dir2
        #     back_arri[0] = back_arri2[0] = ((BOUNDARY[10][0]/2+BOUNDARY[3][0]/2)-235)
        #     back_arri[1] = (BOUNDARY[0][1]/2+BOUNDARY[11][1]/2)
        #     back_arri2[1] = (BOUNDARY[7][1]/2+BOUNDARY[8][1]/2)
        #     if player_p[0][1] < player_p[2][1]: #誰上誰下
        #         top_id = 0
        #         bot_id = 2
        #     else:
        #         top_id = 2
        #         bot_id = 0
        #     back_dir = _unit_vector(player_p[top_id], back_arri)
        #     back_dir2= _unit_vector(player_p[bot_id], back_arri2)
        #     back_way = _find_move_way(back_dir,top_id)
        #     back_way2= _find_move_way(back_dir2,bot_id)
        #     dirction = _rotate(player_d[top_id], WAY_ANGLE[back_way])
        #     angle = _angle(back_dir, dirction)
        #     dirction2 = _rotate(player_d[bot_id], WAY_ANGLE[back_way2])
        #     angle2 = _angle(back_dir2, dirction2)
        #     print('--get back turn--')
        #     job[top_id] = _turning(angle, top_id)
        #     if job[top_id] != 'N1':
        #         stage_each[top_id] = 1
        #     job[bot_id] = _turning(angle2, bot_id)
        #     if job[bot_id] != 'N1':
        #         stage_each[bot_id] = 1
            
        #     #MOVE
            
        #     if stage_each[top_id] == 0 :
        #         print('--back move--')
        #         back_dist = math.hypot(player_p[top_id][0]-back_arri[0], player_p[top_id][1]-back_arri[1])
        #         job[top_id] = _moving(back_dist, top_id, back_way)
        #         if job[top_id] != 'N1':
        #             stage_each[top_id] = 1
        #     if stage_each[bot_id] == 0 :
        #         print('--back move--')
        #         back_dist2 = math.hypot(player_p[bot_id][0]-back_arri2[0], player_p[bot_id][1]-back_arri2[1])
        #         job[bot_id] = _moving(back_dist2, bot_id, back_way2)
        #         if job[bot_id] != 'N1':
        #             stage_each[bot_id] = 1

#######非緊急狀況 正常運行#######
        if stage_each[near_ball_id] == 0:
            if dist >= ERROR_DISTANCE and near_ball_dist > SAFE_DIST+8:
                dirction = _rotate(player_d[near_ball_id], WAY_ANGLE[move_way])
                angle = _angle(move_dir, dirction)
                print('--near first turn--')
                job[near_ball_id] = _turning(angle, near_ball_id)
                if job[near_ball_id] != 'N1':
                    stage_each[near_ball_id] = 1
#離球太近會左右移動卻沒用
            ''' elif near_ball_dist < SAFE_DIST+8: 
                dirction = _rotate(player_d[near_ball_id], WAY_ANGLE[move_way])
                angle = _angle(move_dir, dirction)
                if angle > 0 and angle > 2*ROTATE_ANGLE:
                    print('--too close to ball left turn--')
                    print(near_ball_id, '-->q1')
                    job[near_ball_id] = 'q1'
                    stage_each[near_ball_id] = 1
                elif angle < 0 and angle < -2*ROTATE_ANGLE:
                    print('--too close to ball right turn--')
                    print(near_ball_id, '-->e1')
                    job[near_ball_id] = 'e1'
                    stage_each[near_ball_id] = 1 '''
        
        #MOVE
        
        if stage_each[near_ball_id] == 0 :
            print('--near first move--')
            job[near_ball_id] = _moving(dist, near_ball_id, move_way)
            if job[near_ball_id] != 'N1':
                stage_each[near_ball_id] = 1

        if stage_each[far_ball_id] == 0 :
            dist2 = math.hypot(player_p[far_ball_id][0]-obst_arri[0], player_p[far_ball_id][1]-obst_arri[1])#dist2 = dist of far_ball_robot to obst arri
            if dist2 >= ERROR_DISTANCE:
                dirction = _rotate(player_d[far_ball_id], WAY_ANGLE[move_way2])
                angle = _angle(move_dir2, dirction)
                print('--far first turn--')
                job[far_ball_id] = _turning(angle, far_ball_id)
                if job[far_ball_id] != 'N1':
                    stage_each[far_ball_id] = 1
        '''
        MOVE
        '''
        if stage_each[far_ball_id] == 0 :
            print('--far first move--')
            job[far_ball_id] = _moving(dist2, far_ball_id, move_way2)
            if job[far_ball_id] != 'N1':
                stage_each[far_ball_id] = 1
        
        print('------------------ final check  ------------------')
        global kick_way# Choose the way of kick
        if kick_way == "" :
            kick_way = _find_move_way(kick_dir,near_ball_id)
            print('kickway = ', kick_way, "final arri", first_arri, "player's position",player_p[near_ball_id])
        dirction = _rotate(player_d[near_ball_id], WAY_ANGLE[kick_way])
        angle = _angle(kick_dir, dirction)
        print('final distance to ball:', round(near_ball_dist))
        print('final error angle to ball:', round(angle*180/math.pi)," degree")
        if stage_each[near_ball_id] == 0 :
            print('--kick turn--')
            job[near_ball_id] = _turning(angle, near_ball_id)
            if job[near_ball_id] != 'N1':
                stage_each[near_ball_id] = 1
            
        if stage_each[near_ball_id] == 0 :
            WAYS = ['F', 'R', 'B', 'L']
            for i in [1, 2, 3, 0]:
                move_way = WAYS[(WAYS.index(kick_way)+i) % 4]
                temp_dir = _rotate(dirction, math.pi/2*i)
                diff_vec = [k - p for k, p in zip(kick_point, player_p[near_ball_id])]
                product = _dot(temp_dir, diff_vec)
                print('S3', move_way, ':', product)
                if product > MOVE[move_way]['Bound'][1]:
                    print('kick move:', MOVE[move_way]['Norm'])
                    print(near_ball_id, '---->', MOVE[move_way]['Norm'])
                    job[near_ball_id] = MOVE[move_way]['Norm']
                    print('returning job =', job)
                    return job
        if stage_each[near_ball_id] == 0 :
            print(near_ball_id, 'semi final--->', MOVE[kick_way]['Norm'])
            job[near_ball_id] = MOVE[kick_way]['Norm']
            stage_each[near_ball_id] = 1
            stage = 2
            print('//////////////////////////////////////////')
        if stage_each[0] + stage_each[2] > 0:
            print('job = ', job)
            return job

    if stage == 2: #確定要踢球了
        global ball_ori_x, kick_count

        print('ball_ori_x = ',ball_ori_x)
        print('ball_p = ',ball_p)

        if ball_ori_x != 0 and abs( ball_ori_x - ball_p[0] ) >= 2: #如果球動了就回到stage1
            stage = 1
            ball_ori_x = ball_p[0]
            print('[stage] back to stage 1 ( ball moved )')
            return ['N1', 'N1', 'N1']
        elif kick_count > 3 :
            stage = 1
            kick_count = 0
            kick_way = "" 
            print('[stage] back to stage 1 ( your fool )')
            return ['N1', 'N1', 'N1']

        print('[stage] == 2')
        # if ball_p[0] < (BOUNDARY[10][0]/4+BOUNDARY[3][0]*3/4): #如果沒過半場的一半就繼續帶球
        #     ball_ori_x = ball_p[0]
        #     print(near_ball_id, 'final--->', MOVE[kick_way]['Kick'])
        #     job[near_ball_id] = MOVE[kick_way]['Kick']
        #     stage_each[near_ball_id] = 1
        #     print('KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK')
        #     time.sleep(1)
        #     #5/23
        #     if kick_count >= 3:
        #         stage = 1
        #         kick_count = 0  # reset for next time
        #         # 會踢不到球 建議踢三次
        #ifball_p[0] > (BOUNDARY[10][0]/4+BOUNDARY[3][0]*3/4): #如果過半場的一半就直接用射的
        ball_ori_x = ball_p[0]
        print(near_ball_id, 'final---SHOOT--->', MOVE[kick_way]['Kick'])
        job[near_ball_id] = MOVE[kick_way]['Kick']
        kick_count += 1
        stage_each[near_ball_id] = 1
        print('SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS')
        time.sleep(1)
        #stage = 1
        if stage_each[0] + stage_each[2] > 0:
            print('job = ', job)
            return job

    print('player, kick ball', player_p[near_ball_id], kick_point, near_ball_dist)    
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
    global stage
    print('========= sending job ===========', update)
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
    uniVector = [comp/length for comp in vector]
    return uniVector

def _dot(x, y):
    """Dot product as sum of list comprehension doing element-wise multiplication"""
    return sum(x_i*y_i for x_i, y_i in zip(x, y))

def _rotate(vector, angle):
    rot_vector = [0.0, 0.0]
    rot_vector[0] = (math.cos(angle)*vector[0]) - (math.sin(angle)*vector[1])
    rot_vector[1] = (math.sin(angle)*vector[0]) + (math.cos(angle)*vector[1])
    return rot_vector

def _angle(a, b):
    cross = a[0]*b[1] - a[1]*b[0]
    return math.asin(cross)

def _turning(angle,id):
    if angle > 0 and angle > 2*ROTATE_ANGLE:
        print('[robot]',id, '-->Q1')
        turning_job = 'Q1'
    elif angle < 0 and angle < -2*ROTATE_ANGLE:
        print('[robot]',id, '-->E1')
        turning_job = 'E1'
    elif angle > 0 and angle > ROTATE_ANGLE:
        print('[robot]',id, '-->q1')
        turning_job = 'q1'
    elif angle < 0 and angle < -ROTATE_ANGLE:
        print('[robot]',id, '-->e1')
        turning_job = 'e1'
    else:
        turning_job = 'N1'
    return turning_job

def _find_move_way(move_dir,id,):
    product = -1
    for way in ALLOW_MOVE_WAY:
        temp_product = _dot(move_dir, _rotate(player_d[id], WAY_ANGLE[way]))
        print(way, ':', temp_product, move_dir, _rotate(player_d[id], WAY_ANGLE[way]))
        if temp_product > product:
            product = temp_product
            move_way = way
    return move_way

def _moving(dist,id,way):
    if dist >= MOVE[way]['Bound'][0]:
        print('[robot]',id, '-->', MOVE[way]['Fast'])
        job = MOVE[way]['Fast']
    elif dist >= MOVE[way]['Bound'][1]:
        print('[robot]',id, '-->', MOVE[way]['Norm'])
        job = MOVE[way]['Norm']
    else:
        job = 'N1'
    return job