# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

# import challenge1
# import challenge2_1 as challenge2


for i in range(3):  # 讀取場地座標
    field1 = []
    field2 = []
    try:
        if i == 0:
            f = open('field_x.txt', 'r')  # 開啟場地的x座標文件
            f2 = open('field_y.txt', 'r')  # 開啟場地的y座標文件
        elif i == 1:
            f = open('fieldcenter_x.txt', 'r')  # 開啟場地的x座標文件
            f2 = open('fieldcenter_y.txt', 'r')  # 開啟場地的y座標文件
        elif i == 2:
            f = open('fieldpenalty_x.txt', 'r')  # 開啟場地的x座標文件
            f2 = open('fieldpenalty_y.txt', 'r')  # 開啟場地的y座標文件

        for line in f.readlines():  # 讀取場地的x座標
            list = line.strip('\n')
            field1.append(int(list))
        for line in f2.readlines():  # 讀取場地的y座標
            list = line.strip('\n')
            field2.append(int(list))

        if i == 0:
            field = [[0] * 2 for i in range(len(field1))]
            for i in range(len(field1)):  # 將場地的x與y座標整合進一個二維陣列
                field[i][0] = field1[i]
                field[i][1] = field2[i]
        elif i == 1:
            fieldcenter = [[0] * 2 for i in range(len(field1))]
            for i in range(len(field1)):  # 將場地的x與y座標整合進一個二維陣列
                fieldcenter[i][0] = field1[i]
                fieldcenter[i][1] = field2[i]
        elif i == 2:
            fieldpenalty = [[0] * 2 for i in range(len(field1))]
            for i in range(len(field1)):  # 將場地的x與y座標整合進一個二維陣列
                fieldpenalty[i][0] = field1[i]
                fieldpenalty[i][1] = field2[i]

        f.close()
        f2.close()
    except:
        print('沒找到座標文件')
angle1 = [[0.09590279871463747, -0.9953907037935906],[1,-1]]
location = [(132,132),(135,135)]
oppoP = [(140.1,140.1),(145,145)]
ballcenter = [150.2, 150.2]
#
# challenge1.Initialize()
# challenge1.strategy_update_field(1,field,fieldcenter,fieldpenalty,0,0,0,0,0)
# while True:
#     challenge1.Update_Robo_Info(angle, location, oppoP, ballcenter)
#     challenge1.strategy()


import imageprocess_v4 as imageprocess
import threading
import time
import tkinter as tk
import numpy as np
import challenge_1final_v7 as st_1
import challenge_1final_v7  as cha_1
import challenge2_1 as st_2
import challenge2_1 as cha_2
import challenge3_forward as st_3_1
import challenge3_keeper_final_v2 as st_3_2
import challenge3_forward as cha_3_1
import challenge3_keeper_final_v2 as cha_3_2
import blank_strategy as st_B
import nrf_controller_run_directly as controller

Side = 1

real_real_oppoP = []
location =[]
def challenge1():  # 定義一個函式功能（內容自己自由編寫），供點選Button按鍵時呼叫，呼叫命令引數command=函式名
    imageprocess.ballcenter = []
    imageprocess.field = []
    imageprocess.location = []
    imageprocess.angle1 = []
    imageprocess.angle2 = []
    imageprocess.fieldcenter = []
    imageprocess.fieldpenalty = []
    imageprocess.oppoP = []

    t = threading.Thread(target=imageprocess.job)
    t.start()
    a = 0
    while a < 3:
        time.sleep(1)
        a += 1
        print(a)
        field = imageprocess.field
        print('ballcenter:', imageprocess.ballcenter)
        print('field:', imageprocess.field)
        print('fieldcenter', imageprocess.fieldcenter)
        print('fieldpenalty', imageprocess.fieldpenalty)
        print('location:', imageprocess.location)
        print('angle1:', imageprocess.angle1)
        print('angle2:', imageprocess.angle2)
        print('oppoP:', imageprocess.oppoP)

    # Parse field parameter to strategy and initialize strategy
    # parse_strategy_field(PK_x, FB_x, FB_y, Penalty_y)
    cha_1.strategy_update_field(1, field, fieldcenter, 0, 0, 0, 0, 0, 0)
    cha_1.Initialize()
    ball = [0, 0]
    count = 0
    device = controller.run_robot()
    # real_real_oppoP = []
    while True:
        global location,real_real_oppoP
        # location = []
        location = imageprocess.location
        angle1 = imageprocess.angle1
        angle2 = imageprocess.angle2
        oppoP = imageprocess.oppoP
        ballcenter = imageprocess.ballcenter
        angle = [angle1,angle2]
        print("angle,final:",angle)
        # angle = [[0.09590279871463747, -0.9953907037935906], [1, -1]]
        # location = [(132, 132), (135, 135)]
        # oppoP = [(400, 400),(300, 300), (900, 900),(1500,1500)]
        # ballcenter = [[300, 300],[1500,1500]]
        # ball = [0, 0]

        #過濾器########################################
        print("ballcenter",ballcenter)
        for i in range(len(ballcenter)):
            if field[0][0] < ballcenter[i][0] < field[5][0]:#當球在 field[5][0]和[4][0]之間時只有，球進了/球在場外
                if field[0][1] < ballcenter[i][1] < field[7][1]:
                    ball[0] = ballcenter[i][0]
                    ball[1] = ballcenter[i][1]
        real_oppoP = []
        #如果在球場範圍內 就加入real_oppoP
        for i in range(len(oppoP)):
            if field[0][0] < oppoP[i][0] < field[4][0]:
                if field[0][1] < oppoP[i][1] < field[7][1]:
                   real_oppoP.append([oppoP[i][0],oppoP[i][1]])
        # real_real_oppoP = []
        if real_real_oppoP == []:
            if len(real_oppoP)==2 :
                print("正確取值，現進行分配")
                # real_real_oppoP = []
                #比較靠上的作為機器人1
                if real_oppoP[0][1] < real_oppoP[1][1]:
                    real_real_oppoP.append(real_oppoP[0])
                    real_real_oppoP.append(real_oppoP[1])
                else:
                    real_real_oppoP.append(real_oppoP[1])
                    real_real_oppoP.append(real_oppoP[0])
                print("整理後的oppoP",real_real_oppoP)
            else:
                print("錯誤取值,",real_oppoP)
        ###############################################
        # print("ball",ball)
        #
        # if angle != [] and location != [] and real_real_oppoP != [] and ball != []:
        #     cha_1.Update_Robo_Info(angle, location, real_real_oppoP, ball)
        #     print("指令:",cha_1.strategy())
        print("location",location,"oppoP",real_real_oppoP,"ball",ball,"anlge",angle)
        if angle1 != [] and location != [] and angle2 != [] and real_real_oppoP != [] and ball != []:
            count += 1
            cha_1.Update_Robo_Info(angle, location, real_real_oppoP, ball)
            print(count, cha_1.strategy())
            move_data = cha_1.strategy()  # 如果都有拿到值就跟ch1要資料


        else:
            move_data = ['N1', 'N1', 'N1']  # 不然不動

        print('!!!!!!move_data = ', move_data)

        newball = [0,0]
        newballcenter = imageprocess.ballcenter
        # 過濾器########################################
        print("newballcenter", newballcenter)
        for i in range(len(ballcenter)):
            if field[0][0] < newballcenter[i][0] < field[5][0]:  # 當球在 field[5][0]和[4][0]之間時只有，球進了/球在場外
                if field[0][1] < ballcenter[i][1] < field[7][1]:
                    newball[0] = ballcenter[i][0]
                    newball[1] = ballcenter[i][1]



        print("new_ball",newball)
        if abs(newball[0] - ball[0]) >10 or abs(newball[1] - ball[1]) >10:
            kicked = True
        else:
            kicked =False
        return_sent_cmd1(move_data, kicked)
        controller.main_procedure(device, move_data)
        time.sleep(0.8)
    #
    # """
    # challenge1.Initialize()
    # challenge1.strategy_update_field(1,field,fieldcenter,fieldpenalty,0,0,0,0,0)
    # while True:
    #     challenge1.Update_Robo_Info(angle, location, oppoP, ballcenter)
    #     challenge1.strategy()
    # """
    #

def challenge2():  # 定義一個函式功能（內容自己自由編寫），供點選Button按鍵時呼叫，呼叫命令引數command=函式名
    imageprocess.ballcenter = []
    imageprocess.field = []
    imageprocess.location = []
    imageprocess.angle1 = []
    imageprocess.fieldcenter = []
    imageprocess.fieldpenalty = []
    imageprocess.oppoP = []

    t = threading.Thread(target=imageprocess.job2)
    t.start()
    a = 0
    while a < 15:
        time.sleep(1)
        a += 1
        print(a)
        field = imageprocess.field

        print('ballcenter:', imageprocess.ballcenter)
        print('field:', imageprocess.field)
        print('fieldcenter', imageprocess.fieldcenter)
        print('fieldpenalty', imageprocess.fieldpenalty)
        print('location:', imageprocess.location)
        print('angle:', imageprocess.angle1)
        print('oppoP:', imageprocess.oppoP)


    #Parse field parameter to strategy and initialize strategy
    # parse_strategy_field(PK_x, FB_x, FB_y, Penalty_y)
    cha_2.strategy_update_field(1, field, fieldcenter, 0, 0, 0, 0, 0, 0)
    cha_2.Initialize()
    # st_A.strategy_update_field(1,field,fieldcenter,fieldpenalty,0,0,0,0,0)
    # st_A.strategy_update_field(1, field,fieldcenter,0, 0, 0, 0, 0, 0)
    ball = [0,0]
    count = 0
    #device = controller.get_device()
    device = controller.run_robot()
    # for i in range(1):
    while True:
        # start_time = time.time()
        location = imageprocess.location
        angle1 = imageprocess.angle1
        oppoP = imageprocess.oppoP
        ballcenter = imageprocess.ballcenter
        print("ballcenter",ballcenter)
        for i in range(len(ballcenter)):
            if field[0][0]  < ballcenter[i][0]  <field[5][0]:
                if field[0][1]  < ballcenter[i][1]  <field[7][1]:
                    ball[0] = ballcenter[i][0]
                    ball[1] = ballcenter[i][1]
        real_oppoP = []
        for i in range(len(oppoP)):
            if field[0][0] < oppoP[i][0] < field[4][0]:
                if field[0][1] < oppoP[i][1] < field[7][1]:
                   real_oppoP.append([oppoP[i][0],oppoP[i][1]])
        print(oppoP)
        print("ball",ball)
        print("real_oppoP",real_oppoP)
        # angle1 = [[0.09590279871463747, -0.9953907037935906], [1, -1]]
        # location = [(132, 132), (135, 135)]
        # oppoP = [(140.1, 140.1), (145, 145)]
        # ballcenter = [150.2, 150.2]
        if angle1!=[] and location != [] and real_oppoP != [] and ball != []:
            count += 1
            # time.sleep(0.3)
            cha_2.Update_Robo_Info(angle1, location, real_oppoP, ball)
            print(count, cha_2.strategy())
            move_data = cha_2.strategy() #如果都有拿到值就跟ch2要資料
            # time.sleep(0.5)

            
        else:
            move_data = ['N1', 'N1', 'N1'] #不然不動
        print('move_data = ', move_data)
        return_sent_cmd2(move_data,True)
        controller.main_procedure(device,move_data)
        # end_time = time.time()
        # time.sleep(0.5)
        # print("total time",end_time - start_time)
        # time.sleep(0.5)



def challenge3():  # 定義一個函式功能（內容自己自由編寫），供點選Button按鍵時呼叫，呼叫命令引數command=函式名

    imageprocess.ballcenter = []
    imageprocess.field = []
    imageprocess.location = []
    imageprocess.angle1 = []
    imageprocess.angle2 = []
    imageprocess.angle3 = []
    imageprocess.fieldcenter = []
    imageprocess.fieldpenalty = []
    imageprocess.oppoP = []

    t = threading.Thread(target=imageprocess.job3)
    t.start()
    a = 0
    while a < 10:
        time.sleep(1)
        a += 1
        print(a)
        field = imageprocess.field
        print('ballcenter:', imageprocess.ballcenter)
        print('field:', imageprocess.field)
        print('fieldcenter', imageprocess.fieldcenter)
        print('fieldpenalty', imageprocess.fieldpenalty)
        print('location:', imageprocess.location)
        print('angle1:', imageprocess.angle1)
        print('angle2:', imageprocess.angle2)
        print('angle3:', imageprocess.angle3)
        print('oppoP:', imageprocess.oppoP)
    
    cha_3_1.strategy_update_field(1, field, fieldcenter, 0, 0, 0, 0, 0, 0)
    cha_3_2.strategy_update_field(1, field, fieldcenter, 0, 0, 0, 0, 0, 0)
    cha_3_1.Initialize()
    cha_3_1.Initialize()
    ball = [0, 0]
    count = 0
    device = controller.run_robot()
    while True:
        location = imageprocess.location
        angle1 = imageprocess.angle1
        angle2 = imageprocess.angle2
        angle3 = imageprocess.angle3
        oppoP = imageprocess.oppoP
        ballcenter = imageprocess.ballcenter
        angle = [angle1,angle2,angle3]
        print("angle,final:",angle)

        #過濾器#
        print("ballcenter",ballcenter)
        for i in range(len(ballcenter)):
            if field[0][0] < ballcenter[i][0] < field[5][0]:#當球在 field[5][0]和[4][0]之間時只有，球進了/球在場外
                if field[0][1] < ballcenter[i][1] < field[7][1]:
                    ball[0] = ballcenter[i][0]
                    ball[1] = ballcenter[i][1]
        real_oppoP = []
        #如果在球場範圍內 就加入real_oppoP
        for i in range(len(oppoP)):
            if field[0][0] < oppoP[i][0] < field[4][0]:
                if field[0][1] < oppoP[i][1] < field[7][1]:
                   real_oppoP.append([oppoP[i][0],oppoP[i][1]])
        print("location",location,"oppoP",real_oppoP,"ball",ball,"anlge",angle)
        move_data1 = []
        move_data2 = []
        if angle1 != [] and location != [] and angle2 != [] and angle3 != [] and real_oppoP != [] and ball != []:
            count += 1
            cha_3_1.Update_Robo_Info(angle, location, real_oppoP, ball)
            cha_3_2.Update_Robo_Info(angle, location, real_oppoP, ball)
            move_data1 = cha_3_1.strategy()
            print('!!!!!!move_data1 = ', move_data1)
            move_data2 = cha_3_2.strategy()
            print('!!!!!!move_data2 = ', move_data2)
            move_data = [move_data1[0],move_data2[1],move_data1[2]]
            print(count, move_data)

        else:
            move_data = ['N1', 'N1', 'N1']  # 不然不動待補

        print('!!!!!!move_data = ', move_data)
        return_sent_cmd3_1(move_data1,True)
        return_sent_cmd3_2(move_data2,True)
        controller.main_procedure(device, move_data)
        time.sleep(0.8)

def test_error():
    print("YEAH")
    x = input()
    print('move_data = ', x)
    # imageprocess.ballcenter = []
    # imageprocess.field = []
    imageprocess.location = []
    imageprocess.angle1 = []
    # imageprocess.fieldcenter = []
    # imageprocess.fieldpenalty = []
    # imageprocess.oppoP = []

    t = threading.Thread(target=imageprocess.job2)#借用挑戰2的image——process
    t.start()
    a = 0
    while a < 15:
        time.sleep(1)
        a += 1
        print(a)
        field = imageprocess.field

        # print('ballcenter:', imageprocess.ballcenter)
        # print('field:', imageprocess.field)
        # print('fieldcenter', imageprocess.fieldcenter)
        # print('fieldpenalty', imageprocess.fieldpenalty)
        print('location:', imageprocess.location)
        print('angle:', imageprocess.angle1)
        # print('oppoP:', imageprocess.oppoP)

    # Parse field parameter to strategy and initialize strategy
    # parse_strategy_field(PK_x, FB_x, FB_y, Penalty_y)
    # cha_2.strategy_update_field(1, field, fieldcenter, 0, 0, 0, 0, 0, 0)
    # cha_2.Initialize()
    # st_A.strategy_update_field(1,field,fieldcenter,fieldpenalty,0,0,0,0,0)
    # st_A.strategy_update_field(1, field,fieldcenter,0, 0, 0, 0, 0, 0)
    # ball = [0, 0]
    # count = 0
    # device = controller.get_device()
    device = controller.run_robot()
    # for i in range(1):
    while True:
    # start_time = time.time()
        location = imageprocess.location#要有這兩個才會更新值
        angle1 = imageprocess.angle1
        # x = input()
        # print('move_data = ', x)

# Communication with strategy
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


def return_sent_cmd1(send_data, sent):
    """Pass sent command to strategy"""
    try:
        st_1.get_sent_cmd(send_data[0:3], sent)
        st_B.get_sent_cmd(send_data[3:6], sent)
    except AttributeError:
        pass
def return_sent_cmd2(send_data, sent):
    """Pass sent command to strategy"""
    try:
        st_2.get_sent_cmd(send_data[0:3], sent)
        st_B.get_sent_cmd(send_data[3:6], sent)
    except AttributeError:
        pass
def return_sent_cmd3_1(send_data, sent):
    """Pass sent command to strategy"""
    try:
        st_3_1.get_sent_cmd(send_data[0:3], sent)
        st_B.get_sent_cmd(send_data[3:6], sent)
    except AttributeError:
        pass
def return_sent_cmd3_2(send_data, sent):
    """Pass sent command to strategy"""
    try:
        st_3_2.get_sent_cmd(send_data[0:3], sent)
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

def imagedetection():
    imageprocess.imagedetection(fieldchoose)


def HSVdetection():
    imageprocess.HSVdetection(color)


def ColorMask():
    imageprocess.ColorMask(color)


def hello():
    print('fieldchoose:', fieldchoose)


def fieldlocation():
    global fieldchoose
    global color
    color = 0
    fieldchoose = 0
    var.set('邊界12個點')


def fieldcenter():
    global fieldchoose
    global color
    color = 0
    fieldchoose = 1
    var.set('中心點')


def fieldpenalty():
    global fieldchoose
    global color
    color = 0
    fieldchoose = 2
    var.set('罰球點')


def red():
    global color
    color = 7
    var.set('red')


def green():
    global color
    color = 1
    var.set('green')


def blue():
    global color
    color = 2
    var.set('blue')


def purple():
    global color
    color = 3
    var.set('purple')


def pink():
    global color
    color = 4
    var.set('pink')


def orange():
    global color
    color = 5
    var.set('orange')


def yellow():
    global color
    color = 6
    var.set('yellow')


def clear():
    global color
    imageprocess.clear(color)
    var.set('clear')
    color = 0


def reset():
    global color
    imageprocess.reset()
    var.set('')
    color = 0


def showall():
    global color
    color = 9
    var.set('show all')



try:
    F = open('HSVmax.txt', 'r')  # 開啟各顏色最大HSV值的文件
    F2 = open('HSVmin.txt', 'r')  # 開啟各顏色最小HSV值的文件
    elements = []
    for line in F:
        line = line.strip().split()
        elements.append(line)

    imageprocess.HSVcolormax = np.array(elements)
    imageprocess.HSVcolormax = imageprocess.HSVcolormax.astype(np.int64)

    elements = []
    for line in F2:
        line = line.strip().split()
        elements.append(line)

    imageprocess.HSVcolormin = np.array(elements)
    imageprocess.HSVcolormin = np.array(elements)
    imageprocess.HSVcolormin = imageprocess.HSVcolormin.astype(np.int64)
    F.close()
    F2.close()
except IOError:
    print("沒找到HSV的指定值,將使用預設值")

color = 0  # 哪個顏色
fieldchoose = 0  # 場地模式（0:邊界12個點,1:中心點,2:罰球點）
window = tk.Tk()  # 建立視窗window
window.title('Hello World')  # 給視窗的視覺化起名字
window.geometry("600x480")  # 設定視窗的大小(長 * 寬)
challenge_frame = tk.Frame(window) # 建立一個主frame，長在主window視窗上
challenge_frame.pack()  # 放置標籤
frame = tk.Frame(window)
frame.pack(side=tk.TOP)
field_frame = tk.Frame(window)
field_frame.pack(side=tk.TOP)
color_frame = tk.Frame(window)
color_frame.pack(side=tk.TOP)
color_frame2 = tk.Frame(window)
color_frame2.pack(side=tk.TOP)
nowcolor_frame = tk.Frame(window)
nowcolor_frame.pack(side=tk.TOP)
testframe = tk.Frame(window)
testframe.pack(side=tk.TOP)

comment = '請點選需要的顏色之後再進行HSV偵測或顏色遮罩，點\'clear\'將重置目前所選顏色，'
comment2 = '點\'reset\'將重置所有顏色，點\'showall\'可以在顏色遮照模式中顯示所有顏色'
l2 = tk.Label(window, text=comment2, font=('Arial', 12), ).pack(side=tk.BOTTOM)
l = tk.Label(window, text=comment, font=('Arial', 12), ).pack(side=tk.BOTTOM)
button = tk.Button(challenge_frame, text='挑戰一', fg='purple', command=challenge1)  # 在視窗介面設定放置Button按鍵
button2 = tk.Button(challenge_frame, text='挑戰二', fg='purple', command=challenge2)   # 說明： bg為背景，fg為字型顏色，font為字型，width為長，height為高
button3 = tk.Button(challenge_frame, text='挑戰三', fg='purple', command=challenge3)
button4 = tk.Button(frame, text='影像偵測', fg='blue', command=imagedetection)
button5 = tk.Button(frame, text='HSV偵測', fg='green', command=HSVdetection)
button6 = tk.Button(frame, text='顏色遮罩', fg='brown', command=ColorMask)
button7 = tk.Button(field_frame, text='場地座標', command=fieldlocation)
button8 = tk.Button(field_frame, text='中心點', command=fieldcenter)
button9 = tk.Button(field_frame, text='罰球點', command=fieldpenalty)
button10 = tk.Button(color_frame, text='red', fg='red', command=red)
button11 = tk.Button(color_frame, text='green', fg='green', command=green)
button12 = tk.Button(color_frame, text='blue', fg='blue', command=blue)
button13 = tk.Button(color_frame, text='purple', fg='purple', command=purple)
button14 = tk.Button(color_frame, text='pink', fg='pink', command=pink)
button15 = tk.Button(color_frame, text='orange', fg='orange', command=orange)
button16 = tk.Button(color_frame, text='yellow', fg='yellow', command=yellow)
button17 = tk.Button(color_frame2, text='clear', command=clear)
button18 = tk.Button(color_frame2, text='reset', command=reset)
button19 = tk.Button(color_frame2, text='showall', command=showall)
var = tk.StringVar()
l = tk.Label(nowcolor_frame, textvariable=var, bg='black', fg='white', font=('Arial', 12), width=15, height=2).pack()
button20 = tk.Button(testframe, text='test', command=hello)
button21 = tk.Button(color_frame2, text='機器人誤差調整',fg = 'red', command=test_error)

button.pack(side=tk.LEFT)  # 放置標籤
button2.pack(side=tk.LEFT)
button3.pack(side=tk.LEFT)
button4.pack(side=tk.LEFT)
button5.pack(side=tk.LEFT)
button6.pack(side=tk.LEFT)
button7.pack(side=tk.LEFT)
button8.pack(side=tk.LEFT)
button9.pack(side=tk.LEFT)
button10.pack(side=tk.LEFT)
button11.pack(side=tk.LEFT)
button12.pack(side=tk.LEFT)
button13.pack(side=tk.LEFT)
button14.pack(side=tk.LEFT)
button15.pack(side=tk.LEFT)
button16.pack(side=tk.LEFT)
button17.pack(side=tk.LEFT)
button18.pack(side=tk.LEFT)
button19.pack(side=tk.LEFT)
button20.pack(side=tk.LEFT)
button21.pack(side=tk.LEFT)

window.mainloop()  # 主視窗迴圈顯示
