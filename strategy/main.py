import cv2
import math
import numpy as np


# 建立機器人物件(打者與對手)  # initialize(需要給模擬器的資訊， facing direction 、position.....)

# 紀錄場地資訊變數(ball....)
global  BOUNDARY, CENTER
BOUNDARY = [(140, 70), (870, 70), (870, 170), (900, 170), (900, 350), (870, 350), (870, 450), (140, 450), (140, 350), (110, 350), (110, 170), (140, 170)]
CENTER = [0, 0] #list[int]=[cx,cy] 紀錄中心位置

def strategy_update_field(side, boundary, center, pk_x, fb_x, fb_y, penalty_y, ga_x, ga_y): # 取得初始場地資訊(只在一開始呼叫一次)
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
    pass

def Initialize():
    """
    Description:
        Initialuze the strategy.
        This function will be called by the simulator before a simulation is started.
    """
    # Your code
    pass

def draw_on_simulator(frame):# 畫場地(球、其他所需呈現在模擬器上方便觀察的資訊)
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
    return frame

def Update_Robo_Info(team_d, team_p, oppo_p, ball_p):# 更新資訊
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
    pass

def strategy():
    """
    Description:
        The simulator will ask for strategy after calling Update_Robo_Info()
    Return:
        retva1: list[str] -> command for each robot
    """
    # Your code
    return ['N1','N1','N1']
    pass
    

def get_sent_cmd(sentcmd, update):
    """
    Description:
        Simulator will pass the received strategy and a sending state
    Parameter:
        param1: list[str] -> received command
        param2: bool -> sent or not
    """
    # Your code
    pass
