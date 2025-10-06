import math
def vector_param(p1_xy , p2_xy): 
    p1_p2_ux , p1_p2_uy = 0 , 0
    p1_p2_length = 0
    try:
        #p1_xy -> p2_xy
        p1_p2_x , p1_p2_y = 0,0
        p1_p2_x , p1_p2_y =  p2_xy[0] - p1_xy[0] , p2_xy[1] - p1_xy[1] #vector
        p1_p2_length = math.hypot( p1_p2_x ,  p1_p2_y) #vector length
        p1_p2_ux , p1_p2_uy =  p1_p2_x/ p1_p2_length ,  p1_p2_y/ p1_p2_length #unit vector    
    except ZeroDivisionError:
        p1_p2_length = 0            
        #print('skip a Zero Division Error in the vector_param calculate')  
    return  [p1_p2_ux , p1_p2_uy] , p1_p2_length

def vector_product(p1_x, p1_y, p2_x=0, p2_y=0):
    ValueErrorFlag = 0
    cro = p1_x * p2_y - p1_y * p2_x
    dot = p1_x * p2_x + p1_y * p2_y
    angle = 0
    try:
        angle = math.acos(dot)
        angle_degree = angle * 180 / math.pi #角度(0<=degree<=180)
    except  ValueError:
        #print('ValueError')
        ValueErrorFlag = 1
        if dot > 1:
            angle_degree = 0.0
        else: #dot <= -1
            angle_degree = 180.0   
        return cro,dot,angle_degree,ValueErrorFlag
    return cro,dot,angle_degree,ValueErrorFlag

def rotate_vector(vector,angle_degree): #CLOCKWISE
    '''
    假設在平面上有一點 (x, y)，則以原點為中心，逆時針方向旋轉 θ 後，其座標 (x', y') 與 原座標點 (x, y) 的關係為： 
    x' = cos(θ) * x - sin(θ) * y 
    y' = sin(θ) * x + cos(θ) * y
    '''
    angle_rad = angle_degree * math.pi / 180
    new_vector = [0,0]
    new_vector[0] = math.cos(angle_rad) * vector[0] - math.sin(angle_rad) * vector[1]
    new_vector[1] = math.sin(angle_rad) * vector[0] + math.cos(angle_rad) * vector[1]
    return new_vector[0],new_vector[1]

def vector_angle(vector1,vector2): #compute angle between 2 vectors(it doesn't have to be unit) #vector1->vector2
    try:
        vector1_length = math.hypot(vector1[0],vector1[1]) #vector1 length
        vector2_length = math.hypot(vector2[0],vector2[1]) #vector2 length
        vector1_uxy = [0,0]
        vector2_uxy = [0,0]
        vector1_uxy[0] , vector1_uxy[1] = vector1[0] / vector1_length , vector1[1] / vector1_length
        vector2_uxy[0] , vector2_uxy[1] = vector2[0] / vector2_length , vector2[1] / vector2_length
        cross,dot,angle180,err = vector_product(vector1_uxy[0],vector1_uxy[1],vector2_uxy[0],vector2_uxy[1])
        #angle(0<=degree<360)
        if cross <= 0:
            angle360 = angle180 
        else:
            angle360 = 360 - angle180
    except ZeroDivisionError:
        angle180 , angle360 = 0 , 0
        #print('skip a Zero Division Error in the vector_param calculate')  
    return angle180 , angle360

def full_angle(pos1,pos2,pos3): #compute angle from position1 to position2 compared to position3
    pos1,pos2,pos3 = [pos1[0],pos1[1]],[pos2[0],pos2[1]],[pos3[0],pos3[1]]
    pos31_uxy , pos31_length = vector_param(pos3,pos1)
    pos32_uxy , pos32_length = vector_param(pos3,pos2) 
    cross,dot,angle180,err = vector_product(pos31_uxy[0],pos31_uxy[1],pos32_uxy[0],pos32_uxy[1])
    #angle(0<=degree<360)
    if cross <= 0:
        angle360 = angle180 
    else:
        angle360 = 360 - angle180
    return angle180 , angle360
