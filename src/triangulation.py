import math
from filter import MovAvg_Filter, Kalman_Filter

MAF = MovAvg_Filter()
KF1 = MovAvg_Filter()
KF2 = MovAvg_Filter()

def get_angle(a,b,c): # a, b, c are length of triangle
    a = KF1.mov_avg_filter(a)
    b = KF2.mov_avg_filter(b)
    
    tem = (math.pow(b,2) + math.pow(c,2) - math.pow(a,2)) / (2*b*c)
    print("D1:",a,"D2",b)
    print('cos(angle):', tem)
    if tem > 0.9999:
        tem = 0.9999
    elif tem > 0.995:
        tem = 0.995
    elif tem > 0.984:
        tem = 0.984

    if tem < -0.9999:
        tem = -0.9999
    elif tem < -0.995:
        tem = -0.995
    elif tem < -0.984:
        tem = -0.984

    ang = MAF.mov_avg_filter(math.acos(tem))
    # ang = math.acos(tem)
    return ang

def get_wh(D1,A,dis_anchors):
    h = D1 * math.sin(A)
    w = D1 * math.cos(A) - (dis_anchors/2)
    return [w,h]

def Pythagoras(hypotenuse,height):
    if hypotenuse > height:
        d = math.pow(hypotenuse,2) - math.pow(height,2)
        length = math.sqrt(d)
    else:
        length = 0

    return length


def get_pose(D1,D2,dis_anchors):
    # a = D2, b = D1
    dif_h = 0.2

    if D1 == None or D2 ==None:
        print("UWB0 DATA is None")
        return [0.001,0.001]
    
    else:
        #D1 = Pythagoras(D1,dif_h)
        #D2 = Pythagoras(D2,dif_h)

        A = get_angle(D2,D1,dis_anchors)

        return get_wh(D1,A,dis_anchors)

