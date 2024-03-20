import math
from filter import MovAvg_Filter

MAF = MovAvg_Filter()

def get_angle(a,b,c): # a, b, c are length of triangle
    tem = (b**2 + c**2 - a**2) / (2*b*c)
    tem = 0.83 * tem 
    ang = MAF.mov_avg_filter(math.acos(tem))
    return ang

def get_wh(D1,A,dis_anchors):
    h = D1 * math.sin(A)
    w = D1 * math.cos(A) - (dis_anchors/2)
    return [h,w]

def get_pose(D1,D2,dis_anchors):
    if D1 == None or D2 == None:
        print("UWB0 DATA is None")
        return [0.001,0.001]
    
    else:

        A = get_angle(D2,D1,dis_anchors)

        return get_wh(D1,A,dis_anchors)

