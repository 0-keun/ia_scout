import math
import numpy as np

def get_tf(tag_r,robot_m):
    if tag_r[0] != None and tag_r[1] != None:
        
        yaw = robot_m[3] # robot's yaw angle in the "map" frame

        ## yaw angle of robot in the "map" frame
        rotation_mat = np.array(
            [
                [math.cos(yaw), -1*math.sin(yaw)],
                [math.sin(yaw), math.cos(yaw)]
            ]
        )

        ## position of robot in the "map" frame
        translation_mat = np.array(
            [
                [robot_m[0]],
                [robot_m[1]]
            ]
        )

        ## tag position in the "base_link" frame
        tag = np.array(
            [
                [tag_r[0]],
                [tag_r[1]]
            ]
        )

        ## real goal is before 1m from tag's position
        D = 1 #[m]
        theta = math.atan2((tag_r[0]-robot_m[0])/(tag_r[1]-robot_m[1]))
        safety_distance = np.array(
            [
                [D*math.cos(theta)],
                [D*math.sin(theta)]
            ]
        )

        tag_m = np.matmul(rotation_mat, tag)

        tag_m = tag_m + translation_mat
        safe_goal = tag_m - safety_distance

    else:
        tag_m = np.array(
            [
                [robot_m[0]],
                [robot_m[1]]
            ]
        )   
        safe_goal = tag_m

    return tag_m, safe_goal