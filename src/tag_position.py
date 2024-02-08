# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32
from filter import Kalman_Filter, MovAvg_Filter
from triangulation import get_pose
from tf.transformations import euler_from_quaternion
import numpy as np
import math

KF1 = Kalman_Filter()
KF2 = Kalman_Filter()

MV1 = MovAvg_Filter()
MV2 = MovAvg_Filter()


class Tag_Position():
    def __init__(self):
        rospy.init_node('tag_position', anonymous=True)

        ### sub ###
        rospy.Subscriber('uwb0_pub', Float32, self.anchor1_callback)
        rospy.Subscriber('uwb1_pub', Float32, self.anchor2_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        ### pub ###   
        self.pub_TagInBaselink = rospy.Publisher("tag_pose", PoseStamped, queue_size=1)
        self.pub_TagInMap = rospy.Publisher("tag_in_map", PoseStamped, queue_size=1)
        
        position_tag = PoseStamped()
        position_tag.header.frame_id = "base_link"
        self.position_tag = position_tag

        taginmap = PoseStamped()
        taginmap.header.frame_id = "map"
        self.taginmap = taginmap

        #### pub ####
        self.goal_msg = PoseStamped()
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        #############

        self.last_callback_time = rospy.get_time()  
        self.callback_interval = 0.5  # [s]
        self.current_position = [0,0,0]
        self.dis_threshold = 1

        self.D1, self.D2 = 0.001, 0.001
        self.dis_anchors = 0.9
        self.Y = 0
        self.flag = "GO"
        self.translation = [0,0,0]
        self.rotation = [0,0,0,1]

    def get_tf(self, theta):
        if self.position_tag.pose.position.x != None and self.position_tag.pose.position.y != None:
            ## yaw angle of robot in the "map" frame
            rotation_mat = np.array(
                [
                    [math.cos(theta), -1*math.sin(theta)],
                    [math.sin(theta), math.cos(theta)]
                ]
            )

            ## position of robot in the "map" frame
            translation_mat = np.array(
                [
                    [self.translation[0]],
                    [self.translation[1]]
                ]
            )

            ## tag position in the "base_link" frame
            xy = np.array(
                [
                    [self.position_tag.pose.position.x],
                    [self.position_tag.pose.position.y]
                ]
            )

            x_y_ = np.matmul(rotation_mat, xy)

            x_y_ = x_y_ + translation_mat

        else:
            x_y_ = np.array(
                [
                    [self.translation[0]],
                    [self.translation[1]]
                ]
            )   

        return x_y_

    def amcl_pose_callback(self, data):

        ################# Get amcl_pose #################

        P_x = data.pose.pose.position.x
        P_y = data.pose.pose.position.y
        P_z = data.pose.pose.position.z
    
        O_x = data.pose.pose.orientation.x
        O_y = data.pose.pose.orientation.y
        O_z = data.pose.pose.orientation.z
        O_w = data.pose.pose.orientation.w

        self.translation = [P_x,P_y,P_z]
        self.rotation = [O_x,O_y,O_z,O_w]

        [self.R, self.P, self.Y] = euler_from_quaternion(self.rotation)

    def anchor1_callback(self, data):
        # self.D1 = MV1.mov_avg_filter(data.data)
        self.D1 = data.data

    def anchor2_callback(self, data):
        #self.D2 = MV2.mov_avg_filter(data.data)
        self.D2 = data.data
        self.tag_pub()
        if math.sqrt(math.pow(self.translation[0] - self.tag_in_map[0][0],2) + math.pow(self.translation[1] - self.tag_in_map[1][0],2)) > self.dis_threshold:
            if rospy.get_time() - self.last_callback_time > self.callback_interval:
                self.publish_current_goal()
                self.last_callback_time = rospy.get_time()
        else:
            self.stop()

    def tag_pub(self):
        #################### base_link ####################
        [w,h] = get_pose(self.D1,self.D2,self.dis_anchors)
    
        self.position_tag.pose.position.x = h
        self.position_tag.pose.position.y = w
        self.position_tag.header.stamp = rospy.Time.now()
        self.pub_TagInBaselink.publish(self.position_tag)

        ###################### MAP #######################

        self.tag_in_map = self.get_tf(self.Y)

        self.taginmap.pose.position.x = self.tag_in_map[0][0]
        self.taginmap.pose.position.y = self.tag_in_map[1][0]
        self.taginmap.header.stamp = rospy.Time.now()
        self.pub_TagInMap.publish(self.taginmap)

    def publish_current_goal(self):

        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.header.frame_id = "map"

        self.goal_msg.pose.position.x = self.tag_in_map[0][0]
        self.goal_msg.pose.position.y = self.tag_in_map[1][0]
        self.goal_msg.pose.position.z = 0.0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = 1.0
        self.goal_msg.pose.orientation.w = 0

        self.goal_publisher.publish(self.goal_msg)
        self.flag = "GO"
        
        rospy.loginfo("MoveBaseSimpleGoal published: %s", self.goal_msg)

    def stop(self):
        if self.flag == "GO":
            self.goal_msg.header.stamp = rospy.Time.now()
            self.goal_msg.header.frame_id = "map"

            self.goal_msg.pose.position.x = self.translation[0]
            self.goal_msg.pose.position.y = self.translation[1]
            self.goal_msg.pose.position.z = 0.0
            self.goal_msg.pose.orientation.x = 0
            self.goal_msg.pose.orientation.y = 0
            self.goal_msg.pose.orientation.z = 1.0
            self.goal_msg.pose.orientation.w = 0

            self.goal_publisher.publish(self.goal_msg)  
            self.flag = "STOP"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tag_position = Tag_Position()
        tag_position.run()
    except rospy.ROSInterruptException:
        pass
