# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32
from filter import Kalman_Filter, MovAvg_Filter
from triangulation import get_pose
from tf.transformations import euler_from_quaternion
import numpy as np
import math
from LinearRegression import LR_poly
from transformation import get_tf

D1_poly = LR_poly()
D2_poly = LR_poly()

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
        
        self.tag_r = PoseStamped()
        self.tag_r.header.frame_id = "base_link"

        self.taginmap = PoseStamped()
        self.taginmap.header.frame_id = "map"

        self.goal_msg = PoseStamped()
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        #############
        self.init_callback_time = rospy.get_time()
        self.last_callback_time = rospy.get_time()  
        self.callback_interval = 0.5  # [s]
        self.dis_threshold = 1

        self.D1, self.D2 = 0.001, 0.001
        self.est_D1, self.est_D2 = 0.001, 0.001
        self.d1time, self.d2time = 0, 0
        self.dis_anchors = 0.9

        self.robot_m = [0,0,0,0]

    def amcl_pose_callback(self, data):

        P_x = data.pose.pose.position.x
        P_y = data.pose.pose.position.y
        P_z = data.pose.pose.position.z
    
        O_x = data.pose.pose.orientation.x
        O_y = data.pose.pose.orientation.y
        O_z = data.pose.pose.orientation.z
        O_w = data.pose.pose.orientation.w

        self.rotation = [O_x,O_y,O_z,O_w]

        [R, P, Y] = euler_from_quaternion(self.rotation)

        self.robot_m = [P_x,P_y,P_z,Y]

    def anchor1_callback(self, data):
        self.D1 = data.data

        # self.d1time = rospy.get_time() - self.init_callback_time
        # D1_poly.fit_model(self.d1time,self.D1)
        # self.est_D2 = D2_poly.estimate_data(self.d1time)
        # self.est_D1 = self.D1

        self.tag_pub()
        self.publish_current_goal()

    def anchor2_callback(self, data):
        self.D2 = data.data

        # self.d2time = rospy.get_time() - self.init_callback_time
        # D2_poly.fit_model(self.d2time,self.D2)
        # self.est_D1 = D1_poly.estimate_data(self.d2time)
        # self.est_D2 = self.D2

        self.tag_pub()
        self.publish_current_goal()

    def tag_pub(self):
        #################### base_link ####################
        [y_tag_r,x_tag_r] = get_pose(self.D1,self.D2,self.dis_anchors)
    
        self.tag_r.pose.position.x = x_tag_r
        self.tag_r.pose.position.y = y_tag_r
        self.tag_r.header.stamp = rospy.Time.now()
        self.pub_TagInBaselink.publish(self.tag_r)

        ###################### MAP #######################

        tag_r = [x_tag_r,y_tag_r]
        self.tag_m, self.safe_goal = get_tf(tag_r,self.robot_m)

        self.taginmap.pose.position.x = self.tag_m[0][0]
        self.taginmap.pose.position.y = self.tag_m[1][0]
        self.taginmap.header.stamp = rospy.Time.now()
        self.pub_TagInMap.publish(self.taginmap)
   
        print(self.taginmap)

    def publish_current_goal(self):
        if math.hypot((self.robot_m[0] - self.tag_m[0][0]),(self.robot_m[1] - self.tag_m[1][0])) > self.dis_threshold:
            if rospy.get_time() - self.last_callback_time > self.callback_interval:
        
                self.goal_msg.header.stamp = rospy.Time.now()
                self.goal_msg.header.frame_id = "map"

                self.goal_msg.pose.position.x = self.safe_goal[0][0]
                self.goal_msg.pose.position.y = self.safe_goal[1][0]
                self.goal_msg.pose.position.z = 0.0
                #self.goal_msg.pose.orientation.x = self.rotation[0]
                #self.goal_msg.pose.orientation.y = self.rotation[1]
                #self.goal_msg.pose.orientation.z = self.rotation[2]
                #self.goal_msg.pose.orientation.w = self.rotation[3]

                self.goal_publisher.publish(self.goal_msg)
                
                rospy.loginfo("MoveBaseSimpleGoal published: %s", self.goal_msg)
                
                self.last_callback_time = rospy.get_time()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tag_position = Tag_Position()
        tag_position.run()
    except rospy.ROSInterruptException:
        pass
