#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math

class goal_publish:
    def __init__(self):
        rospy.init_node('goal_publish', anonymous=True)

        rospy.Subscriber('/tag_pose', PoseStamped, self.tag_pose_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        #### pub ####
        self.goal_msg = PoseStamped()
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        #############

        self.last_callback_time = rospy.get_time()  
        self.callback_interval = 0.1  # [s]
        self.current_position = [0,0,0]
        self.dis_threshold = 1

    def tag_pose_callback(self, data):
        current_time = rospy.get_time()

        P_x = data.pose.position.x
        P_y = data.pose.position.y

        self.tag_positions = [P_x,P_y]
        print("callback check")
        if self.tag_positions[0] + self.tag_positions[1]  - self.amcl_positions[0] +self.amcl_positions[1] > self.dis_threshold:
            if current_time - self.last_callback_time >= self.callback_interval:
                self.publish_current_goal()
                self.last_callback_time = current_time
        else:
            self.stop()

    def amcl_pose_callback(self, data):

        P_x = data.pose.pose.position.x
        P_y = data.pose.pose.position.y
        P_z = data.pose.pose.position.z
        
        O_x = data.pose.pose.orientation.x
        O_y = data.pose.pose.orientation.y
        O_z = data.pose.pose.orientation.z
        O_w = data.pose.pose.orientation.w

        self.amcl_positions = [P_x,P_y]
        self.amcl_orientations = [O_x,O_y,O_z,O_w]
    
    def publish_current_goal(self):

        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.header.frame_id = "base_link"

        self.goal_msg.pose.position.x = self.tag_positions[0]
        self.goal_msg.pose.position.y = self.tag_positions[1]
        self.goal_msg.pose.position.z = 0.0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = 1.0
        self.goal_msg.pose.orientation.w = 0

        self.goal_publisher.publish(self.goal_msg)
        
        # rospy.loginfo("MoveBaseSimpleGoal published: %s", self.goal_msg)

    def stop(self):
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.header.frame_id = "base_link"

        self.goal_msg.pose.position.x = 0
        self.goal_msg.pose.position.y = 0

        self.goal_publisher.publish(self.goal_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        Goal_pub = goal_publish()
        Goal_pub.run()
    except rospy.ROSInterruptException:
        pass
