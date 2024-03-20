# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import GoalID
from triangulation import get_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from transformation import get_tf

class Tag_Position():
    def __init__(self):
        rospy.init_node('tag_position', anonymous=True)

        ### sub ###
        rospy.Subscriber('uwb0_pub', Float32, self.anchor1_callback)
        rospy.Subscriber('uwb1_pub', Float32, self.anchor2_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        ### pub ###   
        self.taginmap = Marker()
        self.pub_TagInMap = rospy.Publisher("tag_in_map", Marker, queue_size=1)
        self.taginmap.header.frame_id = "map"
        self.taginmap.type = 2
        self.taginmap.id = 0
        self.taginmap.scale.x = 0.4
        self.taginmap.scale.y = 0.4
        self.taginmap.scale.z = 0.4
        self.taginmap.color.r = 0.0
        self.taginmap.color.g = 1.0
        self.taginmap.color.b = 0.0
        self.taginmap.color.a = 1.0

        self.goal_msg = PoseStamped()
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        self.turn = Twist()
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0

        self.cancel_goal = GoalID()
        self.cancel_publisher = rospy.Publisher('/move_base/cancel',GoalID,queue_size=1)

        #############
        self.init_callback_time = rospy.get_time()
        self.last_callback_time = rospy.get_time()  
        self.callback_interval = 1.0  # [s]
        self.dis_threshold = 1

        self.D1, self.D2 = 0.001, 0.001
        self.est_D1, self.est_D2 = 0.001, 0.001
        self.d1time, self.d2time = 0, 0
        self.dis_anchors = 0.9
        self.local_yaw = 0

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
        self.tag_pub()
        self.publish_current_goal()

    def anchor2_callback(self, data):
        self.D2 = data.data
        self.tag_pub()
        self.publish_current_goal()

    def tag_pub(self):
        tag_r =  get_pose(self.D1,self.D2,self.dis_anchors)
        self.tag_m, self.safe_goal, self.local_yaw = get_tf(tag_r,self.robot_m)

        self.taginmap.pose.position.x = self.tag_m[0][0]
        self.taginmap.pose.position.y = self.tag_m[1][0]
        self.taginmap.header.stamp = rospy.Time.now()
        self.pub_TagInMap.publish(self.taginmap)
   
        print(self.taginmap)

    def publish_current_goal(self):
        if math.hypot((self.robot_m[0] - self.tag_m[0][0]),(self.robot_m[1] - self.tag_m[1][0])) > self.dis_threshold:
            if rospy.get_time() - self.last_callback_time > self.callback_interval:
                self.local_yaw = math.atan2(self.tag_m[1][0]-self.robot_m[1],self.tag_m[0][0]-self.robot_m[0])
                rot = quaternion_from_euler(0,0,self.local_yaw)
                self.goal_msg.header.stamp = rospy.Time.now()
                self.goal_msg.header.frame_id = "map"

                self.goal_msg.pose.position.x = self.tag_m[0][0] - math.cos(self.local_yaw)
                self.goal_msg.pose.position.y = self.tag_m[1][0] - math.sin(self.local_yaw)
                self.goal_msg.pose.position.z = 0.0
                self.goal_msg.pose.orientation.x = rot[0]
                self.goal_msg.pose.orientation.y = rot[1]
                self.goal_msg.pose.orientation.z = rot[2]
                self.goal_msg.pose.orientation.w = rot[3]
                
                self.goal_publisher.publish(self.goal_msg)
                
                rospy.loginfo("MoveBaseSimpleGoal published: %s", self.goal_msg)
                
                self.last_callback_time = rospy.get_time()
        
        else:
            self.cancel_goal.stamp = rospy.Time.now()
            self.cancel_publisher.publish(self.cancel_goal)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tag_position = Tag_Position()
        tag_position.run()
    except rospy.ROSInterruptException:
        pass