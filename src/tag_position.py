# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from triangulation import get_pose
from std_msgs.msg import Float32
from filter import Kalman_Filter, MovAvg_Filter

KF1 = Kalman_Filter()
KF2 = Kalman_Filter()

MV1 = MovAvg_Filter()
MV2 = MovAvg_Filter()

class Tag_Position:
    def __init__(self):
        rospy.init_node('tag_position', anonymous=True)
        rospy.Subscriber('uwb0_pub', Float32, self.anchor1_callback)
        rospy.Subscriber('uwb1_pub', Float32, self.anchor2_callback)
        
        
        self.pub = rospy.Publisher("tag_pose", PoseStamped, queue_size=1)
        
        self.D1, self.D2 = 0.001, 0.001
        position_tag = PoseStamped()
        position_tag.header.frame_id = "base_link"
        self.position_tag = position_tag
        self.dis_anchors = 0.9
    def anchor1_callback(self, data):
        # self.D1 = MV1.mov_avg_filter(data.data)
        self.D1 = data.data
    def anchor2_callback(self, data):
        #self.D2 = MV2.mov_avg_filter(data.data)
        self.D2 = data.data
        self.tag_pub()

    def tag_pub(self):
        [w,h] = get_pose(self.D1,self.D2,self.dis_anchors)
        #w=KF1.kalman_filter(w)
        #h=KF2.kalman_filter(h)
        #w = MV1.mov_avg_filter(w)
        #h = MV2.mov_avg_filter(h)    
    
        self.position_tag.pose.position.x = h
        self.position_tag.pose.position.y = w
        self.position_tag.header.stamp = rospy.Time.now()
        self.pub.publish(self.position_tag)	

        print("=================")
        print(self.D1,"/",self.D2)
        print(w,"/",h)
        print("=================")

            
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tag_position = Tag_Position()
        tag_position.run()
    except rospy.ROSInterruptException:
        pass
