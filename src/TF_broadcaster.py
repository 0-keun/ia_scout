# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros

class TF_Broadcaster():
    def __init__(self):
        rospy.init_node('TF_broadcaster', anonymous=True)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
    
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def amcl_pose_callback(self, data):

        ################# Get amcl_pose #################
        
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = data.pose.pose.position.x
        t.transform.translation.y = data.pose.pose.position.y
        t.transform.translation.z = data.pose.pose.position.z
        t.transform.rotation.x = data.pose.pose.orientation.x
        t.transform.rotation.y = data.pose.pose.orientation.y
        t.transform.rotation.z = data.pose.pose.orientation.z
        t.transform.rotation.w = data.pose.pose.orientation.w

        # P_x = data.pose.pose.position.x
        # P_y = data.pose.pose.position.y
        # P_z = data.pose.pose.position.z
        
        # O_x = data.pose.pose.orientation.x
        # O_y = data.pose.pose.orientation.y
        # O_z = data.pose.pose.orientation.z
        # O_w = data.pose.pose.orientation.w

        # self.positions = [P_x,P_y,P_z]
        # self.orientations = [O_x,O_y,O_z,O_w]

        ################# TF_Broadcaster #################

        self.tf_broadcaster.sendTransform(t)
        print(1)

        ################# print #################

        # print("amcl_positions   :", self.positions)
        # print("amcl_orientations:", self.orientations)

        #########################################

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tf_b = TF_Broadcaster()
        tf_b.run()
    except rospy.ROSInterruptException:
        pass

