#!/usr/bin/env python3
import rospy
#import numpy as np
#from quad_ros import quad_robot
#from sensor_msgs.msg import  Imu
from geometry_msgs.msg import Quaternion, Vector3, Pose, Twist
from gazebo_msgs.msg import ModelStates

class EKF:
    
    def __init__(self):
        pub_att_name = '/quad/kf/attitude'        
        pub_pos_name = '/quad/kf/position'
        pub_vel_name = '/quad/kf/vel'
        sub_sta_name = '/gazebo/model_states'
        
        self.pub_att = rospy.Publisher(pub_att_name, Quaternion, queue_size=10)
        self.pub_pos = rospy.Publisher(pub_pos_name, Vector3, queue_size=10)        
        self.pub_vel = rospy.Publisher(pub_vel_name, Vector3, queue_size=10)    
        self.sub_sta = rospy.Subscriber(sub_sta_name, ModelStates, self.states_callback)
        self.quad_pose = Pose()
        self.quad_twist = Twist()        
        
    def states_callback(self, data):
        # Quad pose, representa a posição e a attitude em quaternion
        self.quad_pose = data.pose[2]    
        # Quad twist representa a velocidade linear e angular
        self.quad_twist = data.twist[2]    
        #
        self.pub_att.publish(self.quad_pose.orientation)
        self.pub_pos.publish(self.quad_pose.position)
        self.pub_vel.publish(self.quad_twist.angular)

if __name__ == '__main__':
    node_name = 'estimator'
    rospy.init_node(node_name)
    EKF()
    rate = rospy.Rate(10) # 10hz
    rate.sleep()
    rospy.spin()
    try:
        EKF()
    except rospy.ROSInterruptException:
        pass