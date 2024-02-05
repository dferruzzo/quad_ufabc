#!/usr/bin/env python3
import rospy
import numpy as np
#from quad_ros import quad_robot
#from sensor_msgs.msg import  Imu
from geometry_msgs.msg import Quaternion, Vector3, Pose, Twist
from gazebo_msgs.msg import ModelStates
from quat_utils import Quat2Euler
from quad_ufabc.msg import Euler

class EKF:
    # TODO: Implementar o filtro de Kalman
    def __init__(self):
        pub_att_name = '/quad/kf/attitude'
        pub_eul_name = '/quad/kf/euler'
        pub_pos_name = '/quad/kf/position'
        pub_vel_lin_name = '/quad/kf/vel_lin'
        pub_vel_ang_name = '/quad/kf/vel_ang'
        
        sub_sta_name = '/gazebo/model_states'
        
        self.quad_pose = Pose()
        self.quad_twist = Twist()  
        self.quad_euler = Euler()
        self.q = np.zeros((4,1)) # para convertir Pose() que é um quaternion em um vetor(4,1)
        self.euler = np.zeros((3,1))
        
        self.pub_att = rospy.Publisher(pub_att_name, Quaternion, queue_size=10)
        self.pub_eul = rospy.Publisher(pub_eul_name, Euler, queue_size=10)    
        self.pub_pos = rospy.Publisher(pub_pos_name, Vector3, queue_size=10)        
        self.pub_vel_lin = rospy.Publisher(pub_vel_lin_name, Vector3, queue_size=10)   
        self.pub_vel_ang = rospy.Publisher(pub_vel_ang_name, Vector3, queue_size=10) 
        
        self.sub_sta = rospy.Subscriber(sub_sta_name, ModelStates, self.states_callback)             
        
    def states_callback(self, data):
        # Quad pose, representa a posição e a attitude em quaternion
        self.quad_pose = data.pose[2]    
        
        # quad_pose.orientation representa a attitute em quateernion
        self.q[0] = self.quad_pose.orientation.w
        self.q[1] = self.quad_pose.orientation.x
        self.q[2] = self.quad_pose.orientation.y
        self.q[3] = self.quad_pose.orientation.z
        # convertir o quaternio de attitude para angulos de Euler
        self.euler = Quat2Euler(self.q)
        # passar para ros msg 
        self.quad_euler.phi = self.euler[0]
        self.quad_euler.theta = self.euler[1]
        self.quad_euler.psi = self.euler[2]
        
        # Quad twist representa a velocidade linear e angular
        self.quad_twist = data.twist[2]
        self.pub_att.publish(self.quad_pose.orientation)
        self.pub_eul.publish(self.quad_euler)
        self.pub_pos.publish(self.quad_pose.position)
        self.pub_vel_lin.publish(self.quad_twist.linear)
        self.pub_vel_ang.publish(self.quad_twist.angular)

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