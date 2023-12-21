#!/usr/bin/env python3
#
import rospy
import numpy as np
from quad_ros import quad_robot
from quat_utils import QuatProd
from quad_control import Controller
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Quaternion, Vector3, Pose, Vector3Stamped, PoseStamped
from quad_ufabc.msg import CartesianPointStamped, Pose, Velocity, Accel, Point, Quaternion1, Num, PositionControllerOutputStamped

from sensor_msgs.msg import Imu

class Pos_Vel_Control:
    
    def __init__(self) -> None:
        
        self.pos_atual = Vector3()              # posição atual
        self.vel_atual = Vector3()              # velocidade atual
        self.orientation_atual = Quaternion()   # orientação atual
        
        self.pos_des = Point()                  # posição desejada
        self.vel_des = Point()                  # velocidade desejada
        self.orientation_des = Quaternion1()    # orientação desejada
                
        self.controller = Controller()          # Instancia o controlador 
        
        self.pos_control_output = PositionControllerOutputStamped()
        
        self.desired_trajectory = CartesianPointStamped()   # valores de trajetória desejados
        
        sub_att_name = '/quad/kf/attitude'
        sub_pos_name = '/quad/kf/position'      # nome do tópico de posição
        sub_vel_name = '/quad/kf/vel'           # nome do tópico de velocidade
        sub_tra_name = '/desired_trajectory'    # nome do tópico de trajetória desejada

        pub_pos_cont_out_name = '/quad/control/position_controller_output'

        self.sub_att = rospy.Subscriber(sub_att_name, Quaternion, self.callback_attitude)
        self.sub_pos = rospy.Subscriber(sub_pos_name, Vector3, self.callback_position)
        self.sub_vel = rospy.Subscriber(sub_vel_name, Vector3, self.callback_velocity)
        self.sub_tra = rospy.Subscriber(sub_tra_name, CartesianPointStamped, self.callback_trajectory)
        
        self.pub_pos_control_output = rospy.Publisher(pub_pos_cont_out_name, PositionControllerOutputStamped, queue_size=10)        
        
    def callback_position(self, data):
        # a posição atual
        self.pos_atual.x = data.x
        self.pos_atual.y = data.y
        self.pos_atual.z = data.z            
    
    def callback_velocity(self, data):
        # a velocidade atual
        self.vel_atual.x = data.x
        self.vel_atual.y = data.y
        self.vel_atual.z = data.z        

    def callback_attitude(self, data):
        # orientação atual
        self.orientation_atual = data
        pass       
        
    def callback_trajectory(self, data):
        self.pos_des = data.cartesian_point.pose.position
        self.vel_des = data.cartesian_point.velocity.linear
        self.orientation_des = data.cartesian_point.pose.orientation
        #
        self.control()      
    
    def control(self):
        #       
        T, q_erro = self.controller.pos_control_quat_v1(
            self.point_to_np_array(self.pos_atual),
            self.point_to_np_array(self.pos_des),
            self.point_to_np_array(self.vel_atual),
            self.point_to_np_array(self.vel_des),
            self.quat_to_np_array(self.orientation_des))
        # T é o empuxo total
        # q_pdes é o quaternio de atitude desejado correspondente
        self.pos_control_output.position_controller_output.thrust.num = T
        self.pos_control_output.position_controller_output.orientation_set_point.w = q_erro[0]
        self.pos_control_output.position_controller_output.orientation_set_point.x = q_erro[1]
        self.pos_control_output.position_controller_output.orientation_set_point.y = q_erro[2]
        self.pos_control_output.position_controller_output.orientation_set_point.z = q_erro[3]     
        
        self.pos_control_output.header.stamp = rospy.Time.now()

        self.pub_pos_control_output.publish(self.pos_control_output)
        
    def point_to_np_array(self, point):
        return np.array([[point.x, point.y, point.z]]).T
    
    def quat_to_np_array(self, quat):
        return np.array([[quat.w, quat.x, quat.y, quat.z]]).T

if __name__ == '__main__':
    try:
        node_name = 'controller_2'
        rospy.init_node(node_name)
        Pos_Vel_Control()        
        rate = rospy.Rate(50) # 50hz
        rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    
    
         
    