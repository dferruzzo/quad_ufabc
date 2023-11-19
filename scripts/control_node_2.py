#!/usr/bin/env python3
"""
TODO:
19/11/2023
[ ] habilitar o controle 
[ ] garantir taxa de atualização
[ ] é ppossível que precise de dois filtros de Kalman, um para posição, velocidade e aceleração e um outro para a dinâmica rotacional, ambos a frequências diferentes.
"""
import rospy
import pickle
import os, sys
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

#from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats

from quad_ros import quad_robot
from quat_utils import QuatProd
from quad_control import Controller
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3, Quaternion, Pose, Vector3Stamped, PoseStamped
from quad_ufabc.msg import Vector4

from sensor_msgs.msg import Imu

class Control:
    
    def __init__(self) -> None:
        
        self.attitude = Quaternion()
        self.position = Vector3()
        self.velocity = Vector3()
        self.control_output = Vector4()
        
        sub_att_name = '/quad/kf/attitude'
        sub_pos_name = '/quad/kf/position'
        sub_vel_name = '/quad/kf/vel'
        pub_control_output_name = '/quad/control/output'

        self.sub_att = rospy.Subscriber(sub_att_name, Quaternion, self.callback_attitude)
        self.pos_att = rospy.Subscriber(sub_pos_name, Vector3, self.callback_position)
        self.sub_vel = rospy.Subscriber(sub_vel_name, Vector3, self.callback_velocity)
        
        self.pub_control_output = rospy.Publisher(pub_control_output_name, Vector4,queue_size=10)
        
    def callback_position(self, data):
        """
        O loop de posição deveria atualizar a 10hz
        """
        self.position = data
    
    def callback_velocity(self, data):
        """
        O loop de velocidade deveria atualizar a 10hz
        """
        self.velocity = data
        
    def callback_attitude(self, data):
        """
        O loop de attitude deveria atualizar a 400hz
        """
        self.attitude = data
        """
        Aqui é chamado o algoritmo de controle que deverá publicar as quatro saídas de controle, o thrust, e os três torques.
        Idealmente o quadrotor deveria se subscriber nessa publicação e atualizar as velocidades angulares dos motores.
        """
        self.control_output.thrust = 1.0
        self.control_output.torque_phi = 0.1
        self.control_output.torque_theta = 0.2
        self.control_output.torque_psi = 0.3       
        
        self.pub_control_output.publish(self.control_output)

if __name__ == '__main__':
    node_name = 'controller_2'
    rospy.init_node(node_name)
    Control()
    rate = rospy.Rate(10) # 10hz
    rate.sleep()
    rospy.spin()
    try:
        Control()
    except rospy.ROSInterruptException:
        pass    
    
         
    