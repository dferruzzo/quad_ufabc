#!/usr/bin/env python3
#
import rospy
import numpy as np
#from quad_ros import quad_robot
from quat_utils import QuatProd, Quat2Euler
from quad_control import Controller
#from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Quaternion, Vector3, Pose, Vector3Stamped, PoseStamped
from quad_ufabc.msg import\
    CartesianPointStamped, Pose, Velocity, Accel, Point,\
        Quaternion1, Num, Euler, PositionControllerOutputStamped,\
            AttitudeControllerOutputStamped, \
                AttitudeControllerErrorStamped

class Attitude_Control(Controller):
    
    """
    A classe herda os atributos e métodos da classe Controller e se subscribe aos seguintes tópicos:
        - /quad/kf/attitude
        - /quad/control/position_controller_output
    publica o  tópico 
        - /quad/control/attitude_controller_output (implementar)
    que é do tipo tipo quad_ufabc/AttitudeControllerOutputStamped (implementar)
    e que publica,
        - O tempo na forma de stamp,
        - Os torques de saída em phi, theta, psi 
    Nesta versão implemento o método 'COLOCAR AQUI O MÉTODO',
    Entradas: 
        - COLOCAR AQUI AS ENTRADAS DO MÉTODO. 
    Salidas:
        - COLOCAR AQUI AS SAÍDAS
    """    
    def __init__(self):
        super().__init__()
        # parâmeters
        self.orientation_atual = Quaternion1()
        self.orientation_atual_euler = Euler()
        self.pos_control_output = PositionControllerOutputStamped()
        self.attitude_error_quat = Quaternion1()
        self.attitude_error_euler = Euler()
        self.att_control_output = AttitudeControllerOutputStamped()
        self.att_control_error = AttitudeControllerErrorStamped()
        self.vel_angular_atual = Vector3()
        
        # Subscrição tópico de atitude
        sub_att_name = '/quad/kf/attitude'
        self.sub_att = rospy.Subscriber(sub_att_name,\
            Quaternion,\
                self.callback_attitude)
        
        sub_att_name = '/quad/kf/euler'
        self.sub_att = rospy.Subscriber(sub_att_name,\
            Euler,\
                self.callback_attitude_euler)
        
        # Subscrição tópico de velocidade angular
        sub_vel_ang_name = '/quad/kf/vel_ang'
        self.sub_vel_ang = rospy.Subscriber(sub_vel_ang_name,\
            Vector3,\
                self.callback_vel_ang)
        
        # Subscrição tópico 'saída do controlador de posição'
        sub_pos_cont_out = '/quad/control/position_controller_output'
        self.pos_cont_out = rospy.Subscriber(sub_pos_cont_out,\
            PositionControllerOutputStamped,\
                self.callback_pos_control_out)
        
        # Publicações
        pub_att_cont_out = '/quad/control/attitude_controller_output'
        self.pub_att_control_output = rospy.Publisher(pub_att_cont_out,\
            AttitudeControllerOutputStamped,\
                queue_size=10)  
        
        pub_att_control_error = '/quad/control/attitude_controller_error'
        self.pub_att_control_error = rospy.Publisher(pub_att_control_error,\
            AttitudeControllerErrorStamped,\
                queue_size=10)      
        
    def callback_attitude(self, data):
        # orientação atual
        # esse quaternion está na configuração (x,y,z,w)
        # e precisa ser rescrito na forma (w,x,y,z) que é a forma
        # utilizada no controle de atitude.
        self.orientation_atual.w = data.w
        self.orientation_atual.x = data.x
        self.orientation_atual.y = data.y
        self.orientation_atual.z = data.z
   
    def callback_attitude_euler(self, data):
        # orientação atual em ângulos de Euler
        self.orientation_atual_euler = data
    
    def callback_vel_ang(self, data):
        self.vel_angular_atual = data
        
    def callback_pos_control_out(self, data):
        self.pos_control_output = data
        # O erro de atitude em quaternions obtido do controle de posição e velocidade.
        self.attitude_error_quat =\
            self.pos_control_output.position_controller_output.orientation_set_point
        self.attitude_error_euler =\
                self.pos_control_output.position_controller_output.euler_set_point
        self.control()
    
    def control(self):
        # chama o controle de attitude 
        # publica os três torques
        """
        Controle de atitude em quaternions:
        att_control_quat(
                         quaternion_atual,
                         quaternion_desejado,
                         velocidade_angular_atual) -> (tau, error)
        """

        #tau, error = self.att_control_quat(
        #    self.quat_to_np_array(self.orientation_atual),
        #    self.quat_to_np_array(self.attitude_error_quat),
        #    self.point_to_np_array(self.vel_angular_atual))
        
        
        """
        Controle de atitude PD baseado em ângulos de Euler:
        att_control_PD(
                       ângulos_de Euler_atual, 
                       velocidade_angular_atual, 
                       ângulos_de_Euler_desejados) -> (tau, error)
        """
        
        tau, error = self.att_control_PD(
                self.euler_to_np_array(self.orientation_atual_euler),
                self.point_to_np_array(self.vel_angular_atual),
                self.euler_to_np_array(self.attitude_error_euler))

        self.att_control_output.torques.x = tau[0]
        self.att_control_output.torques.y = tau[1]
        self.att_control_output.torques.z = tau[2]
        self.att_control_output.header.stamp = rospy.Time.now()
        
        self.pub_att_control_output.publish(self.att_control_output)
        
        self.att_control_error.attitude_error.x = error[0]
        self.att_control_error.attitude_error.y = error[1]
        self.att_control_error.attitude_error.z = error[2]
        self.att_control_error.ang_vel_error.x = error[3]
        self.att_control_error.ang_vel_error.y = error[4]
        self.att_control_error.ang_vel_error.z = error[5]
        self.att_control_error.header.stamp = rospy.Time.now()
        
        self.pub_att_control_error.publish(self.att_control_error)
        
if __name__ == '__main__':
    try:
        node_name = 'attitude_controller_2'
        rospy.init_node(node_name)
        Attitude_Control()        
        rate = rospy.Rate(50) # 50hz
        rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    
