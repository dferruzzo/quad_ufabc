#!/usr/bin/env python3
#
import rospy
import numpy as np
#from quad_ros import quad_robot
from quat_utils import QuatProd, Quat2Euler
from quad_control import Controller
#from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Quaternion, Vector3, Pose, Vector3Stamped, PoseStamped
from quad_ufabc.msg import CartesianPointStamped, Pose, Velocity, Accel, Point, Quaternion1, Num, Euler, PositionControllerOutputStamped, AttitudeControllerOutputStamped 

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
    def __init__(self) -> None:
        
        # parâmeters
        self.orientation_atual = Quaternion()
        self.pos_control_output = PositionControllerOutputStamped()
        self.attitude_error_quat = Quaternion1()
        self.att_control_output = AttitudeControllerOutputStamped()
        
        # Subscrições
        sub_att_name = '/quad/kf/attitude'
        sub_pos_cont_out = '/quad/control/position_controller_output'
        self.sub_att = rospy.Subscriber(sub_att_name,\
            Quaternion,\
                self.callback_attitude)
        self.pos_cont_out = rospy.Subscriber(sub_pos_cont_out,\
            PositionControllerOutputStamped,\
                self.callback_pos_control_out)
        
        # Publicações
        pub_att_cont_out = '/quad/control/attitude_controller_output'
        """
        self.pub_att_control_output = rospy.Publisher(pub_att_cont_out,\
            PositionControllerOutputStamped,\
                queue_size=10)        
        """
        
    def callback_attitude(self, data):
        # orientação atual
        self.orientation_atual = data
    
    def callback_pos_control_out(self, data):
        self.pos_control_output = data
        self.attitude_error_quat =\
            self.pos_control_output.position_controller_output.orientation_set_point
        self.control()
    
    def control(self):
        # chama o controle de attitude 
        # publica os três torques
        return None
    
    
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
