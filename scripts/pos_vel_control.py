#!/usr/bin/env python3
#
import rospy
import numpy as np
#from quad_ros import quad_robot
from quat_utils import QuatProd, Quat2Euler, Euler2Quat
from quad_control import Controller
#from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Quaternion, Vector3, Pose, Vector3Stamped, PoseStamped
from quad_ufabc.msg import CartesianPointStamped,\
    Pose, Velocity, Accel, Point, Quaternion1,\
        Num, Euler, PositionControllerOutputStamped

#from sensor_msgs.msg import Imu

class Pos_Vel_Control(Controller):
    
    """
    A classe herda os atributos e métodos da classe Controller e se subscribe aos seguintes tópicos:
        - /quad/kf/attitude,
        - /quad/kf/position,
        - /quad/kf/vel,
        - /desired_trajectory,
    publica o  tópico 
        - /quad/control/position_controller_output
    que é do tipo tipo quad_ufabc/PositionControllerOutputStamped
    e que publica,
        - O tempo na forma de stamp,
        - O empuxo total, thrust,    
        - O erro de atitude em quaternion (w,x,y,z),
        - O erro de atitude em ângulos de Euler (phi, theta, psi)
    Nesta versão implemento o método 'pos_control_quat_v1',
    Entradas:
        - pos_atual,
        - pos_des,
        - vel_atual,
        - vel_des,
        - orientation_des
    Salidas:
        - T, o empuxo total 
        - q_erro, o erro de atitude que é recebido pelo controle de atitude.
        - erro de atitude em ângulos de Euler.
    """    
    
    def __init__(self, controller) -> None:
        
        # Parâmetros
        self.controller = controller        # tipo de controlador
        # Inicialização
        self.pos_atual = Vector3()              # posição atual
        self.vel_atual = Vector3()              # velocidade atual
        self.orientation_atual = Quaternion()   # orientação atual
        self.orien_atual_euler = Euler()        # orientação em Euler        
        self.pos_des = Point()                  # posição desejada
        self.vel_des = Point()                  # velocidade desejada
        self.acel_des = Point()                 # aceleração desejada
        self.orientation_des = Quaternion1()    # orientação desejada            
        self.euler_orientation_des = Euler()    # orientação desejada            
        self.pos_control_output = PositionControllerOutputStamped()
        self.desired_trajectory = CartesianPointStamped()   # valores de trajetória desejados
        
        # Subscrições
        sub_att_name = '/quad/kf/attitude'
        sub_pos_name = '/quad/kf/position'      # nome do tópico de posição
        sub_vel_name = '/quad/kf/vel'           # nome do tópico de velocidade
        sub_tra_name = '/desired_trajectory'    # nome do tópico de trajetória desejada
        self.sub_att = rospy.Subscriber(sub_att_name,\
            Quaternion,\
                self.callback_attitude)
        self.sub_pos = rospy.Subscriber(sub_pos_name,\
            Vector3,\
                self.callback_position)
        self.sub_vel = rospy.Subscriber(sub_vel_name,\
            Vector3,\
                self.callback_velocity)
        self.sub_tra = rospy.Subscriber(sub_tra_name,\
            CartesianPointStamped,\
                self.callback_trajectory)

        # Publicações
        pub_pos_cont_out_name = '/quad/control/position_controller_output'
        self.pub_pos_control_output = rospy.Publisher(pub_pos_cont_out_name,\
            PositionControllerOutputStamped,\
                queue_size=10)        
        
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
        
    def callback_trajectory(self, data):
        self.pos_des = data.cartesian_point.pose.position
        self.vel_des = data.cartesian_point.velocity.linear
        self.acel_des = data.cartesian_point.acceleration.linear
        self.orientation_des = data.cartesian_point.pose.orientation
        self.euler_orientation_des = data.cartesian_point.pose.euler_orientation
        #
        self.control()      
    
    def control(self):
        # Saída:  
        # T é o empuxo total.
        # q_pdes é o quaternio de atitude desejado correspondente aos ângulos de Euler desejados.
        # euler_out é a atitude desejada em ângulos de Euler.
        #    
        if self.controller == "Quat":
            T, q_erro, euler_out = self.pos_control_Quat(
                self.point_to_np_array(self.pos_atual),
                self.point_to_np_array(self.pos_des),
                self.point_to_np_array(self.vel_atual),
                self.point_to_np_array(self.vel_des),
                self.quat_to_np_array(self.orientation_des))
        elif self.controller == "PD":
            T, q_erro, euler_out = self.pos_control_PD(
                self.point_to_np_array(self.pos_atual),
                self.point_to_np_array(self.pos_des),
                self.point_to_np_array(self.vel_atual),
                self.point_to_np_array(self.vel_des),
                self.point_to_np_array(self.acel_des),
                np.double(self.euler_orientation_des.psi)) 
        else:
            T, q_erro, euler_out = self.pos_control_Quat(
                self.point_to_np_array(self.pos_atual),
                self.point_to_np_array(self.pos_des),
                self.point_to_np_array(self.vel_atual),
                self.point_to_np_array(self.vel_des),
                self.quat_to_np_array(self.orientation_des))
                
        self.orien_atual_euler.phi = euler_out[0]
        self.orien_atual_euler.theta = euler_out[1]
        self.orien_atual_euler.psi = euler_out[2]       
        #        
        self.pos_control_output.position_controller_output.thrust.num = T
        self.pos_control_output.position_controller_output.orientation_set_point.w = q_erro[0]
        self.pos_control_output.position_controller_output.orientation_set_point.x = q_erro[1]
        self.pos_control_output.position_controller_output.orientation_set_point.y = q_erro[2]
        self.pos_control_output.position_controller_output.orientation_set_point.z = q_erro[3]     
        self.pos_control_output.position_controller_output.euler_set_point =  self.orien_atual_euler
        #        
        self.pos_control_output.header.stamp = rospy.Time.now()
        #
        self.pub_pos_control_output.publish(self.pos_control_output)

if __name__ == '__main__':
    try:
        node_name = 'pos_vel_controller_2'
        rospy.init_node(node_name)
        Pos_Vel_Control(controller="PD")        
        rate = rospy.Rate(50) # 50hz
        rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    
    
         
    
