#!/usr/bin/env python3
import rospy
import numpy as np
from quad_ros import quad_robot
from quad_control import Controller
from quad_ufabc.msg import PositionControllerOutputStamped, AttitudeControllerOutputStamped

class Quadrotor(quad_robot):
    def __init__(self):
        self.name = 'quad'
        super().__init__(self.name)
        self.quad_ufabc = quad_robot(self.name)
        self.quad_ufabc.reset()
        self.control_output = np.zeros(4)
        # subscribir a tópico '/quad/control/position_controller_output'
        sub_pos_cont_out_name = '/quad/control/position_controller_output'
        self.sub_pos_cont_out = rospy.Subscriber(sub_pos_cont_out_name,
                                                 PositionControllerOutputStamped,
                                                 self.callback_pos_control_output)
        # subscribir a tópcio '/quad/control/attitude_controller_output'
        sub_att_cont_out_name = '/quad/control/attitude_controller_output'
        self.sub_att_cont_out = rospy.Subscriber(sub_att_cont_out_name,
                                                 AttitudeControllerOutputStamped,
                                                 self.callback_att_control_output)
            
    def callback_pos_control_output(self, data):
        self.control_output[0] = data.position_controller_output.thrust.num
    
    def callback_att_control_output(self, data):
        self.control_output[1] = data.torques.x
        self.control_output[2] = data.torques.y
        self.control_output[3] = data.torques.z
        self.control_allocator()
    
    def control_allocator(self):
        controller = Controller()
        w, _, _ = controller.f2w(self.control_output[0],
                                 [self.control_output[1],
                                  self.control_output[2], 
                                  self.control_output[3]]) 
        # Send the command to quadrotor
        self.quad_ufabc.step(w/10)   

if __name__ == '__main__':
    try:       
        node_name = 'quadrotor_node_2'
        rospy.init_node(node_name)
        Quadrotor()        
        #rate = rospy.Rate(100) 
        #rate.sleep()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass    