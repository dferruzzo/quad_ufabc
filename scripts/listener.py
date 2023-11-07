#!/usr/bin/env python3
import rospy
import numpy as np
#from std_msgs.msg import String
#from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

quad_position = Vector3()       # Vetor posição
quad_orientation = Quaternion() # quaternion de atitude
quad_velocity = Vector3()       # velocidade translacional
quad_ang_velocity = Vector3()   # velocidade angular

def callback(data):
    # Quad pose, representa a posição e a attitude em quaternion
    quad_pose = data.pose[2]    
    # Quad twist representa a velocidade linear e angular
    quad_twist = data.twist[2]  
    
    quad_position = quad_pose.position
    quad_orientation = quad_pose.orientation # Em quaternion
    quad_velocity = quad_twist.linear
    quad_ang_velocity = quad_twist.angular    #

    rospy.loginfo(quad_orientation)
    
    
def listener():
    rospy.init_node('jointstate_listener', anonymous=True)

    rospy.Subscriber("gazebo/model_states", ModelStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()