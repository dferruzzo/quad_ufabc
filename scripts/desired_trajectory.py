#!/usr/bin/env python3
"""
Data: 19/11/2023
Autor: Diego Ferruzzo
Descrição: Publica um tópico que publica as referências de:

    1. Posição,
    2. Velocidade,
    3. Aceleração,
    4. Tempo,

    a partir do arquivo 'trajetory_#.p'
"""
import rospy
import pickle
import os, sys
from quat_utils import Quat2Euler
import numpy as np

#from trajectory_msgs.msg import JointTrajectoryPoint
from quad_ufabc.msg import CartesianPointStamped
# from cartesian_control_msgs.msg import CartesianTrajectoryPoint 

def trajectory() -> None: 
   
    rospy.init_node('desired_trajectory')#, anonymous=False)
    pub = rospy.Publisher('desired_trajectory', CartesianPointStamped, queue_size=10)
       
    # Import the desired trajectory computed by 'trajectory_generator.py' executable.        
    mydir = os.path.abspath(sys.path[0]) #Set main directory
    infile = open(mydir + '/'+'data/trajectory_6.p', 'rb')    
    #infile = open(mydir + '/'+'data/trajectory_3.p', 'rb')    
    #infile = open(mydir + '/'+'data/trajectory_2.p', 'rb')
    traj = pickle.load(infile)
    infile.close()        
    
    desired_trajectory = CartesianPointStamped()
    #desired_trajectory = JointTrajectoryPoint()
    #desired_trajectory = CartesianTrajectoryPoint()
    
    step:int = 0
    dt = traj['time'][step+1]-traj['time'][step]
    rate = rospy.Rate(1/dt)
    time_start = rospy.Time.now()
    
    while not rospy.is_shutdown():
        #
        time_begin = rospy.Time.now()

        desired_trajectory.cartesian_point.pose.position.x = traj['x'][step]
        desired_trajectory.cartesian_point.pose.position.y = traj['y'][step]
        desired_trajectory.cartesian_point.pose.position.z = traj['z'][step]
        
        desired_trajectory.cartesian_point.velocity.linear.x = traj['dx'][step]
        desired_trajectory.cartesian_point.velocity.linear.y = traj['dy'][step]
        desired_trajectory.cartesian_point.velocity.linear.z = traj['dz'][step]
        
        desired_trajectory.cartesian_point.acceleration.linear.x = traj['ddx'][step]
        desired_trajectory.cartesian_point.acceleration.linear.y = traj['ddy'][step]
        desired_trajectory.cartesian_point.acceleration.linear.z = traj['ddz'][step]
        
        qz = traj['qz ref'][step]        
        desired_trajectory.cartesian_point.pose.orientation.w = qz[0]
        desired_trajectory.cartesian_point.pose.orientation.x = qz[1]
        desired_trajectory.cartesian_point.pose.orientation.y = qz[2]
        desired_trajectory.cartesian_point.pose.orientation.z = qz[3]
        
        # computa os angulos de Euler da trajetória a seguir no sistema inercial
        euler_traj = Quat2Euler(qz)
        desired_trajectory.cartesian_point.pose.euler_orientation.phi = euler_traj[0]
        desired_trajectory.cartesian_point.pose.euler_orientation.theta = euler_traj[1]
        desired_trajectory.cartesian_point.pose.euler_orientation.psi = euler_traj[2]
        
        desired_trajectory.header.stamp = rospy.Time.now()
        
        pub.publish(desired_trajectory)

        step += 1
       
        if step == len(traj['x']):
            step = len(traj['x'])-1
            
        rate.sleep()
        time_end = rospy.Time.now()
        duration = time_end - time_begin

        #rospy.loginfo("Duration: " + str(duration.to_sec()) + " secs")
    
if __name__ == '__main__':
    try:
        trajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass