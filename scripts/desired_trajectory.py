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
from trajectory_msgs.msg import JointTrajectoryPoint

def trajectory() -> None:
    pub = rospy.Publisher('desired_trajectory', JointTrajectoryPoint, queue_size=10)
    rospy.init_node('desired_trajectory')#, anonymous=False)
       
    # Import the desired trajectory computed by 'trajectory_generator.py' executable.        
    mydir = os.path.abspath(sys.path[0]) #Set main directory
    infile = open(mydir + '/'+'data/trajectory_3.p', 'rb')    
    #infile = open(mydir + '/'+'data/trajectory_2.p', 'rb')
    traj = pickle.load(infile)
    infile.close()        
    
    desired_trajectory = JointTrajectoryPoint()
    
    step:int = 0
    dt = traj['time'][step+1]-traj['time'][step]
    rate = rospy.Rate(1/dt)
    time_start = rospy.Time.now()
    
    while not rospy.is_shutdown():
        #
        time_begin = rospy.Time.now()

        desired_trajectory.positions = [traj['x'][step], traj['y'][step], traj['y'][step]]
        desired_trajectory.velocities = [traj['dx'][step], traj['dy'][step], traj['dz'][step]]
        desired_trajectory.accelerations = [traj['ddx'][step], traj['ddy'][step], traj['ddz'][step]]
        desired_trajectory.time_from_start = rospy.Time.now() - time_start
        
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
    except rospy.ROSInterruptException:
        pass