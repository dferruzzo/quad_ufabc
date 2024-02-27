#!/usr/bin/env python3
import rospy
import roslaunch

def start_pos_vel_control():

    rospy.loginfo('Starting position and velocity Quadrotor Control Node...')

    package = 'quad_ufabc'
    executable = 'pos_vel_control.py'
    node_name = 'position_velocity_control_node'

    node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    if process.is_alive():
        print('')
        print('Position and Velocity Control Initialized')
        print('')

def start_attitude_control():

    rospy.loginfo('Starting attitude Control Node...')

    package = 'quad_ufabc'
    executable = 'attitude_control.py'
    node_name = 'attitude_control_node'

    node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    if process.is_alive():
        print('')
        print('Attitude Control Initialized')
        print('')
        
def start_estimator():
    rospy.loginfo('Starting Estimation Node...')
    
    package = 'quad_ufabc'
    executable = 'estimation_node.py'
    node_name = 'estimation_node'
    
    node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    if process.is_alive():
        print('')
        print('Estimation Initialized')
        print('')

def start_desired_traj(): 
    rospy.loginfo('Publishing desired trajectory...')
    
    package = 'quad_ufabc'
    executable = 'desired_trajectory.py'
    node_name = 'desired_trajectory_node'
    
    node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    if process.is_alive():
        print('')
        print('Publishing desired trajectory')
        print('')

def start_recorder_node(): 
    rospy.loginfo('Starting Recorder node...')
    
    package = 'quad_ufabc'
    executable = 'bag_recorder_node.py'
    node_name = 'bag_recorder_node'
    
    node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    if process.is_alive():
        print('')
        print('Starting Recorder node')
        print('')

def start_quadrotor(): 
    rospy.loginfo('Starting Quadrotor node...')
    
    package = 'quad_ufabc'
    executable = 'quadrotor_node.py'
    node_name = 'quadrotor_node'
    
    node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    if process.is_alive():
        print('')
        print('Starting Quadrotor node')
        print('')
        
def main():
    # 
    rospy.init_node('Main')
    rospy.sleep(1)
    #start_recorder_node()       
    start_quadrotor()
    start_estimator()   
    #start_pos_vel_control()     
    #start_attitude_control()    
    start_desired_traj()        
    rospy.spin()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException():
        pass
