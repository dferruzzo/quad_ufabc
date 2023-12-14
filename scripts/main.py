#!/usr/bin/env python3
import rospy
import roslaunch

def start_control():

    rospy.loginfo('Starting Quadrotor Control Node...')

    package = 'quad_ufabc'
    executable = 'control_node.py'
    node_name = 'control_node'

    node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    if process.is_alive():
        print('')
        print('Control Initialized')
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

def main():
    # ------------------------------------------------------------------------------------------
    # TODO:
    # [x] (13/12/2023) Publicar no nodo 'desired trajectory' a attitude em quaternio e em Euler.
    # ------------------------------------------------------------------------------------------ 
    rospy.init_node('Main')
    start_estimator()
    rospy.sleep(1)
    start_control()
    start_desired_traj()
    rospy.spin()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException():
        pass
