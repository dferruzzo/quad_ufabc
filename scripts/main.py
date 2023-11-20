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

def main():
    rospy.init_node('Main')
    start_estimator()
    rospy.sleep(1)
    start_control()
    rospy.spin()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException():
        pass
