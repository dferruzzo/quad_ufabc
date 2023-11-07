#!/usr/bin/env python3
import rospy
import roslaunch

def start_control():

    rospy.loginfo('Starting Quadrotor Control Node...')

    package = 'quad_ufabc'
    executable_control = 'control_node.py'

    node_control = roslaunch.core.Node(package, executable_control)

    launch_control = roslaunch.scriptapi.ROSLaunch()

    launch_control.start()

    controller = launch_control.launch(node_control)

    if controller.is_alive():
        print('')
        print('Control Initialized')
        print('')

def start_estimator():
    rospy.loginfo('Starting Estimation Node...')
    
    package = 'quad_ufabc'
    executable_estimation = 'estimation_node.py'
    node_estimation = roslaunch.core.Node(package, executable_estimation)
    launch_estimation = roslaunch.scriptapi.ROSLaunch()
    launch_estimation.start()
    estimation = launch_estimation.launch(node_estimation)

    if estimation.is_alive():
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
