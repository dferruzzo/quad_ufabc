#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters

from kalman_filter import KF
from collections import deque
from std_msgs.msg import Bool
from quad_ros import quad_robot
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3Stamped, Pose, Vector3

from quat_utils import Quat2Rot

class MEKF_node(KF):
    def __init__(self):
        super().__init__()
        self.b_gx = np.random.normal(0.2, 1e-4)
        self.b_gy = np.random.normal(0.4, 1e-4)
        self.b_gz = np.random.normal(-0.3, 1e-4)
        self.accel_raw = None
        self.ang_vel = None
        
        self.ang_est = Quaternion()
        self.pos_est = Vector3()

        self.t_ant = rospy.Time.now().to_nsec()
        
        #Declare attitude publisher based on quaternion parametrization
        self.attitude_pub = rospy.Publisher('quad/kf/attitude', Quaternion, queue_size=10)
        self.position_pub = rospy.Publisher('quad/kf/position', Vector3, queue_size=10)
        self.bias_gyro_pub = rospy.Publisher('quad/kf/bias_gyro', Vector3, queue_size=10)
        self.trace_pub = rospy.Publisher('quad/kf/trace', Float32, queue_size=10)
        self.P_pub = rospy.Publisher('quad/kf/P_k', numpy_msg(Floats), queue_size=10)
        self.error_pub = rospy.Publisher('quad/kf/error_state', numpy_msg(Floats), queue_size=10)
        self.vel_pub = rospy.Publisher('quad/kf/vel', Vector3, queue_size=10)
    
        #Sensors subscribe
        self.imu_sub = rospy.Subscriber('/quad/imu', Imu, self.callback_imu, queue_size=10)
        
    def callback_imu(self, data):
        self.accel_raw = np.array([[
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z]]).T
        self.ang_vel = np.array([[
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z]]).T
        
        t = rospy.Time.now().to_nsec()
        dt = round((t - self.t_ant)*1e-9, 3)
        self.t_ant = t
        if dt > 0.02:dt = 0.01
        
        self.MEKF(self.accel_raw, self.ang_vel, None)
        
        # Publishing
        # ------------------------------------------------------
        # Publish estimated quaternion on 'quad/kf/attitude' topic
        #Store the estimated quaternion in message
        self.ang_est.x = self.q_K[1,0]
        self.ang_est.y = self.q_K[2,0]
        self.ang_est.z = self.q_K[3,0]
        self.ang_est.w = self.q_K[0,0]
        self.attitude_pub.publish(self.ang_est)       


if __name__ == '__main__':
    rospy.init_node('estimation_node_2')
    MEKF_node()
    rate = rospy.Rate(100) # 100hz
    rate.sleep()
    rospy.spin()
    try:
        EKF_node()
    except rospy.ROSInterruptException:
        pass
    