from quad_control import Controller
import numpy as np 
controller = Controller()
pos_atual = np.array([[1,2,3]]).T
pos_des = np.array([[4,5,6]]).T
vel_atual = np.array([[7,8,9]]).T
vel_des = np.array([[2,4,6]]).T
accel_des = np.array([[3,5,7]]).T
print('\n')
print('Resultado do pos_control_quad\n')
print(controller.pos_control_quat(pos_atual, pos_des, vel_atual, vel_des, accel_des))
print('\n')


