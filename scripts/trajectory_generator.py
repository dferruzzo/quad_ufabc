#!/usr/bin/env python3
#
import pickle
import os, sys
from sqlite3 import Time
from xmlrpc.client import Boolean
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from quat_utils import Euler2Quat
from quad_control import Controller

class waypoint:
    # waypoint
    def __init__(self, x: float, y: float, z: float, psi: float, t: float) -> None:
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.psi: float = psi
        self.t: float = t       # time
    
    def __repr__(self) -> str:
        return "{}(x={}, y={}, z={}, psi={}, time={})".format(self.__class__.__name__, self.x, self.y, self.z, self.psi, self.t)


#Instance Controller Class
controller = Controller()

save_to_file:bool = True
file_name:str = 'trajectory_6.p'

# Define the trajectory points and the desired yaw angle
# waypoint(x, y, z, psi, tempo)

#Trajectory 1
# x_wp = np.array([[0.6, 0.25, -0.5, 0, 0]]).T
# y_wp = np.array([[0, 0, 0, 0.2, 0.6]]).T
# z_wp = np.array([[0, 1, 1.5, 1, 1.5]]).T
# psi_wp = np.array([[0, np.pi/4, np.pi/2, 0, 0]]).T

# #Trajectory 2
# x_wp = np.array([[0.6, 0.6, 0.0, 0.0, 0.0]]).T
# y_wp = np.array([[0, -0.2, 0, 0.2, 0.6]]).T
# z_wp = np.array([[0, 1, 1.5, 1, 2]]).T
# psi_wp = np.array([[0, 0, np.pi/8, np.pi/6, np.pi/4]]).T

# Trajectory 3
home = waypoint(0.6, 0.0, 0.0, 0.0, 0.0)
wp1  = waypoint(0.6, 0.0, 2.0, np.pi/6, 5.0)
wp2  = waypoint(1.5, 0.0, 2.0, np.pi/6, 10.0)
wp3  = waypoint(2.0, 0.5, 2.0, np.pi/8, 15.0)
wp4  = waypoint(2.0, 1.0, 2.0, 0, 20.0)

# Voo pairado
#home = waypoint(0.6, 0.0, 0.0, 0.0, 0.0)
#wp1  = waypoint(0.6, 0.0, 0.5, 0.0, 5.0)
#wp2  = waypoint(0.6, 0.0, 1.0, 0.0, 10.0)
#wp3  = waypoint(0.6, 0.0, 1.5, 0.0, 15.0)
#wp4  = waypoint(0.6, 0.0, 2.0, 0.0, 20.0)

#Set sample time
step_controller = 0.01

# ----------------------------------------------
x_wp = np.array([[home.x, wp1.x, wp2.x, wp3.x, wp4.x]]).T
y_wp = np.array([[home.y, wp1.y, wp2.y, wp3.y, wp4.y]]).T
z_wp = np.array([[home.z, wp1.z, wp2.z, wp3.z, wp4.z]]).T
psi_wp = np.array([[home.psi, wp1.psi, wp2.psi, wp3.psi, wp4.psi]]).T

#Define desired time
time = [home.t, wp1.t, wp2.t, wp3.t, wp4.t]
t0 = home.t
tf = wp4.t

#Compute minimum snap trajectory for position and minimum acceleration for yaw angle
_, _, x_matrix = controller.getCoeff_snap(x_wp, time)
_, _, y_matrix = controller.getCoeff_snap(y_wp, time)
_, _, z_matrix = controller.getCoeff_snap(z_wp, time)
_, _, psi_matrix = controller.getCoeff_accel(psi_wp, time)

x_ref, dotx_ref, ddotx_ref, _, _ = controller.evaluate_equations_snap(time, step_controller, x_matrix)
y_ref, doty_ref, ddoty_ref, _, _ = controller.evaluate_equations_snap(time, step_controller, y_matrix)
z_ref, dotz_ref, ddotz_ref, _, _ = controller.evaluate_equations_snap(time, step_controller, z_matrix)
psi_ref, _, _ = controller.evaluate_equations_accel(time, step_controller, psi_matrix)

qz_ref = []

for i in range(len(psi_ref)):

    # qz_ref.append()
    q_z_ham = np.zeros((4,1))
    q_z_ham[0] = np.cos(psi_ref[i]/2)
    q_z_ham[-1] = np.sin(psi_ref[i]/2)
    q_z_ham_norm = np.linalg.norm(q_z_ham)
    q_z_ham = q_z_ham/q_z_ham_norm
    qz_ref.append(q_z_ham)

time = np.arange(t0, tf, step_controller)

# print([qz_ref[k][0,0] for k in range(len(qz_ref))])

#Plot desired trajectory
fig, (x, y, z, psi) = plt.subplots(4,1,figsize=(9,7))
x.plot(time, x_ref)
y.plot(time, y_ref)
z.plot(time, z_ref)
# psi.plot(time, [qz_ref[k][0,0] for k in range(len(qz_ref))])
# psi.plot(time, [qz_ref[k][1,0] for k in range(len(qz_ref))])
# psi.plot(time, [qz_ref[k][2,0] for k in range(len(qz_ref))])
# psi.plot(time, [qz_ref[k][3,0] for k in range(len(qz_ref))])
psi.plot(time, psi_ref)

psi.set_xlabel('Tempo(s)')
x.set_ylabel(r'$X(m)$')
y.set_ylabel(r'$Y(m)$')
z.set_ylabel(r'$Z(m)$')
psi.set_ylabel(r'$\psi(rad)$')

x.grid()
y.grid()
z.grid()
psi.grid()
# psi2.grid()

fig3 = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title('Trajetória')
ax.plot3D(x_ref, y_ref, z_ref, 'g--')
ax.set_xlabel('X(m)')
ax.set_ylabel('Y(m)')
ax.set_zlabel('Z(m)')
plt.show()

# Save trajectory in a dictionary
if save_to_file:
    trajectory = {'x': x_ref,
                  'y': y_ref,
                  'z': z_ref,
                  'dx': dotx_ref,
                  'dy': doty_ref,
                  'dz': dotz_ref,
                  'ddx': ddotx_ref,
                  'ddy': ddoty_ref,
                  'ddz': ddotz_ref,
                  'psi': psi_ref,
                  'qz ref': qz_ref,
                  'time': time}    
    mydir = os.path.abspath(sys.path[0]) # sets main directory
    outfile = open(mydir + '/data/' + file_name, 'wb')
    pickle.dump(trajectory, outfile)
    outfile.close()