#!/usr/bin/env python3
#import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import mpld3
from mpld3._server import serve

file_name = 'dados'
#b = bagreader('../bags/dados.bag')
b = bagreader('../bags/' + file_name + '.bag')

# Trajetória desejada topic -> pandas database
des_traj = b.message_by_topic(topic='/desired_trajectory')
des_traj_df = pd.read_csv(des_traj)

# Tópico "posição atual" do filtro de Kalman -> pandas databse
actual_pos = b.message_by_topic(topic='/quad/kf/position')
actual_pos_df = pd.read_csv(actual_pos)

# gráfico das Posições
fig1 = plt.figure()
plt.subplot(3,1,1)
plt.title('Posições XYZ')
plt.plot(des_traj_df['Time'].to_numpy(), \
    des_traj_df['cartesian_point.pose.position.x'].to_numpy())
plt.plot(actual_pos_df['Time'].to_numpy(), actual_pos_df['x'].to_numpy())
frame1 = plt.gca()
frame1.axes.xaxis.set_ticklabels([])
#frame1.axes.get_xaxis().set_ticks([])
#frame1.axes.get_xaxis().set_visible(False)
#plt.grid()
plt.legend(['x(t) desejado','x(t) FK'])
plt.subplot(3,1,2)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.position.y'].to_numpy())
plt.plot(actual_pos_df['Time'].to_numpy(), actual_pos_df['y'].to_numpy())
plt.legend(['y(t) desejado','y(t) FK'])
frame2 = plt.gca()
frame2.axes.xaxis.set_ticklabels([])
#frame2.axes.get_xaxis().set_visible(False)
#frame2.axes.get_xaxis().set_visible(False)
#plt.grid()
plt.subplot(3,1,3)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.position.z'].to_numpy())
plt.plot(actual_pos_df['Time'].to_numpy(), actual_pos_df['z'].to_numpy())
plt.legend(['z(t) desejado','z(t) FK'])
#plt.grid()
plt.xlabel('tempo')

# tópico "velocidades lineares" -> pandas database 
actual_vel = b.message_by_topic(topic='/quad/kf/vel_lin')
actual_vel_df = pd.read_csv(actual_vel)

# gráfico das Velocidades
fig2 = plt.figure()
plt.subplot(3,1,1)
#plt.rcParams['text.usetex'] = True
plt.title('Velocidades XYZ')
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.velocity.linear.x'].to_numpy(), label='\\(v_x(t) desejado\\)')
plt.plot(actual_vel_df['Time'].to_numpy(), actual_vel_df['x'].to_numpy(),\
        label= '\\(v_x(t) FK\\)')
#plt.legend([r'$v_x(t) desejado$', r'$v_x(t) FK$'])
plt.legend()
frame1 = plt.gca()
frame1.axes.xaxis.set_ticklabels([])
#frame1.axes.get_xaxis().set_ticks([])
#frame1.axes.get_xaxis().set_visible(False)
#plt.grid()

plt.subplot(3,1,2)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.velocity.linear.y'].to_numpy())
plt.plot(actual_vel_df['Time'].to_numpy(), actual_vel_df['y'].to_numpy())
plt.legend(['$v_y$(t) desejado','$v_y$(t) FK'])
frame2 = plt.gca()
frame2.axes.xaxis.set_ticklabels([])
#frame2.axes.get_xaxis().set_visible(False)
#frame2.axes.get_xaxis().set_visible(False)
#plt.grid()

plt.subplot(3,1,3)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.velocity.linear.z'].to_numpy())
plt.plot(actual_vel_df['Time'].to_numpy(), actual_vel_df['z'].to_numpy())
plt.legend(['$v_z$(t) desejado','$v_z$(t) FK'])
#plt.grid()
plt.xlabel('tempo')

# Topico "atitude" -> pandas database
actual_ati = b.message_by_topic(topic='/quad/kf/attitude')
actual_ati_df = pd.read_csv(actual_ati)

# gráfico de pose
fig3 = plt.figure()
plt.subplot(2,2,1)
plt.title('Atitude Quaternions')
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.orientation.w'].to_numpy())
plt.plot(actual_ati_df['Time'].to_numpy(), actual_ati_df['w'].to_numpy())
plt.legend(['w(t) desejado','w(t) FK'])
frame1 = plt.gca()
frame1.axes.xaxis.set_ticklabels([])
#frame1.axes.get_xaxis().set_ticks([])
#frame1.axes.get_xaxis().set_visible(False)
#plt.grid()

plt.subplot(2,2,2)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.orientation.x'].to_numpy())
plt.plot(actual_ati_df['Time'].to_numpy(), actual_ati_df['x'].to_numpy())
plt.legend(['x(t) desejado','x(t) FK'])
frame2 = plt.gca()
frame2.axes.xaxis.set_ticklabels([])
#frame2.axes.get_xaxis().set_visible(False)
#frame2.axes.get_xaxis().set_visible(False)
#plt.grid()

plt.subplot(2,2,3)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.orientation.y'].to_numpy())
plt.plot(actual_ati_df['Time'].to_numpy(), actual_ati_df['y'].to_numpy())
plt.legend(['y(t) desejado','y(t) FK'])
#plt.grid()
plt.xlabel('tempo')

plt.subplot(2,2,4)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.orientation.z'].to_numpy())
plt.plot(actual_ati_df['Time'].to_numpy(), actual_ati_df['z'].to_numpy())
plt.legend(['z(t) desejado','z(t) FK'])
#plt.grid()
plt.xlabel('tempo')

#
#plt.show()

# tópico "atitude em euler" -> pandas database
actual_ati_euler = b.message_by_topic(topic='/quad/kf/euler')
actual_ati_euler_df = pd.read_csv(actual_ati_euler)

# Gráficos de Atitude em ầngulos de Euler
fig4 = plt.figure()
plt.subplot(3,1,1)
plt.title('Atitude em Euler')
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.euler_orientation.phi'].to_numpy())
plt.plot(actual_ati_euler_df['Time'].to_numpy(), actual_ati_euler_df['phi'].to_numpy())
plt.legend(['$\phi$(t) desejado','$\phi$(t) FK'])
frame1 = plt.gca()
frame1.axes.xaxis.set_ticklabels([])
#frame1.axes.get_xaxis().set_ticks([])
#frame1.axes.get_xaxis().set_visible(False)
#plt.grid()

plt.subplot(3,1,2)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.euler_orientation.theta'].to_numpy())
plt.plot(actual_ati_euler_df['Time'].to_numpy(), actual_ati_euler_df['theta'].to_numpy())
plt.legend(['$\\theta$(t) desejado','$\\theta$(t) FK'])
frame2 = plt.gca()
frame2.axes.xaxis.set_ticklabels([])
#frame2.axes.get_xaxis().set_visible(False)
#frame2.axes.get_xaxis().set_visible(False)
#plt.grid()

plt.subplot(3,1,3)
plt.plot(des_traj_df['Time'].to_numpy(), des_traj_df['cartesian_point.pose.euler_orientation.psi'].to_numpy())
plt.plot(actual_ati_euler_df['Time'].to_numpy(), actual_ati_euler_df['psi'].to_numpy())
plt.legend(['$\psi$t) desejado','$\psi$(t) FK'])
#plt.grid()
plt.xlabel('tempo')
#plt.show()

# saída do controle de posição e velocidade
pos_control_output = b.message_by_topic(topic='/quad/control/position_controller_output')
pos_control_output_df = pd.read_csv(pos_control_output)

# gráficos da saída de controle de posição e velocidade.
# Empuxo total e atitude em quaternion
fig5 = plt.figure()
plt.subplot(5,1,1)
plt.title('Saídas do controle de posição e velocidade')
plt.plot(pos_control_output_df['Time'].to_numpy(), pos_control_output_df['position_controller_output.thrust.num'].to_numpy())
#plt.grid()
plt.legend(['Empuxo total'])
frame1 = plt.gca()
frame1.axes.xaxis.set_ticklabels([])
#
plt.subplot(5,1,2)
plt.plot(pos_control_output_df['Time'].to_numpy(), pos_control_output_df['position_controller_output.orientation_set_point.w'].to_numpy())
#plt.grid()
plt.legend(['quaternion.w'])
frame2 = plt.gca()
frame2.axes.xaxis.set_ticklabels([])
#
plt.subplot(5,1,3)
plt.plot(pos_control_output_df['Time'].to_numpy(), pos_control_output_df['position_controller_output.orientation_set_point.x'].to_numpy())
#plt.grid()
plt.legend(['quaternion.x'])
frame3 = plt.gca()
frame3.axes.xaxis.set_ticklabels([])
#
plt.subplot(5,1,4)
plt.plot(pos_control_output_df['Time'].to_numpy(), pos_control_output_df['position_controller_output.orientation_set_point.y'].to_numpy())
#plt.grid()
plt.legend(['quaternion.y'])
frame4 = plt.gca()
frame4.axes.xaxis.set_ticklabels([])
#
plt.subplot(5,1,5)
plt.plot(pos_control_output_df['Time'].to_numpy(), pos_control_output_df['position_controller_output.orientation_set_point.z'].to_numpy())
#plt.grid()
plt.legend(['quaternion.z'])
#
#plt.show()

# gráficos da saída de controle de posição e velocidade.
# Empuxo total e atitude em ângulos de Euler
fig6 = plt.figure()
plt.subplot(4,1,1)
plt.title('Saídas do controle de posição e velocidade')
plt.plot(pos_control_output_df['Time'].to_numpy(),\
    pos_control_output_df['position_controller_output.thrust.num'].to_numpy(),\
        linewidth=1.5)
#plt.grid()
plt.legend(['Empuxo total'])
frame1 = plt.gca()
frame1.axes.xaxis.set_ticklabels([])
#
plt.subplot(4,1,2)
plt.plot(pos_control_output_df['Time'].to_numpy(),\
    pos_control_output_df['position_controller_output.euler_set_point.phi'].to_numpy(),\
        linewidth=1.5)
#plt.grid()
plt.legend(['$\phi$'])
frame2 = plt.gca()
frame2.axes.xaxis.set_ticklabels([])
#
plt.subplot(4,1,3)
plt.plot(pos_control_output_df['Time'].to_numpy(),\
    pos_control_output_df['position_controller_output.euler_set_point.theta'].to_numpy(),
    linewidth=1.5)
#plt.grid()
plt.legend(['$\\theta$'])
frame3 = plt.gca()
frame3.axes.xaxis.set_ticklabels([])
#
plt.subplot(4,1,4)
plt.plot(pos_control_output_df['Time'].to_numpy(),\
    pos_control_output_df['position_controller_output.euler_set_point.psi'].to_numpy(),\
        linewidth=1.5)
#plt.grid()
plt.legend(['$\psi$'])
#
#plt.show()

# saída do controle de atitude
att_control_output = b.message_by_topic(topic='/quad/control/attitude_controller_output')
att_control_output_df = pd.read_csv(att_control_output)
#print(att_control_output_df)

fig7 = plt.figure()
plt.subplot(4,1,1)
plt.title('Saídas do controle')
plt.plot(pos_control_output_df['Time'].to_numpy(), pos_control_output_df['position_controller_output.thrust.num'].to_numpy())
#plt.grid()
plt.legend(['Empuxo total'])
frame1 = plt.gca()
frame1.axes.xaxis.set_ticklabels([])
#
plt.subplot(4,1,2)
plt.plot(att_control_output_df['Time'].to_numpy(), att_control_output_df['torques.x'].to_numpy())
#plt.grid()
plt.legend(['torque \phi'])
frame2 = plt.gca()
frame2.axes.xaxis.set_ticklabels([])
#
plt.subplot(4,1,3)
plt.plot(att_control_output_df['Time'].to_numpy(), att_control_output_df['torques.y'].to_numpy())
#plt.grid()
plt.legend(['torque \\theta'])
frame3 = plt.gca()
frame3.axes.xaxis.set_ticklabels([])
#
plt.subplot(4,1,4)
plt.plot(att_control_output_df['Time'].to_numpy(), att_control_output_df['torques.z'].to_numpy())
#plt.grid()
plt.legend(['torque \psi'])
frame4 = plt.gca()
frame4.axes.xaxis.set_ticklabels([])
#
# create html for both graphs 
html1 = mpld3.fig_to_html(fig1)
html2 = mpld3.fig_to_html(fig2)
html3 = mpld3.fig_to_html(fig3)
html4 = mpld3.fig_to_html(fig4)
html5 = mpld3.fig_to_html(fig5)
html6 = mpld3.fig_to_html(fig6)
html7 = mpld3.fig_to_html(fig7)

# serve joined html to browser
serve(html1+html2+html3+html4+html5+html6+html7)
