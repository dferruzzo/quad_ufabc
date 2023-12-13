# quad_ufabc
Quadrirrotor em ROS Gazebo.
## ROS Noetic
Precisa:
1. instalar ROS Noetic.
2. criar um workspace
3. clonar o branch `master` dentro da pasta `src`
## Como clonar o repo
`git clone -b master https://github.com/dferruzzo/quad_ufabc.git`
## Iniciar o Workspace
```
cd /path/to/work/directory/src
source /opt/ros/noetic/setup.bash
catkin_init_workspace
cd ..
catkin_make
```
## Iniciar o Projeto
`roslaunch quad_ufabc quad.launch`
## Iniciar o main.py
`rosrun quad_ufabc main.py`

