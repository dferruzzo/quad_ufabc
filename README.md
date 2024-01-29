# quad_ufabc

Quadrirrotor em ROS Noetic Gazebo gzweb.

## Clonar o repo

`git clone https://github.com/dferruzzo/quad_ufabc.git`

## ROS Noetic + Gazebo + gzweb

[https://hub.docker.com/r/dferruzzo/ros-noetic-gazebo-gzweb](https://hub.docker.com/r/dferruzzo/ros-noetic-gazebo-gzweb)

```
docker run \
--rm \
-it \
--name [name-of-your-container] \
-v [path-to-quad-ufabc]:/home/catkin_ws/src/quad-ufabc \
-v [path-to-quad-ufabc]:/root/gzweb/http/client/assets/quad-ufabc \
-p 8080:8080 \
dferruzzo/ros-noetic-gazebo-gzweb:v1.02
```

## Iniciar o Workspace

```
cd /catkin_ws/src
source /opt/ros/noetic/setup.bash
catkin_init_workspace
cd ..
catkin_make
```

## Iniciar o Projeto com gazebo

Precisa criar o container com permissões para acessar a tela.

`roslaunch quad_ufabc quad.launch`

## Iniciar o projeto com gzweb

Uma forma mais fácil é utilizar o gzweb, só precisa liberar o porto 8080 como mostrado no `docker run`.

```
roslaunch quad_ufabc quad.launch gui:=false
cd ~/gzweb/
npm start
```

O gzweb está disponível no `http://localhost:8080`

## Iniciar o main.py

`rosrun quad_ufabc main.py`
