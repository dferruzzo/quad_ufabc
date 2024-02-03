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
--network host \
dferruzzo/ros-noetic-gazebo-gzweb:v1.02 /bin/bash -c '/home/catkin_ws/src/quad_ufabc/startup.sh && /bin/bash'
```

O script `/home/catkin_ws/quad_ufabc/startup.sh` inicia o workspace e executa `catkin_make`.

## Integração com VScode

Abra o Vscode no seu computador e utilize a extensão Docker para abrir o container com VScode. Caso não utilize Vscode pode utilizar `screen` para abrir vários terminais.

## Iniciar o projeto com gzweb

Para utlizar o navegador utilizar o gzweb. Abra um terminal no novo container. 

```
roslaunch quad_ufabc quad.launch gui:=false
```

num outro terminal

```
cd ~/gzweb/
npm start
```

O gzweb está disponível no `http://localhost:8080`

## Iniciar o Projeto com Gazebo

Precisa criar o container com permissões para acessar a tela.

`roslaunch quad_ufabc quad.launch`

## Iniciar a simulação

Em qulquer caso, seja no navegador ou na tela, num outro terminal rodar `rosrun quad_ufabc main.py` para iniciar a simulação. Utilize `Ctrl+c` para finalizar o script `main.py`.

## Visualizar os topicos

Pode rodar, utilizar e modificar o script `figures.py` para gerar as figuras da maioria dos tópicos relevante. O script utiliza `mpld3` para gerar o server html na porta 8888, `http://localhost:8888`.