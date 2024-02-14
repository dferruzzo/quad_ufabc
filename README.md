# Quad UFABC

## Descrição

Quad UFABC é um projeto que implementa um quadrirrotor em ROS Noetic Gazebo. A simulação conta com sensores IMU (acelerômetros e giroscópios). É possível implementar diferentes tipos de controladores para a dinâmica translacional e para a rotacional. No momento tem sido implementado os seguintes controladores:

* Para a dinâmica Translacional:
  * Controlador PD em ângulos de Euler,
  * Controlador PD em quatérnios.   
* Para a dinâmica Rotacional:
  * Controlador PD em quatérnios,
  * Controlador LQR em ângulos de Euler.

## Requerimentos

* [Visual Studio Code](https://code.visualstudio.com/) com extensão: [Microsoft Dev Container](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers),
* [Docker](https://www.docker.com/),
* No Windows precisa `Ubuntu` no [WSL2](https://learn.microsoft.com/pt-br/windows/wsl/install).

## Instalação

Abra um terminal no Linux ou no WSL no Windows e  clone o repositório

```bash
git clone https://github.com/dferruzzo/quad_ufabc.git
```
Abra o Vscode, no Windows precisa se conectar ao WSL antes de continuar.
Abrir a pasta `quaf_ufabc` no `Vscode`. No menu do canto inferior esquerdo do `Vscode` selecionar a opção `Reabrir no Contêiner`. Isso irá preparar o ambiente de trabalho e ira fazer o download da imagem do `ROS noetic + Gazebo` caso não tenha sido abaixado antes.

## Iniciar o projeto

Num terminal do Vscode rode o comando

```bash
roslaunch quad_ufabc quad.launch
```
que inicia a simulação em ROS Noetic Gazebo, ou
```bash
roslaunch quad_ufabc quad.launch gui:=false
```
Para iniciar a simulação sem GUI e num outro terminal rode

```bash
cd ~/gzweb/
npm start
```
para abrir o Gazebo no browser `http://localhost:8080`.

## Iniciar a simulação

Em qualquer caso, seja no navegador ou na tela, num outro terminal rode
```bash
rosrun quad_ufabc main.py
```
para iniciar a simulação. Utilize `Ctrl+c` para finalizar o script `main.py`.

## Visualizar os tópicos

Utilizar o script `script/figures.py` 
```bash
./figures.py
```
para gerar as figuras da maioria dos tópicos relevante. O script utiliza `mpld3` para gerar o server html na porta 8888, `http://localhost:8888`.
