# Use a imagem oficial do ROS Noetic como base
FROM osrf/ros:noetic-desktop-full

# Instale as dependências do Gzweb
RUN apt-get update && apt-get install -y \
    nodejs \
    npm \
    libjansson-dev \
    libboost-dev \
    imagemagick \
    libtinyxml-dev \
    mercurial \
    cmake \
    build-essential

# Clone o repositório Gzweb e construa
RUN hg clone https://bitbucket.org/osrf/gzweb ~/gzweb
RUN cd ~/gzweb && hg up gzweb_1.4.0 && ./deploy.sh -m

