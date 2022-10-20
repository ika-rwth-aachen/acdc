FROM rwthika/noetic-tf-2.8:latest
ARG TARGETARCH

USER root

RUN apt-get update && apt-get install -y \
    build-essential \
    llvm \
    libboost-all-dev \
    libeigen3-dev \
    qt5-default \
    libpcap-dev \
    libjpeg-dev \
    doxygen \
    tmux \
    git \
    wget \
    lua5.1-dev \
    mosquitto \
    mosquitto-clients \
    python3-paho-mqtt \
    && rm -rf /var/lib/apt/lists/*


# Setup ros specific libs
RUN apt-get update && apt-get install -y \
    ros-noetic-grid-map-core \
    ros-noetic-grid-map-msgs \
    ros-noetic-grid-map-ros \
    ros-noetic-grid-map-rviz-plugin \
    ros-noetic-image-geometry \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-pcl-ros \
    ros-noetic-image-pipeline \
    ros-noetic-rosbridge-library \
    ros-noetic-mqtt-bridge \
    ros-noetic-mqtt-client \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN git clone https://github.com/giaf/blasfeo.git && \
    cd blasfeo && \
    git checkout 0.1.1 && \
    mkdir build && \
    cd build && \
    cmake -DTARGET=GENERIC .. && \
    make && \
    make install && \
    cd .. && rm -rf blasfeo

RUN git clone https://github.com/giaf/hpipm.git && \
    cd hpipm && \
    git checkout 806c845 && \
    if [ $TARGETARCH = "arm64" ]; then sed -i "/TARGET = AVX/c\TARGET = GENERIC" Makefile.rule; fi  && \
    make static_library && \
    make examples && \
    make install_static && \
    cd .. && rm -rf hpipm

## install CppAD
RUN mkdir /tmp/cppadcg_deps && \
    cd /tmp/cppadcg_deps && \
    wget https://github.com/coin-or/CppAD/archive/20200000.3.tar.gz && \
    tar -xzf 20200000.3.tar.gz && \
    cd CppAD-20200000.3 && \
    mkdir build && \
    cd build && \
    cmake -Dcppad_prefix:PATH='/usr/local' .. && \
    make install && \
    rm -rf /tmp/cppadcg_deps

## install CppADCodeGen
RUN git clone https://github.com/joaoleal/CppADCodeGen.git /tmp/CppADCodeGen && \
    cd /tmp/CppADCodeGen && \
    git checkout v2.4.3 && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make && make install && \
    rm -rf /tmp/CppADCodeGen

# Install required libs for mqtt_bridge
RUN pip3 install inject

### --- INSTALL ROS2 on top of all other dependecies ------------------------------------------------------

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl \
    bash-completion \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install ROS2
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-foxy-desktop \
    ros-foxy-ros-base \
    ros-foxy-ros1-bridge \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

COPY TensorFlowConfig.cmake /usr/local/lib/cmake/tensorflow/TensorFlowConfig.cmake

RUN mkdir -p $DOCKER_HOME/ws/catkin_workspace
WORKDIR $DOCKER_HOME/ws/catkin_workspace
USER root 
