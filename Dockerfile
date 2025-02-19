# syntax=docker/dockerfile:1.4
# Use a build argument for the ROS version
ARG ROS_VERSION=noetic

# Use the specified ROS version as the base image
FROM ros:${ROS_VERSION}-robot
ENV DEBIAN_FRONTEND=noninteractive

# after every FROM statement, all the ARGs are no longer available, this solves it:
ARG ROS_VERSION

# Update package lists and install dependencies
RUN apt-get update && apt-get install -y sudo build-essential gfortran \
    git curl python3-tk python3-pip libjpeg-dev wget patchelf nano libglfw3-dev \
    locate terminator gedit

RUN apt-get install -y libassimp-dev liblapack-dev libblas-dev libyaml-cpp-dev libmatio-dev
# require for cartesio_acceleration_support
RUN apt-get install -y ros-${ROS_VERSION}-interactive-markers ros-${ROS_VERSION}-moveit
# required for casadi python bindings
RUN apt-get install -y swig
# required for realsense
RUN apt-get install -y ros-noetic-gazebo-ros-pkgs
# clang to generate mpc functions in c++
RUN apt-get install -y clang

# Install ROS catkin tools
RUN apt-get install -y ros-${ROS_VERSION}-catkin

# install joy
RUN apt-get install -y ros-${ROS_VERSION}-joy

# install graphviz-dev for horizon urdf modifier
RUN apt-get install -y graphviz-dev

# Upgrade cmake for forest
RUN wget https://github.com/Kitware/CMake/releases/download/v3.31.4/cmake-3.31.4-linux-x86_64.tar.gz
RUN tar -xvf cmake-3.31.4-linux-x86_64.tar.gz 
RUN cd cmake-3.31.4-linux-x86_64 && sudo cp -r bin/* /usr/local/bin/
RUN cd cmake-3.31.4-linux-x86_64 && sudo cp -r share/* /usr/local/share/

# Upgrade pip and install the latest version of NumPy 
# only required for ROS noetic
RUN pip3 install --upgrade pip \
    && pip3 install --upgrade numpy \
    && pip3 install --upgrade scipy

RUN pip3 install hhcm-forest

# required packages
RUN pip3 install numpy_ros matplotlib
RUN pip3 install colorama Cython networkx pygraphviz

# Check NumPy version
RUN python3 -c "import numpy; print('NumPy version:', numpy.__version__)"

# Verify CMake installation
RUN cmake --version


ARG HOST_UID
RUN test -n "$HOST_UID"
#RUN addgroup --gid $HOST_UID groupcontainer

ARG USER_NAME=user
# add user
RUN adduser --uid $HOST_UID --disabled-password --gecos '' ${USER_NAME} \
    && adduser ${USER_NAME} sudo \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers \
    && usermod -aG sudo ${USER_NAME} \
    && mkdir -p /home/${USER_NAME}/.ssh \
    && chmod 700 /home/${USER_NAME}/.ssh \
    && touch /home/${USER_NAME}/.ssh/known_hosts \
    && ssh-keyscan github.com >> /home/${USER_NAME}/.ssh/known_hosts \
    && chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}/.ssh

RUN mkdir -p /home/${USER_NAME}/horizon_ws
RUN chown ${USER_NAME} /home/${USER_NAME}/horizon_ws

# update keys and install xbot2
RUN sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
RUN wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install -y xbot2_desktop_full

user ${USER_NAME}
ENV PATH="/home/${USER_NAME}/.local/bin:${PATH}"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo ". /opt/xbot/setup.sh" >> ~/.bashrc

WORKDIR  /home/${USER_NAME}/horizon_ws
RUN forest init 

# source forest
RUN echo "source ~/horizon_ws/setup.bash" >> ~/.bashrc

# restart bash to make the source effective
SHELL ["bash", "-ic"]

RUN echo $ROS_PACKAGE_PATH

# break cache for re-installation of casadi (to always download the last version)
# ARG CACHE_DATE="date"

# add recipes
RUN forest add-recipes git@github.com:ADVRHumanoids/multidof_recipes.git --clone-protocol https --tag fr_recipes

# install pybind
RUN forest grow pybind11 --clone-protocol https -j7

# install hpp-fcl
RUN forest grow hpp-fcl --clone-protocol https -j7

#install pinocchio
RUN forest grow pinocchio --clone-protocol https -j7

# install casadi
RUN forest grow casadi --clone-protocol https -j7

# install casadi_kin_dyn
RUN forest grow casadi_kin_dyn --clone-protocol https -j7

# install horizon
RUN forest grow horizon --clone-protocol https -j7

# clone and install phase_manager
RUN forest grow phase_manager --clone-protocol https -j7

# clone and install and mujoco_cmake
RUN forest grow mujoco_cmake --clone-protocol https -j7

RUN echo $CMAKE_PREFIX_PATH

RUN forest grow xbot2_mujoco --clone-protocol https -j7

RUN forest grow talos_horizon --clone-protocol https -j7

ARG TALOS_ROBOT_BRANCH=1.1.32
ARG TALOS_CARTESIO_CONFIG_BRANCH=horizon_demo

RUN mkdir ros_src
# # clone talos robot utilities
RUN cd ~/horizon_ws/ros_src && git clone --single-branch --branch ${TALOS_ROBOT_BRANCH} https://github.com/pal-robotics/talos_robot.git

# # clone talos_mujoco
ARG TALOS_CARTESIO_CONFIG=horizon_demo
RUN cd ~/horizon_ws/ros_src && git clone --single-branch --branch ${TALOS_CARTESIO_CONFIG} https://github.com/hucebot/talos_cartesio_config.git

RUN sudo apt-get update && sudo apt-get install -y mesa-utils
# restart bash to make the source effective
SHELL ["bash", "-ic"]

RUN set_xbot2_config /home/user/horizon_ws/ros_src/talos_cartesio_config/mujoco/xbot2/hal/talos_mj.yaml

RUN echo 'export __NV_PRIME_RENDER_OFFLOAD=1' >> ~/.bashrc
RUN echo 'export __GLX_VENDOR_LIBRARY_NAME=nvidia' >> ~/.bashrc
RUN echo 'export __VK_LAYER_NV_optimus=NVIDIA_only' >> ~/.bashrc
RUN echo 'export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json' >> ~/.bashrc
