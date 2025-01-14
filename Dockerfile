# Use a build argument for the ROS version
ARG ROS_VERSION=noetic

# Use the specified ROS version as the base image
FROM ros:${ROS_VERSION}-robot

# after every FROM statement, all the ARGs are no longer available, this solves it:
ARG ROS_VERSION

# Update package lists and install dependencies
RUN apt-get update 
RUN apt-get install -y sudo 
RUN apt-get install -y build-essential 
RUN apt-get install -y gfortran
RUN apt-get install -y git 
RUN apt-get install -y curl 
RUN apt-get install -y python3-tk 
RUN apt-get install -y python3-pip 
RUN apt-get install -y libjpeg-dev 
RUN apt-get install -y wget 
RUN apt-get install -y patchelf

RUN apt-get install -y libassimp-dev liblapack-dev libblas-dev libyaml-cpp-dev libmatio-dev
# require for cartesio_acceleration_support
RUN apt-get install -y ros-${ROS_VERSION}-interactive-markers ros-${ROS_VERSION}-moveit
# required for casadi python bindings
RUN apt-get install -y swig
# required for realsense
RUN apt-get install -y ros-noetic-gazebo-ros-pkgs

# Install ROS catkin tools
RUN apt-get install -y ros-${ROS_VERSION}-catkin

# Upgrade pip and install the latest version of NumPy 
# only required for ROS noetic
RUN pip3 install --upgrade pip \
    && pip3 install --upgrade numpy

RUN pip3 install hhcm-forest

# required packages
RUN pip3 install scipy numpy_ros matplotlib

# Check NumPy version
RUN python3 -c "import numpy; print('NumPy version:', numpy.__version__)"

# Verify CMake installation
RUN cmake --version

# required for displaying matplotlib plots
ENV DISPLAY :1
# add user with sudo privileges which is not prompted for password
RUN adduser --disabled-password --gecos '' user
RUN adduser user sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user
WORKDIR /home/user
ENV PATH="/home/user/.local/bin:${PATH}"

# install catkin workspace
WORKDIR /home/user/
RUN mkdir -p kyon_ws/src
RUN cd ~/kyon_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_VERSION}/setup.sh && cd ~/kyon_ws && catkin_make install

# source workspace for ROS_PACKAGE_PATH (source catkin before other source)
RUN echo "source ~/kyon_ws/devel/setup.bash" >> ~/.bashrc

# set up forest for installation
RUN mkdir forest_ws

WORKDIR /home/user/forest_ws
RUN forest init 

# source forest
RUN echo "source ~/forest_ws/setup.bash" >> ~/.bashrc

# # restart bash to make the source effective
SHELL ["bash", "-ic"]

RUN echo $ROS_PACKAGE_PATH

# add recipes
RUN forest add-recipes git@github.com:ADVRHumanoids/multidof_recipes.git --tag master

RUN forest grow pybind11 --clone-protocol https -j7

# install horizon
RUN forest grow horizon --clone-protocol https -j7 --verbose --tag receding_horizon

# clone and install phase_manager
RUN forest grow phase_manager --clone-protocol https -j7 --verbose --tag new_architecture

# clone and install and xbot2_mujoco
RUN forest grow xbot2_mujoco --clone-protocol https -j7 --verbose --tag 3.x

# clone and install and mujoco_cmake
RUN forest grow mujoco_cmake --clone-protocol https -j7 --verbose --tag 3.x


# WORKDIR /home/user/
# update keys and install xbot2
RUN sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
RUN wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install -y xbot2_desktop_full

# source xbot2 for ROS_PACKAGE_PATH
RUN echo ". /opt/xbot/setup.sh" >> ~/.bashrc

# # restart bash to make the source effective
SHELL ["bash", "-ic"]

# clone talos robot utilities
RUN cd ~/forest_ws/ros_src && git clone https://github.com/pal-robotics/talos_robot.git

# clone talos_mujoco
RUN cd ~/forest_ws/ros_src && git clone https://github.com/hucebot/talos_cartesio_config.git



