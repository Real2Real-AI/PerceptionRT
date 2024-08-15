ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG HOST_UID
ARG HOST_GID
ARG HOST_USER
ENV HOST_UID=${HOST_UID?-1000}
ENV GROUP_ID=${HOST_GID?-1000}
ENV HOST_USER=${HOST_USER}

RUN groupadd -g $HOST_GID $HOST_USER && \
    useradd -rm -u $HOST_UID -g $HOST_USER $HOST_USER && \
    usermod -aG sudo $HOST_USER && \
    echo "$HOST_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN sudo apt-get install -y libyaml-cpp-dev -y
RUN mkdir -p ~/ros2_humble/src && cd ~/ros2_humble
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
RUN sudo apt upgrade
RUN sudo rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN cd ~/ros2_humble/ && colcon build --symlink-install

# Set environment variables
#RUN echo "source /opt/ros/humble/setup.bash" >> /home/$HOST_USER/.bashrc
RUN echo "source ~/ros2_humble/install/local_setup.bash" >> /home/$HOST_USER/.bashrc

# ROS 2 환경 설정
#RUN echo "source /opt/ros/humble/setup.bash" >> ~/$HOST_USER/.bashrc
#SHELL ["/bin/bash", "-c"]
#
## OpenCV와 같은 추가적인 패키지 설치
#RUN apt-get update && apt-get install -y \
#    ros-humble-rviz2 \
#    ros-humble-turtlesim \
#    libopencv-dev
##RUN rm -rf /var/lib/apt/lists/*
#
#RUN pip install transforms3d