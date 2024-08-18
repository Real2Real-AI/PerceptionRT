FROM nvcr.io/nvidia/tensorrt:23.04-py3

# Set environment variables
ENV NVENCODE_CFLAGS "-I/usr/local/cuda/include"
ENV CV_VERSION=4.2.0
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV OpenCV_DIR=/usr/share/OpenCV
ENV NVIDIA_VISIBLE_DEVICES="all"
ENV NVIDIA_DRIVER_CAPABILITIES="all"
ENV TORCH_CUDA_ARCH_LIST="5.0;6.0;6.1;7.0;7.5;8.0;8.6;8.9+PTX"

# 1. 로케일 설정
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# 로케일 확인을 위한 명령어 추가 (옵션)
RUN echo "LANG=$LANG" && echo "LC_ALL=$LC_ALL" && locale

# Install essential tools and libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    software-properties-common \
    libyaml-cpp-dev \
    autoconf \
    automake \
    libtool \
    curl \
    gnupg2 \
    sudo \
    bash-completion \
    locales \
    lsb-release \
    debconf \
    xfce4-terminal \
    git \
    neovim \
    zip \
    unzip \
    libc6 \
    libssl-dev \
    libcap2 \
    libusb-1.0-0 \
    libomp5 \
    libstdc++6 \
    libqt5core5a \
    libqt5xml5 \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5concurrent5 \
    libqt5opengl5 \
    libatk-adaptor \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    libboost-all-dev \
    libtbb2 \
    libtbb-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libpcl-dev \
    python3-pcl \
    pcl-tools \
    python3-pip \
    python3-tornado \
    python3-dev \
    python3-numpy \
    python3-virtualenv \
    libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV from source
WORKDIR /opencv
RUN git clone https://github.com/opencv/opencv.git -b $CV_VERSION
WORKDIR /opencv/opencv/build
RUN cmake .. && \
    make -j12 && \
    make install && \
    ldconfig && \
    rm -rf /opencv

# Install PyTorch for CUDA 12.1
RUN pip install torch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 --index-url https://download.pytorch.org/whl/cu121

# Install OpenPCDet dependencies
RUN pip install -U pip
RUN pip install numpy==1.23.0
RUN pip install llvmlite numba tensorboardX easydict pyyaml scikit-image tqdm SharedArray
#RUN pip install open3d==0.16.0
RUN pip install mayavi av2
RUN pip install kornia==0.6.8
RUN pip install pyquaternion colored
RUN pip install spconv-cu120
RUN pip install opencv-python==4.2.0.34
RUN pip install onnx==1.16.0
RUN pip install onnxsim==0.4.36
RUN pip install onnx_graphsurgeon --extra-index-url https://pypi.ngc.nvidia.com
RUN pip install waymo-open-dataset-tf-2-12-0

## Locale setup
#RUN locale-gen en_US en_US.UTF-8 && \
#    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
#
## Add ROS2 apt repository and install ROS2 Humble
#RUN apt-get update && apt-get install curl -y && \
#    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
#RUN apt-get update -y
#RUN apt-get upgrade -y
#RUN apt-get install -y \
#    ros-humble-desktop \
#    python3-argcomplete \
#    ros-humble-pcl-conversions \
#    ros-humble-pcl-msgs \
#    ros-humble-pcl-ros \
#    && rm -rf /var/lib/apt/lists/*
#
## Install ROS2 development tools
#RUN apt-get update && apt-get install -y \
#    python3-flake8-docstrings \
#    python3-pytest-cov \
#    ros-dev-tools
#
## Install Python packages
#RUN python3 -m pip install -U \
#    flake8-blind-except \
#    flake8-builtins \
#    flake8-class-newline \
#    flake8-comprehensions \
#    flake8-deprecated \
#    flake8-import-order \
#    flake8-quotes \
#    "pytest>=5.3" \
#    pytest-repeat \
#    pytest-rerunfailures

# Set default command
#CMD ["/bin/bash"]