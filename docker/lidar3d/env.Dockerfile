FROM nvcr.io/nvidia/tensorrt:23.04-py3

# Set environment variables
ENV NVENCODE_CFLAGS "-I/usr/local/cuda/include"
ENV CV_VERSION=4.2.0
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color

# 환경 변수 설정 ROS2
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Get all dependencies including ROS 2 Humble dependencies
# 필수 도구 및 패키지 관리
RUN apt-get update && apt-get install -y build-essential cmake pkg-config software-properties-common autoconf automake libtool curl gnupg2 sudo bash-completion locales lsb-release debconf xfce4-terminal

# 버전 관리 및 텍스트 편집기, 기타 유틸리티
RUN apt-get install -y git neovim zip unzip

# 네트워크 및 시스템 라이브러리
RUN apt-get install -y libc6 libssl-dev libcap2 libusb-1.0-0 libomp5 libstdc++6

# GUI 관련 라이브러리
RUN apt-get install -y libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 libatk-adaptor libgtk2.0-dev

# 미디어 및 이미지 처리 라이브러리
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

# 수학 및 과학 계산 라이브러리
RUN apt-get install -y libboost-all-dev libtbb2 libtbb-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev

# ROS 관련 라이브러리
RUN apt-get install -y libpcl-dev python3-pcl pcl-tools

# Python 관련 패키지
RUN apt-get install -y python3-pip python3-tornado python3-dev python3-numpy python3-virtualenv libpython3-dev

# 기타 유틸리티
RUN rm -rf /var/lib/apt/lists/*

#
## Get all dependencies
#RUN apt-get update && apt-get install -y \
#    git zip unzip libssl-dev libcairo2-dev lsb-release libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev software-properties-common \
#    build-essential cmake pkg-config libapr1-dev autoconf automake libtool curl libc6 libboost-all-dev debconf libomp5 libstdc++6 \
#    libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 libcap2 libusb-1.0-0 libatk-adaptor neovim \
#    python3-pip python3-tornado python3-dev python3-numpy python3-virtualenv libpcl-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev \
#    libsuitesparse-dev python3-pcl pcl-tools libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev \
#    libpng-dev libtiff-dev libdc1394-22-dev xfce4-terminal bash-completion sudo
#
## ROS 2 Humble 설치를 위한 종속성 설치
#RUN apt-get update && apt-get install -y \
#    curl \
#    gnupg2 \
#    lsb-release \
#    build-essential \
#    cmake \
#    git \
#    sudo \
#    locales \
#    libpython3-dev \
#    python3-pip \
#    && rm -rf /var/lib/apt/lists/*

# OpenCV
WORKDIR /opencv
RUN git clone https://github.com/opencv/opencv.git -b $CV_VERSION

WORKDIR /opencv/opencv/build

RUN cmake .. &&\
make -j12 &&\
make install &&\
ldconfig &&\
rm -rf /opencv

WORKDIR /
ENV OpenCV_DIR=/usr/share/OpenCV

# PyTorch for CUDA 12.1
RUN pip install torch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 --index-url https://download.pytorch.org/whl/cu121
ENV TORCH_CUDA_ARCH_LIST="5.0;6.0;6.1;7.0;7.5;8.0;8.6;8.9+PTX"

# OpenPCDet Dependencies
RUN apt remove python3-blinker -y
RUN pip install -U pip
RUN pip install numpy==1.23.0 llvmlite numba tensorboardX easydict pyyaml scikit-image tqdm SharedArray open3d==0.16.0 mayavi av2 kornia==0.6.8 pyquaternion colored
RUN pip install spconv-cu120
RUN pip install opencv-python==4.2.0.34
RUN pip install onnx==1.16.0
RUN pip install onnxsim==0.4.36
RUN pip install onnx_graphsurgeon --extra-index-url https://pypi.ngc.nvidia.com
RUN pip install waymo-open-dataset-tf-2-12-0

ENV NVIDIA_VISIBLE_DEVICES="all" \
    NVIDIA_DRIVER_CAPABILITIES="all"

# 1. 로케일 설정
RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. ROS2 apt 리포지토리 추가
RUN apt-get software-properties-common libyaml-cpp-dev -y && \
    add-apt-repository universe && \
##########################
RUN apt-get install software-properties-common libyaml-cpp-dev -y && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# 3. 개발 도구 및 ROS 도구 설치
RUN apt-get update && apt-get install -y python3-flake8-docstrings python3-pip python3-pytest-cov ros-dev-tools

RUN python3 -m pip install -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    "pytest>=5.3" \
    pytest-repeat \
    pytest-rerunfailures




