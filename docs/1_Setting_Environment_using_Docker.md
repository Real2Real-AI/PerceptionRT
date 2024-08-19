## 1) Set Environment

### 1.1 Install Docker Engine on Ubuntu
- Please refer to the [`docker.docs`](https://docs.docker.com/engine/install/ubuntu/) for more details.
- If you would like to know more details, please refer to:
  - [`install guide for nvidia container toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) 
  - [`nvidia container toolkit`](https://github.com/NVIDIA/nvidia-container-toolkit?tab=readme-ov-file) 
- If you want to change `Docker Image Storage Location`, please follow ths [link](./0_Docker_Storage_Setup_KR_ENG.md)
- docker 설치 후 /var/run/docker.sock의 permission denied 발생하는 경우
``` shell
sudo chmod 666 /var/run/docker.sock
```


### 1.2 Clone this repository
``` shell
git clone https://github.com/Real2Real-AI/PerceptionRT.git
```
### 1.3 Docker Container Start

- Build the docker base image
```shell script
cd PerceptionRT
docker build -f docker/lidar3d/env.Dockerfile -t perception-rt-lidar3d-env docker/
```

- Create the container.
``` shell
cd docker/lidar3d && docker compose up --build -d
```

- Execute the container
```
docker exec -it lidar3d-RT bash
```

- Please refer to the [docker/README.md](docker/README.md) for more details.

## [Return to the main page.](../README.md)