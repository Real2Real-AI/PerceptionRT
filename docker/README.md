# Guidance to use OpenPCDet with docker

You can either build the docker image through Dockerfile or pull the docker image from dockerhub. Please make sure nvidia-docker is corretly installed.

## Build Through Dockerfile
Build the docker base image.
```shell script
cd PerceptionRT
docker build -f docker/lidar3d/env.Dockerfile -t perception-rt-lidar3d-env docker/
```

## Container Start
- If you want to change account name of container, Before create the container
  - please change the `HOST_USER` from `lidar` to `you want` in the `.env` file of `root dir` (CenterPointPillar)
  - Fill in the `.env` file with the GID, UID, and username like below.
``` shell
HOST_UID=1000
HOST_GID=1000
HOST_USER=lidar3d
```````
- Create the container.
``` shell
cd docker/lidar3d && docker compose up --build -d
```

- Execute the container
```
docker exec -it lidar3d-RT bash
```

- Get your GID, UID, and username in the container env 
``` shell
# GID -> 1000
echo $(id -g)  
# UID -> 1000
echo $(id -u)
# username -> lidar3d
echo $(id -un)
```








