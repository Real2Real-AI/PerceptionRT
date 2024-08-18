
# Docker 이미지 저장 장소 변경하기 
## Changing Docker Image Storage Location
- 이 가이드는 Docker 이미지 및 컨테이너 데이터를 기본 경로가 아닌 사용자 지정 경로에 저장하는 방법을 설명합니다.
  - This guide explains how to store Docker images and container data in a custom directory instead of the default path.

## 1. Docker 서비스 중지 (Stop Docker Service)
- 먼저, Docker 서비스를 중지합니다. 이는 안전하게 설정을 변경하기 위함입니다.
  - First, stop the Docker service. This is necessary to change the settings safely.

```bash
sudo systemctl stop docker
```

## 2. 새로운 디렉토리 생성 (Create a New Directory)

- 이미지와 컨테이너 데이터를 저장할 새로운 디렉토리를 생성합니다. 예를 들어 `/mnt/docker-storage`에 저장하려면 다음 명령어를 사용하세요
  - Create a new directory to store images and container data. For example, to store data in `/mnt/docker-storage`, use the following command
```bash
sudo mkdir -p /mnt/docker-storage
```

## 3. Docker 설정 파일 수정 (Modify Docker Configuration File)
- Docker의 설정 파일인 `daemon.json`을 편집합니다. 이 파일의 경로는 `/etc/docker/daemon.json`입니다. 파일이 없으면 새로 생성합니다.
  - Edit the Docker configuration file, `daemon.json`. The path to this file is `/etc/docker/daemon.json`. If the file does not exist, create it.
```bash
sudo nano /etc/docker/daemon.json
```
- 파일에 다음 내용을 추가하거나 수정하여 Docker의 데이터 루트를 변경합니다
  - Add or modify the following content in the file to change Docker's data root
```json
{
  "data-root": "/mnt/docker-storage"
}
```

## 4. 기존 데이터 이동 (선택 사항) (Move Existing Data - Optional)
- 기존의 Docker 이미지와 컨테이너 데이터를 새 경로로 이동하려면 다음 명령어를 사용하여 데이터를 복사할 수 있습니다.
  - If you want to move the existing Docker images and container data to the new path, you can use the following command to copy the data.
```bash
sudo rsync -aP /var/lib/docker/ /mnt/docker-storage/
```

## 5. Docker 서비스 재시작 (Restart Docker Service)
- 설정을 완료한 후, Docker 서비스를 재시작합니다
  - After completing the configuration, restart the Docker service
```bash
sudo systemctl start docker
```

## 6. 설정 확인 (Verify Configuration)
- Docker가 새로운 경로를 사용하고 있는지 확인하려면 다음 명령어로 확인할 수 있습니다
  - To verify that Docker is using the new path, run the following command
```bash
docker info | grep "Docker Root Dir"
```

- 출력된 경로가 새로 설정한 디렉토리(`/mnt/docker-storage`)로 되어 있으면 설정이 완료된 것입니다.
  - If the output path is the newly set directory (`/mnt/docker-storage`), the configuration is complete.
- 이제 Docker는 이미지를 포함한 모든 데이터를 새로운 경로에 저장하게 됩니다.
  - Now, Docker will store all data, including images, in the new path.
