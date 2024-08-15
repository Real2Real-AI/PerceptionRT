## (Python) Inference a model with Pytorch Models on ROS2 Node

### 1. Setting ROS2
- If you are not using a container env, 
  - please install ROS2 Humble as follows: 
    - [Ubuntu (source)](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
    - [Ubuntu (Debian packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- If you are using a container env, your container already installed ROS2.

### 2. Convert Waymo dataset to Rosbag for ROS2
- You can use two repositories step by step. 
  - Waymo data ==> Rosbag for ROS1: https://github.com/yukke42/waymo2bag
    - 첫번째 깃허브로 ROS1 bag 파일로 만든 후
  - Rosbag for ROS1 ==> Rosbag for ROS2: https://ternaris.gitlab.io/rosbags/topics/convert.html
    - 2번째 라이브러리로 ros1을 ros2 bag파일로 변환하면 됨
  

### 3. ROS2 play bagfile on the container
```
docker exec -it lidar3d-RT bash
cd /Dataset
ros2 bag play segment-10359308928573410754_720_000_740_000_with_camera_labels/  # ros2 bag play folder_with_ros2bag
```

### 4. Run ros2_demo.py on the container
``` shell
docker exec -it lidar3d-RT bash
cd ~/PerceptionRT
python tools/visualization/ros2rosbag_rviz2.py --cfg_file {fullpath/config.yaml} --ckpt {fullpath/pytorch_models.pth}
python tools/visualization/ros2rosbag_rviz2.py --cfg_file ~/PerceptionRT/tools/cfgs/waymo_models/centerpoint_pillar_train_refactoring.yaml --ckpt ~/PerceptionRT/ckpt/checkpoint_epoch_24.pth

```

### 5. Run rviz2
``` shell
docker exec -it lidar3d-RT bash
rviz2
```

### 6. Setting rviz2
- Fixed Frame: base_link
- Add -> By display type -> PountCloud2 -> Topic: /lidar/top/pointcloud, Size(m): 0.03
- Add -> By topic -> /boxes/MarkerArray

<img src="../sources/rviz2_add_topic.png" align="center" width="359">

### 7. Results with rviz2
<img src="./sources/rviz2.png" align="center" width="100%">

## [Return to the main page.](../README.md)
