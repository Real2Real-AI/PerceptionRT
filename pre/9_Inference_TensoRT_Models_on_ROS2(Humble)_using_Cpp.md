
## (C++) Inference a model with TensorRT on ROS2 Node

### 1. Setting ROS2
- If you are not using a container env, 
  - please install ROS2 Humble as follows: 
    - [Ubuntu (source)](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
    - [Ubuntu (Debian packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- If you are using a container env, your container already installed ROS2.

### 2. Build the ROS2 package of centerpoint c++ Node in your ROS2 workspace.
``` shell
docker exec -it lidar3d-RT bash
cd ~/ && mkdir -p ros2_humble/src && cd ros2_humble/ && colcon build --symlink-install
cd src && ln -s ~/PerceptionRT/tools/ros2/centerpoint/ .
```

- If not exist `centerpoint/models`, copy `onnx` file.
``` shell
cd centerpoint && mkdir models
cd models
ln -s ~/PerceptionRT/onnx/config.yaml config.yaml
ln -s ~/PerceptionRT/onnx/model.onnx model.onnx  
```
- Build the ROS2 package of centerpoint c++ Node in your ROS2 workspace.
``` shell
cd ~/ros2_humble && colcon build --symlink-install

cd ~/ros2_humble
colcon build --symlink-install --packages-select centerpoint

source ~/.bashrc
ros2_ws/install/setup.bash
```

### 3. Run the ROS2 Node.
``` shell
docker exec -it lidar3d-RT bash
source ~/ros2_ws/install/setup.bash
ros2 launch centerpoint centerpoint.launch.py
```

- Once running ros2 centerpoint node, create tensorRT file to the same folder having onnx file, automatically.

### 4. ROS2 play bagfile on the container
```
docker exec -it lidar3d-RT bash
cd /Dataset
ros2 bag play segment-10359308928573410754_720_000_740_000_with_camera_labels/  # ros2 bag play folder_with_ros2bag
```

### 5. Run rviz2
``` shell
docker exec -it lidar3d-RT bash
rviz2
```
- Fixed Frame: base_link
- Add -> By display type -> PountCloud2 -> Topic: /lidar/top/pointcloud, Size(m): 0.03
- Add -> By topic -> /boxes/MarkerArray

<img src="./sources/rviz2_ros_cpp.png" align="center" width="100%">

## [Return to the main page.](../README.md)
