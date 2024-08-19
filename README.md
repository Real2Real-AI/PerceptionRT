# PerceptionRT
Open Source for Real-Time 2D/3D Perception

## Proceeding List until Phase #2 - lidar (Centerpoint-pillar)
- [X] Sensor Types
  - [X] Lidar based 3D Object Detection
- [X] Detection & Tracking
  - [X] Single Frame based Detection
- [X] Datasets
  - [X] Waymo
- [X] Training
  - [X] single GPU
  - [X] multi GPUs
- [X] Evaluation
  - [X] mAP 
- [X] Inference
  - [X] pytorch
    - [X] python
    - [ ] c++
  - [X] onnx
    - [X] python
    - [ ] c++ 
  - [X] tensorRT + ROS2
    - [X] python
    - [X] c++
  - [X] 2D/3D Visualization
    - [X] ROS2(Rosbag + rviz2)
        - [X] python
        - [X] c++
    - [X] Open3d + npz files
      - [X] python
      - [ ] c++
- [X] Code Refactoring
  - [X] python codes
  - [X] Remove and clean up duplicate files and codes in the `centerpoint` (cuda, cpp, python) codes for ros2 and pybind11  

## To Do List for Phase #3 - lidar
- [ ] 2D/3D Visualization
  - [ ] Rerun.io + npz files 
      - [ ] python
      - [ ] c++- 
- [ ] Code Refactoring  
  - [ ] cleaning all codes
  - [ ] add description
  - [ ] add test codes

    
## TODO List for Next Phases
- [ ] Sensor Types
  - [ ] Single Camera based 2D Object Detection
  - [ ] Multi Camera based 3D Object Detection
- [ ] Detection & Tracking
  - [ ] Multi Frames based Detection
  - [ ] Tracking
- [ ] Datasets
  - [ ] KITTI
  - [ ] nuScenes
- [ ] Training
  - [ ] knowledge distillation
- [ ] Evaluation
  - [ ] Precision / Recall / F1 measure
  - [ ] mIOU / ...
  - [ ] HOTA / MOTA / ...
- [ ] 2D/3D Visualization
  - [ ] Open3D
  - [ ] VTK
  - [ ] Rerun.io
