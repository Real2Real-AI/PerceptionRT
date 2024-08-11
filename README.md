# PerceptionRT
Open Source for Real-Time 2D/3D Perception

## Proceeding List for Phase #1 - lidar (Centerpoint-pillar)
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
  - [X] ROS2 Rviz 
- [X] Code Refactoring
  - [X] python codes

## Proceeding List for Phase #2 - lidar (Centerpoint-pillar)
- [X] Code Refactoring
  - [ ] Remove and clean up duplicate files and codes in the `centerpoint` (cuda, cpp, python) codes for ros2 and pybind11
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
