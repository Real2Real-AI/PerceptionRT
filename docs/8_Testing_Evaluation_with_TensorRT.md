## If you did not build at [Section 3_Setting_Dev_Environments](./3_Setting_Dev_Environments.md), Please build python-wrapped centerpoint for tensorRT.
``` shell
docker exec -it lidar3d-RT bash
cd ~/PerceptionRT/tools/tensorrt
cmake -BRelease && cmake --build Release
```

## Evaluation with TensorRT model
``` shell
cd ~/PerceptionRT
python object_detection/test.py --cfg_file {fullpath/config.yaml} --TensorRT
python object_detection/test.py --cfg_file {fullpath/config.yaml} --TensorRT --ckpt_dir {full_directory_path_with_onnx_and_config}

# local
cd ~/PerceptionRT
python object_detection/test.py --cfg_file /home/hyunkoo/DATA/HDD8TB/real2realAI/PerceptionRT/tools/cfgs/waymo_models/centerpoint_pillar_train_refactoring.yaml --TensorRT
python object_detection/test.py --cfg_file /home/hyunkoo/DATA/HDD8TB/real2realAI/PerceptionRT/tools/cfgs/waymo_models/centerpoint_pillar_train_refactoring.yaml --TensorRT --ckpt_dir /home/hyunkoo/DATA/HDD8TB/real2realAI/PerceptionRT/onnx
python object_detection/test.py --cfg_file /home/hyunkoo/DATA/HDD8TB/real2realAI/PerceptionRT/tools/cfgs/waymo_models/centerpoint_pillar_train_refactoring.yaml --TensorRT --ckpt_dir /home/hyunkoo/Dataset/HDD8TB/ttt

# docker
docker exec -it lidar3d-RT bash
cd ~/PerceptionRT
python object_detection/test.py --cfg_file /home/lidar3d/PerceptionRT/tools/cfgs/waymo_models/centerpoint_pillar_train_refactoring.yaml --TensorRT
python object_detection/test.py --cfg_file /home/lidar3d/PerceptionRT/tools/cfgs/waymo_models/centerpoint_pillar_train_refactoring.yaml --TensorRT --ckpt_dir /home/lidar3d/PerceptionRT/onnx
python object_detection/test.py --cfg_file /home/lidar3d/PerceptionRT/tools/cfgs/waymo_models/centerpoint_pillar_train_refactoring.yaml --TensorRT --ckpt_dir /Dataset/HDD8TB/ttt/
```
- Results as shown:
```
2024-07-07 05:33:34,115   INFO  
OBJECT_TYPE_TYPE_VEHICLE_LEVEL_1/AP: 0.5724 
OBJECT_TYPE_TYPE_VEHICLE_LEVEL_1/APH: 0.5667 
OBJECT_TYPE_TYPE_VEHICLE_LEVEL_1/APL: 0.5724 
OBJECT_TYPE_TYPE_VEHICLE_LEVEL_2/AP: 0.4969 
OBJECT_TYPE_TYPE_VEHICLE_LEVEL_2/APH: 0.4919 
OBJECT_TYPE_TYPE_VEHICLE_LEVEL_2/APL: 0.4969 
OBJECT_TYPE_TYPE_PEDESTRIAN_LEVEL_1/AP: 0.5827 
OBJECT_TYPE_TYPE_PEDESTRIAN_LEVEL_1/APH: 0.3159 
OBJECT_TYPE_TYPE_PEDESTRIAN_LEVEL_1/APL: 0.5827 
OBJECT_TYPE_TYPE_PEDESTRIAN_LEVEL_2/AP: 0.5036 
OBJECT_TYPE_TYPE_PEDESTRIAN_LEVEL_2/APH: 0.2730 
OBJECT_TYPE_TYPE_PEDESTRIAN_LEVEL_2/APL: 0.5036 
OBJECT_TYPE_TYPE_SIGN_LEVEL_1/AP: 0.0000 
OBJECT_TYPE_TYPE_SIGN_LEVEL_1/APH: 0.0000 
OBJECT_TYPE_TYPE_SIGN_LEVEL_1/APL: 0.0000 
OBJECT_TYPE_TYPE_SIGN_LEVEL_2/AP: 0.0000 
OBJECT_TYPE_TYPE_SIGN_LEVEL_2/APH: 0.0000 
OBJECT_TYPE_TYPE_SIGN_LEVEL_2/APL: 0.0000 
OBJECT_TYPE_TYPE_CYCLIST_LEVEL_1/AP: 0.3070 
OBJECT_TYPE_TYPE_CYCLIST_LEVEL_1/APH: 0.2551 
OBJECT_TYPE_TYPE_CYCLIST_LEVEL_1/APL: 0.3070 
OBJECT_TYPE_TYPE_CYCLIST_LEVEL_2/AP: 0.2953 
OBJECT_TYPE_TYPE_CYCLIST_LEVEL_2/APH: 0.2453 
OBJECT_TYPE_TYPE_CYCLIST_LEVEL_2/APL: 0.2953
```

## [Return to the main page.](../README.md)
