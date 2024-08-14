
## Setting OpenPCDet

- Execute the container
```
docker exec -it lidar3d-RT bash
```

- Install OpenPCDet based CenterPointPillar
``` shell
cd ~/PerceptionRT
python setup.py develop # sudo python setup.py develop
```

- To Build Python module, you have to install and wrap the c++ to python API.
``` shell
cd ~/
git clone https://github.com/pybind/pybind11.git
cd pybind11
cmake .
sudo make install

pip install --upgrade pip
sudo apt install python3-testresources

cd ~/PerceptionRT/inference_ros2/centerpoint
cmake -BRelease
cmake --build Release
```

## [Return to the main page.](../README.md)
