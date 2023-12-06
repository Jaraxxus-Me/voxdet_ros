# VoxDet-ros
This is a ROS package for VoxDet, which simply utilizes the toolbox [MMDetection](https://github.com/open-mmlab/mmdetection) of [OpenMMLab](https://openmmlab.com/).
You can deploy VoxDet on real-robots using this library!

### Requirements
#### Build your own env (Not recommended)
- ROS Noetic
- Python 3.8+, PyTorch 1.2+, CUDA 11.3+ and [MMCV](https://mmcv.readthedocs.io/en/latest/#installation)
#### Use our docker (Strongly recommended, it is super easy)
- [Docker image]([bowenli1024/voxdet:ros-v1](https://hub.docker.com/layers/bowenli1024/voxdet/ros-v1/images/sha256-77b1d0d6f33a05b4e8ab64893ca328d6df3fd8f34803caa2403ff4b3f3ffe89a?context=repo))
	```
	docker pull bowenli1024/voxdet:ros-v1
	```



### Installation

1. Clone all needed packages: this package, VoxDet libraries, [Bop_toolkit](https://mega.nz/file/BAEj3TgS#yzwX2AHUg9CtCsmDV17rxVkmFhw4mh34y6gvQ3FDS4E), vision_msgs, and realsense-ros
    ```shell
    cd {ROS WORKSPACE}/src
    # this package and vision_msgs
    git clone git@github.com:Jaraxxus-Me/voxdet_ros.git
    git clone https://github.com/ros-perception/vision_msgs.git
    cd voxdet_ros
    # VoxDet library    
    git clone https://github.com/Jaraxxus-Me/VoxDet.git
    cd VoxDet
    pip install -e -v .
    # Modified BOP_tookit, use the link above
    cd bop_toolkit
    pip install -e .
    # realsense-ros
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd realsense-ros/
    git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
    cd ..
    ```

2. make, build, and install the ros packages

   ```bash
    cd ..
    catkin_make clean
    catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
    catkin_make install
    catkin_make
   ```
3. Install rospkg.

   ```shell
   pip install rospkg
   ```



### Prepare phase1 support information

1. You need to generate the mask and rotations for the phase 1 references the desired path look like:
    ```
    src/voxdet_ros
        VoxDet/
            data/
                [Your Data Folder]/
                    test_video (similar to the lmo and ycbv)
    ```
    Then, modify the `~p1_path` in the `mmdetector.launch` file.
   
3. Use our tool to get and `.npz` file for each instance
    ```
    python3 src/voxdet_ros/VoxDet/tools/pre_save_p1_bop.py #change the path and object id accordingly
    ```
   

### ROS Interfaces

#### params

- `~publish_rate`: the debug image publish rate. default: 30hz
- `~is_service`: whether or not to use service instead of subscribe-publish. default: False
- `~visualization`: whether or not to show the debug image. default: True
- `~p1_path`: support information path. Use one specific instance from the prepared folder (the directory to info.npz)

#### topics

- `~debug_image`: publish the debug image
- `~objects`: publish the inference result, containing the information of detected objects
- `~image`: subscribe the input image. The default one is `/camera/color/image_raw` for Realsense camera.



### Usage

- In terminal 0, start roscore
    ```shell
    source /opt/ros/noetic/setup.bash
    roscore
    ```

- In terminal 1, start rviz
    ```shell
    rviz # load the rviz file in src/voxdet_ros/rviz/test_image.rviz
    ```

- In terminal 2, start realsense camera

    ```shell
    cd src
    bash voxdet_ros/scripts/start_camera.sh
    ```

- In terminal 3, start detector node

    First specify the config file and model checkpoint in mmdetector.py

    ```shell
    roslaunch voxdet_ros mmdetector.launch
    ```

    You will see visualization of both original input and detector output.

    Here is a demo video
    

https://github.com/Jaraxxus-Me/voxdet_ros/assets/56875617/7a2afa18-49a3-4f1a-b21e-fc490eee9ac2

## Reference
If our work inspires your research, please cite us as:

```
@INPROCEEDINGS{Li2023vox,       
	author={Li, Bowen and Wang, Jiashun and Hu, Yaoyu and Wang, Chen and Scherer, Sebastian},   
	booktitle={Proceedings of the Advances in Neural Information Processing Systems (NeurIPS)}, 
	title={{VoxDet: Voxel Learning for Novel Instance Detection}},
	year={2023},
	volume={},
	number={}
}
```
