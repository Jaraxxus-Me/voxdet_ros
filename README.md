# VoxDet-ros
This is a ROS package for instance object detection, which simply utilizes the toolbox [MMDetection](https://github.com/open-mmlab/mmdetection) of [OpenMMLab](https://openmmlab.com/).

### Requirements

- ROS Noetic
- Python 3.8+, PyTorch 1.2+, CUDA 11.3+ and [MMCV](https://mmcv.readthedocs.io/en/latest/#installation)
- [Docker image](bowenli1024/voxdet:ros-v1)



### Installation

1. Clone all needed packages: this package, detector libraries, vision_msgs, and realsense-ros
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

1. Generate mask from video

   In `[ROS workspace]/src/zid3d_ros/PoolNet`, open `create_bop.py`, change the video, image path, then run:

   ```shell
   python create_bop.py
   ```

   You'll get RGB and masks in your specified folder

2. Compress all the information into info.npz

   ```shell
   cd src/zid3d_ros/Zid3D
   python tools/pre_save_p1_demo.py # remember to configure the p1 support path and instance id
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
    rviz # load the rviz file in src/zid3d_ros/rviz/test_image.rviz
    ```

- In terminal 2, start realsense camera

    ```shell
    cd src
    bash zid3d_ros/scripts/start_camera.sh
    ```

- In terminal 3, start detector node

    First specify the config file and model checkpoint in mmdetector.py

    ```shell
    roslaunch zid3d_ros mmdetector.launch
    ```

    You will see visualization of both original input and detector output.
