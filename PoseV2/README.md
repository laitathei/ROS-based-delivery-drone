# Pose Estimation

## Pose index
![Index](src/testing_source/Pose_Index.jpeg)

## Different function of Different version 

`Recommend to use V2 for single person detection and V5 for multiple person detection`

| Version | Function |
|:---|:---|
|Pose_estim_V2| Single person pose detection|
|Pose_estim_V3| Multiple people pose Estimation(with YOLOv4)|
|Pose_estim_V3_1|Multiple people pose Estimation(with YOLOv4)|
|Pose_estim_V4| First testing with sprating YOLO and pose estimation|
|Pose_estim_V5| Multiple people pose Estimation(with YOLOv5)|
|Pose_estim_V6| Multiple people pose Estimation(with YOLOv5) - Testing version|

## Ros Topic list

|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Subscriber|/camera/color/image_raw|Image|
|Subscriber|/camera/aligned_depth_to_color/image_raw|Image|
|Subscriber|/camera/color/camera_info|CameraInfo|
|Publisher|/detected_human|Image|
|Publisher|/detected_gesture|String|

## Installation guide
```bash
# Please check your ubuntu version 
# All of the guide would be written for ubuntu 20.04

# If you did not install pip for python3 
sudo apt install python3-pip
# Mediapipe installation by pip
pip install mediapipe

# Install YOLOv5
git clone https://github.com/ultralytics/yolov5  # clone
cd yolov5
pip install -r requirements.txt  # install

```

## Operation guide 


```bash 
# Start testing in Gazebo environment with control gui
roslaunch px4 mavros_posix_sitl.launch
roslaunch control_gui control_gui.launch
roslaunch realsense2_camera rs_camera.launch align_depth:=true
python3 pose_estim_V5.py # Choose the version you want

```
### If you want to train your dataset
[Customize dataset train in YOLOv4](https://github.com/laitathei/ROS-based-delivery-drone/tree/main/Object%20detection)
[Customize dataset train in YOLOv5](https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data)

### Gazebo setup
[Gazebo setup](https://github.com/laitathei/ROS-based-delivery-drone/tree/main/Simulation)

### Control GUI setup
[GUI setup](https://github.com/laitathei/ROS-based-delivery-drone/tree/main/Control%20GUI)
