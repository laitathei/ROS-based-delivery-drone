# ROS-based-delivery-drone

## Hardware requirement
| Component                  | quantity |
|:--------------------------:|:---------:|
|Pixhawk mini                | 1         |
|Jetson Xavier NX            | 1         |
|D455 Realsense              | 1         |
|VESC                        | 4         |
|Gripper                     | 1         |
|LiPo                        | 1         |

## Software requirement
| Name                  | Remark          |
|:---------------------:|:------:|
|ROS melodic            | 1               |
|Ubuntu 18.04           | 1               |
|Gazebo 9               | Simulation      |
|Jetpack 4.5.1          | Nvidia package  |
|Darknet                | Model Training  |
|ONNX                   | Convert Model   |
|TensorRT               | Increase speed  |
|Google Collaboratory   | Train model     |

## ROS Network
![image](https://github.com/laitathei/ROS-based-delivery-drone/blob/main/figure/ros_network.jpg)

## detection node
* Publisher
     * `/desired/input/box_array`, `BoundingBoxArray`
     * `/detection_result/number_of_obstacle`, `Int32`
     * `/detection_result/number_of_human`, `Int32`
     * `/detection_result/number_of_injury`, `Int32`
     * `/detected_human`, `Image`
     * `/detected_human_gesture`, `String`

* Subscriber
     * `/camera/color/image_raw`, `Image`
     * `/camera/aligned_depth_to_color/image_raw`, `Image`
     * `/camera/aligned_depth_to_color/camera_info`, `CameraInfo`
     * `/camera/color/camera_info`, `CameraInfo`
     * `/desired_path/local_trajectory`, `Path`

## vision navigation node
* Publisher
     * `/drone/input_postion/pose`, `PoseStamped`
     * `/desired_path/position`, `MarkerArray`
     * `/desired_path/validation_position`, `MarkerArray`
     * `/desired_path/local_marker`, `MarkerArray`
     * `/desired_path/local_trajectory`, `Path`

* Subscriber
     * `/drone/nagvation/pos`, `PoseStamped`
     * `/auto_mode/status`, `BoolStamped`
     * `/extract_indices/output`, `PointCloud2`
     * `/detection_result/number_of_obstacle`, `Int32`
     * `/detection_result/number_of_human`, `Int32`
     * `/detection_result/number_of_injury`, `Int32`

## drone control node


## servo node


## pose estimation node
### Pose index
![Index](https://github.com/laitathei/ROS-based-delivery-drone/blob/main/figure/Pose_Index.jpeg)

### Different function of Different version 

| Version | Function |
|:---|:---|
|Pose_estim_V2| Single person pose detection|
|Pose_estim_V3| Multiple people pose Estimation(with YOLOv4)|
|Pose_estim_V3_1|Multiple people pose Estimation(with YOLOv4)|
|Pose_estim_V4| First testing with sprating YOLO and pose estimation|
|Pose_estim_V5| Multiple people pose Estimation(with YOLOv5)|


### Ros Topic list


|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Subscriber|/camera/color/image_raw|Image|
|Subscriber|/camera/aligned_depth_to_color/image_raw|Image|
|Subscriber|/camera/color/camera_info|CameraInfo|
|Publisher|/detected_human|Image|
|Publisher|/detected_gesture|String|