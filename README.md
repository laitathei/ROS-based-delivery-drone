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
