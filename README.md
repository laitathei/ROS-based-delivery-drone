# ROS-based-delivery-drone

## Hardware requirement
| Component                  | quantity |
|:--------------------------:|:---------:|
|Pixhawk mini                | 1         |
|Jetson Xavier NX            | 1         |
|D455 Realsense              | 1         |
|VESC                        | 4         |
|Gripper                     | 1         |
|LiPo (6s)                       | 1         |    

P.S. highly recommand to upgrade to Pixhawk 4 mini  or Pixhawk 4     

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

|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Publisher| /desired/input/box_array| BoundingBoxArray|
|Publisher| /detection_result/number_of_obstacle| Int32|
|Publisher| /detection_result/number_of_human| Int32|
|Publisher| /detection_result/number_of_injury| Int32|
|Publisher| /detected_human| Image|
|Publisher| /detected_human_gesture| String|
|Subscriber | /camera/color/image_raw| Image|
|Subscriber | /camera/aligned_depth_to_color/image_raw| Image|
|Subscriber | /camera/aligned_depth_to_color/camera_info| CameraInfo|
|Subscriber | /camera/color/camera_info| CameraInfo|
|Subscriber | /desired_path/local_trajectory| Path|

## vision navigation node

|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Publisher| /drone/input_postion/pose|PoseStamped|
|Publisher| /desired_path/position|MarkerArray|
|Publisher| /desired_path/validation_position|MarkerArray|
|Publisher| /desired_path/local_marker|MarkerArray|
|Publisher| /desired_path/local_trajectory|Path|
|Subscriber| /drone/nagvation/pos|PoseStamped|
|Subscriber| /auto_mode/status|BoolStamped|
|Subscriber| /extract_indices/output|PointCloud2|
|Subscriber| /detection_result/number_of_obstacle|Int32|
|Subscriber| /detection_result/number_of_human|Int32|
|Subscriber| /detection_result/number_of_injury|Int32|

## drone control node
[Detail Refer to Repo V3.0](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node)
### Onboard dron_control_node
|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Publisher| /drone/nagvation/pos| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Publisher| /auto_mode/status| [BoolStamped](http://docs.ros.org/en/indigo/api/jsk_recognition_msgs/html/msg/BoolStamped.html)|
|Publisher| /mavros/setpoint_position/local| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Publisher| /drone/current/control_status| Custom Message [GetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|
|Publisher| /servo/angle| [AllServoAngle](https://github.com/Drone-FYP2021-PolyU-EIE/ROS_CircuitPython_ServoKit)|
|Subscriber| /drone/input_posistion/pose| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Subscriber| /mavros/local_position/pose| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Subscriber| /mavros/state| [State](http://docs.ros.org/en/melodic/api/mavros_msgs/html/msg/State.html)|
|Subscriber| /drone/set/control_status| Custom Message [SetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|
### Offboard offboard/dron_control_node
|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Subscriber| /drone/current/control_status| Custom Message [GetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|
|Publisher| /drone/set/control_status| Custom Message [SetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|

## servo node
[Detail Refer to Repo V1.5](https://github.com/Drone-FYP2021-PolyU-EIE/ROS_CircuitPython_ServoKit)
|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Subscriber| /servo/angle| Custom Message [AllServoAngle](https://github.com/Drone-FYP2021-PolyU-EIE/ROS_CircuitPython_ServoKit#allservoangle)|
##  control GUI node


### Ros Topic list


|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Subscriber|/mavros/state|State|
|Subscriber|/mavros/local_position/pose|PoseStamped|
|Subscriber|/detected_human|Image|
|Subscriber|/detection_result/image|Image|
|Subscriber|/detected_human_gesture|String|
|Subscriber|/detected_human_pos|String|
|Publisher|/mavros/setpoint_position/local|PoseStamped|
|Publisher|/servo/angle|AllServoAngle|
|Publisher|/auto_mode/status|Bool|
