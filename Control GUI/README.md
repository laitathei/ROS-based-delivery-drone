# Control GUI

## Pose 
|No. of the Pose|State|
|:---:|:---:|
|1 and 6|Forward|
|2 and 3|Turn Left|
|4 and 7|Turn Right|
|5 and 8|Backward|

[Pose Index](https://github.com/laitathei/ROS-based-delivery-drone/tree/main/PoseV2)

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

## Installation guide
```bash
# If you did not install pip for python3 
sudo apt install python3-pip
# Install tkinter
sudo apt-get install -y python-tk python3-pil python3-pil.imagetk

```

## Operation guide 
```bash 
# Start testing in Gazebo environment
roslaunch px4 mavros_posix_sitl.launch
python3 control_gui.py

```

### Gazebo setup
[Gazebo setup](https://github.com/laitathei/ROS-based-delivery-drone/tree/main/Simulation)
