### Technic implemented

### 1. Locating obstacle centroid (K-means clustering)
![image](https://github.com/laitathei/ROS-based-delivery-drone/blob/main/figure/Locating_obstacle_centroid.jpg)

### 2. Finding out the nearest obstacle (Nearest neighbor search)
![image](https://github.com/laitathei/ROS-based-delivery-drone/blob/main/figure/Nearest_obstacle.jpg)

### 3. Generating the trajectory (Cubic B-spline interpolation approximation)
![image](https://github.com/laitathei/ROS-based-delivery-drone/blob/main/figure/Cubic_B-spline.jpg)

### 4. Following the trajectory
No control algorithm implemented, just PX4 default controller

### 5. Deciding the path (Nearest neighbor search)
![image](https://github.com/laitathei/ROS-based-delivery-drone/blob/main/figure/Collision_checking.jpg)
