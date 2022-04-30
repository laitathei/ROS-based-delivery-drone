#!/usr/bin/env python  
from email import message
from tokenize import String
import roslib
roslib.load_manifest('offboard_test')
import cmath
import math
import numpy
import scipy
import scipy.signal
import argparse
import rospy
import threading
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import  PoseStamped, Quaternion, Point
from std_msgs.msg import Int32,Header,Bool
from visualization_msgs.msg import *
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from gazebo_msgs.msg import *
from nav_msgs.msg import *
from scipy.interpolate import *
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from jsk_recognition_msgs.msg import BoolStamped
import matplotlib.pyplot as plt
import itertools
import pcl
import skfmm
import time
import random
import message_filters
from scipy.spatial import distance
import scipy.interpolate as scipy_interpolate


service_timeout=5.0
camera_frame_name="D435i::camera_depth_frame"
world_frame_name="Gazebo"
base_link_name="base_link"

# k means parameters
MAX_LOOP = 10
DCOST_TH = 0.1

class Clusters:

    def __init__(self, x, y, n_label):
        self.x = x
        self.y = y
        self.n_data = len(self.x)
        self.n_label = n_label
        self.labels = [random.randint(0, n_label - 1)
                       for _ in range(self.n_data)]
        self.center_x = [0.0 for _ in range(n_label)]
        self.center_y = [0.0 for _ in range(n_label)]

    def plot_cluster(self):
        cluster_x_list=[]
        cluster_y_list=[]
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            cluster_x_list.append(x)
            cluster_y_list.append(y)
        return cluster_x_list, cluster_y_list

    def calc_centroid(self,height):
        cluster_dict=dict()
        cluster_euclidean_distance_list=[]

        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            n_data = len(x)
            self.center_x[label] = sum(x) / n_data
            self.center_y[label] = sum(y) / n_data
            for i in range (n_data):
                cluster_euclidean_distance_list.append(((x[i]-self.center_x[label])**2+(y[i]-self.center_y[label])**2)**0.5)
                max_cluster_euclidean_distance_list=max(cluster_euclidean_distance_list)
            cluster_dict["cluster_"+str(label)]=(self.center_x[label],self.center_y[label],height,max_cluster_euclidean_distance_list)

        return cluster_dict

    def update_clusters(self):
        cost = 0.0

        for ip in range(self.n_data):
            px = self.x[ip]
            py = self.y[ip]

            dx = [icx - px for icx in self.center_x]
            dy = [icy - py for icy in self.center_y]

            dist_list = [math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
            min_dist = min(dist_list)
            min_id = dist_list.index(min_dist)
            self.labels[ip] = min_id
            cost += min_dist

        return cost

    def _get_labeled_x_y(self, target_label):
        x = [self.x[i] for i, label in enumerate(self.labels) if label == target_label]
        y = [self.y[i] for i, label in enumerate(self.labels) if label == target_label]
        return x, y

class moving():
    def __init__(self):
        # start ros node
        rospy.init_node('PX4_AuotFLy')
        self.finish_ready_position = False
        self.update_new_path = False
        self.use_the_second_part = False
        self.shift = False
        self.global_path_at_beginning = False
        self.auto_mode = False

        self.pointcloud_x_list = []
        self.pointcloud_y_list = []
        self.pointcloud_z_list = []
        self.past_passthrough_height=[]
        self.past_cluster_center_list=[]
        self.past_nearest_point_circle=[]
        self.past_local_path_x_leftside=[]
        self.past_local_path_y_leftside=[]
        self.past_local_path_x_rightside=[]
        self.past_local_path_y_rightside=[]
        self.past_concat_x=[]
        self.past_concat_y=[]
        self.past_concat_z=[]
        self.past_local_path_x_after_Bspline=[]
        self.past_local_path_y_after_Bspline=[]
        self.past_local_path_z_after_Bspline=[]
        self.past_global_x_trajectory=[]
        self.past_global_y_trajectory=[]
        self.past_global_z_trajectory=[]
        
        # prepare the publisher and subscriber
        #self.position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.pass_position_to_gui_pub = rospy.Publisher("/drone/input_postion/pose", PoseStamped, queue_size=1)
        self.dron_nagvation_pose_sub = message_filters.Subscriber("/drone/nagvation/pos", PoseStamped)          ## this message filter
        self.auto_mode_status_sub = message_filters.Subscriber("/auto_mode/status", BoolStamped)                                 ## this message filter
        time_synchronizer = message_filters.TimeSynchronizer([self.dron_nagvation_pose_sub, self.auto_mode_status_sub], queue_size=1)
        time_synchronizer.registerCallback(self.time_synchronizer_callback)
        #self.local_position_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped, self.callback_local_position)

        self.point_cloud2_sub  = rospy.Subscriber("/extract_indices/output",PointCloud2,self.callback_pointcloud)
        self.number_of_obstacle_sub  = rospy.Subscriber("/detection_result/number_of_obstacle", Int32, self.callback_number_of_obstacle)
        self.number_of_human_sub  = rospy.Subscriber("/detection_result/number_of_human", Int32, self.callback_number_of_human)
        self.number_of_injury_sub  = rospy.Subscriber("/detection_result/number_of_injury", Int32, self.callback_number_of_injury)

        self.marker_vis = rospy.Publisher("/desired_path/position",MarkerArray,queue_size=1)
        self.validation_marker_vis = rospy.Publisher("/desired_path/validation_position",MarkerArray,queue_size=1)

        self.local_marker_vis = rospy.Publisher("/desired_path/local_marker",MarkerArray,queue_size=1)
        self.local_trajectory_vis = rospy.Publisher("/desired_path/local_trajectory",Path,queue_size=1)

        # Trajectory config
        self.local_marker_array=MarkerArray()

        # TF config
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.transform = tf2_geometry_msgs.PointStamped()

        # customize parameter
        self.pi = 3.141592654
        self.drone_width_for_the_obstacle = 0.5
        self.drone_height_for_the_obstacle = 0.25
        self.updated_path_time = 0
        self.desired_position_x = 0.0
        self.desired_position_y = 0.0
        self.desired_position_z = 0.0
        self.local_position_x = 0.0
        self.local_position_y = 0.0
        self.local_position_z = 0.0
        self.number_of_obstacle = 0
        self.number_of_human = 0
        self.number_of_injury = 0

        self.target_position_x = 0
        self.target_position_y = 0
        self.target_position_z = 0
        self.target_orientation_x = 0
        self.target_orientation_y = 0
        self.target_orientation_z = 0
        self.target_orientation_w = 1
        
        self.past_target_position_x = 0
        self.past_target_position_y = 0
        self.past_target_position_z = 0

        self.hover_in_goal_position_x = 0
        self.hover_in_goal_position_y = 0
        self.hover_in_goal_position_z = 0
        self.hover_in_goal_quatrernion_x = 0
        self.hover_in_goal_quatrernion_y = 0
        self.hover_in_goal_quatrernion_z = 0
        self.hover_in_goal_quatrernion_w = 1

        self.display_point = int(abs(self.target_position_x)*abs(self.target_position_y))
        if self.display_point<100:
            self.display_point=100
        print("success init!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    def callback_local_position(self):
        for i in range(len(self.model.name)):
            if self.model.name[i] == 'iris':
                iris = ModelState()
                iris.pose = self.model.pose[i]
                self.global_x_trajectory = numpy.linspace(self.local_position_x,self.target_position_x,self.display_point)
                self.global_y_trajectory = numpy.linspace(self.local_position_y,self.target_position_y,self.display_point)
                self.global_z_trajectory = numpy.full((1, len(self.global_x_trajectory)), self.target_position_z)
                self.local_position_x = round(iris.pose.position.x,3)
                self.local_position_y = round(iris.pose.position.y,3)
                self.local_position_z = round(iris.pose.position.z,3)
                self.local_orientation_x = round(iris.pose.orientation.x,2)
                self.local_orientation_y = round(iris.pose.orientation.y,2)
                self.local_orientation_z = round(iris.pose.orientation.z,2)
                self.local_orientation_w = round(iris.pose.orientation.w,2)

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = world_frame_name
        t.child_frame_id = base_link_name
        t.transform.translation.x = self.local_position_x
        t.transform.translation.y = self.local_position_y
        t.transform.translation.z = self.local_position_z
        t.transform.rotation.x = iris.pose.orientation.x
        t.transform.rotation.y = iris.pose.orientation.y
        t.transform.rotation.z = iris.pose.orientation.z
        t.transform.rotation.w = iris.pose.orientation.w
        br.sendTransform(t)

        #orientation_quaternion_list = [self.local_orientation_x, self.local_orientation_y, self.local_orientation_z, self.local_orientation_w]
        #local_position_quaternion = euler_from_quaternion (orientation_quaternion_list)
        #print("original position in gazebo: {}".format(self.local_position_x)+" "+str(self.local_position_y)+" "+str(self.local_position_z))
        #print("original quaternion in gazebo: {}".format(self.local_orientation_x)+" "+str(self.local_orientation_y)+" "+str(self.local_orientation_z)+" "+str(self.local_orientation_w))
        #print("euler to quaternion in gazebo: {}".format(str(round(local_position_quaternion[0],3))+" "+str(round(local_position_quaternion[1],3))+" "+str(round(local_position_quaternion[2],3))))

    def time_synchronizer_callback(self,dron_nagvation_pose, auto_mode_status):
        self.target_position_x = round(dron_nagvation_pose.pose.position.x,3)
        self.target_position_y = round(dron_nagvation_pose.pose.position.y,3)
        self.target_position_z = round(dron_nagvation_pose.pose.position.z,3)
        self.display_point = int(abs(self.target_position_x)*abs(self.target_position_y))
        if self.display_point<100:
            self.display_point=100
        self.auto_mode = auto_mode_status.data



    def callback_number_of_obstacle(self,data):
        self.number_of_obstacle = int(data.data)
    def callback_number_of_human(self,data):
        self.number_of_human = int(data.data) 
    def callback_number_of_injury(self,data):
        self.number_of_injury = int(data.data) 
    def callback_pointcloud(self, data):
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(data,field_names=("x","y","z"), skip_nans=True)
        for p in gen:
            self.pointcloud_x_list.append(p[0])
            self.pointcloud_y_list.append(p[1])
            self.pointcloud_z_list.append(p[2])

    def kmeans_clustering(self,rx, ry, nc):
        clusters = Clusters(rx, ry, nc)
        clusters.calc_centroid(self.target_position_z)
        pre_cost = float("inf")
        for loop in range(MAX_LOOP):
            cost = clusters.update_clusters()
            cluster_dict=clusters.calc_centroid(self.target_position_z)
            d_cost = abs(cost - pre_cost)
            if d_cost < DCOST_TH:
                break
            pre_cost = cost
        return clusters, cluster_dict

    def interpolate_b_spline_path(self,x: list, y: list, n_path_points: int, degree: int = 3) -> tuple:
        # interpolate_b_spline_path
        ipl_t = numpy.linspace(0.0, len(x)-1, len(x))
        spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
        spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)
        travel = numpy.linspace(0.0, len(x) - 1, n_path_points)
        return spl_i_x(travel), spl_i_y(travel)

        # approximate_b_spline_path
        # t = range(len(x))
        # x_tup = scipy_interpolate.splrep(t, x, k=degree)
        # y_tup = scipy_interpolate.splrep(t, y, k=degree)
        # x_list = list(x_tup)
        # x_list[1] = x + [0.0, 0.0, 0.0, 0.0]
        # y_list = list(y_tup)
        # y_list[1] = y + [0.0, 0.0, 0.0, 0.0]
        # ipl_t = numpy.linspace(0.0, len(x) - 1, n_path_points)
        # rx = scipy_interpolate.splev(ipl_t, x_list)
        # ry = scipy_interpolate.splev(ipl_t, y_list)
        # return rx, ry

    def store_path_planner_result(self,passthrough_height,cluster_center_list,nearest_point_circle,local_path_x_leftside,local_path_y_leftside,local_path_x_rightside,local_path_y_rightside):
        if passthrough_height is not None:
            if len(passthrough_height)!=0:
                if self.update_new_path==False:
                    self.past_passthrough_height = list(self.past_passthrough_height)
                    self.past_cluster_center_list = list(self.past_cluster_center_list)
                    self.past_nearest_point_circle = list(self.past_nearest_point_circle)
                    self.past_local_path_x_leftside = list(self.past_local_path_x_leftside)
                    self.past_local_path_y_leftside = list(self.past_local_path_y_leftside)
                    self.past_local_path_x_rightside = list(self.past_local_path_x_rightside)
                    self.past_local_path_y_rightside = list(self.past_local_path_y_rightside)
                    del self.past_passthrough_height[:]
                    del self.past_cluster_center_list[:]
                    del self.past_nearest_point_circle[:]
                    del self.past_local_path_x_leftside[:]
                    del self.past_local_path_y_leftside[:]
                    del self.past_local_path_x_rightside[:]
                    del self.past_local_path_y_rightside[:]

                    self.past_passthrough_height = passthrough_height
                    self.past_cluster_center_list = cluster_center_list
                    self.past_nearest_point_circle = nearest_point_circle
                    self.past_local_path_x_leftside = local_path_x_leftside
                    self.past_local_path_y_leftside = local_path_y_leftside
                    self.past_local_path_x_rightside = local_path_x_rightside
                    self.past_local_path_y_rightside = local_path_y_rightside
                    print("store path planner result")
                elif self.update_new_path==True:
                    print("output path planner result when update new path is True")
                    return self.past_passthrough_height,self.past_cluster_center_list,self.past_nearest_point_circle,self.past_local_path_x_leftside,self.past_local_path_y_leftside,self.past_local_path_x_rightside,self.past_local_path_y_rightside
            elif len(passthrough_height)==0:
                print("output path planner result when no obstacle pointcloud")
                return self.past_passthrough_height,self.past_cluster_center_list,self.past_nearest_point_circle,self.past_local_path_x_leftside,self.past_local_path_y_leftside,self.past_local_path_x_rightside,self.past_local_path_y_rightside
        else:
            print("just want to get output path planner result")
            return self.past_passthrough_height,self.past_cluster_center_list,self.past_nearest_point_circle,self.past_local_path_x_leftside,self.past_local_path_y_leftside,self.past_local_path_x_rightside,self.past_local_path_y_rightside

    def shortest_distance_from_point_to_line(self,start,end,point):
        d = numpy.linalg.norm(numpy.cross(end-start, start-point))/numpy.linalg.norm(end-start)
        return d

    def path_planner(self):
        rospy.wait_for_message('/gazebo/model_states', ModelStates)
        rospy.wait_for_message('/extract_indices/output', PointCloud2)
        self.callback_local_position()
        ripple_filter = pcl.PointCloud()
        pointcloud = numpy.array((self.pointcloud_x_list,self.pointcloud_y_list,self.pointcloud_z_list),dtype=numpy.float32)
        pointcloud = pointcloud.T
        ripple_filter.from_array(pointcloud)
        if pointcloud.size!=0:
            passthrough = ripple_filter.make_passthrough_filter()
            passthrough.set_filter_field_name("z")
            passthrough.set_filter_limits(self.target_position_z-0.5, self.target_position_z+0.5)
            passthrough_height = passthrough.filter()
            if passthrough_height.size!=0:
                passthrough_height = numpy.array(passthrough_height).T
                #passthrough_height = numpy.delete(passthrough_height, 2, 0)
                clusters, cluster_dict = self.kmeans_clustering(passthrough_height[0], passthrough_height[1], self.number_of_obstacle)
                cluster_centroid_x_list=[]
                cluster_centroid_y_list=[]
                cluster_centroid_z_list=[]
                safety_region=[]
                tmp_list=[]
                cluster_center_list=[]
                nearest_point_circle=[]

                for i in range(len(cluster_dict)):
                    tmp_list=cluster_dict.get(("cluster_"+str(i)),0)
                    if tmp_list!=0:
                        cluster_centroid_x_list.append(tmp_list[0])
                        cluster_centroid_y_list.append(tmp_list[1])
                        cluster_centroid_z_list.append(tmp_list[2])
                        safety_region.append(tmp_list[3])

                try:
                    cluster_centroid_x_list.remove(0)
                    cluster_centroid_y_list.remove(0)
                    cluster_centroid_z_list.remove(0)
                    safety_region.remove(0)
                    print("cluster missing!!!!!!!!!!!!!!!!")
                except:
                    print("no cluster missing")

                cluster_centroid_x_list = [round(num, 3) for num in cluster_centroid_x_list]
                cluster_centroid_y_list = [round(num, 3) for num in cluster_centroid_y_list]
                
                tmp_cluster_center_list = numpy.array((cluster_centroid_x_list,cluster_centroid_y_list,cluster_centroid_z_list),dtype=numpy.float32)
                for i in range(len(cluster_centroid_x_list)):
                    cluster_center_list.append([cluster_centroid_x_list[i],cluster_centroid_y_list[i],cluster_centroid_z_list[i]])


                ripple_filter.from_array(tmp_cluster_center_list.T)
                resolution = 0.2
                octree = ripple_filter.make_octreeSearch(resolution)
                octree.add_points_from_input_cloud()
                searchPoint = pcl.PointCloud()
                searchPoints = numpy.zeros((1, 3), dtype=numpy.float32)
                searchPoints[0][0] = self.local_position_x
                searchPoints[0][1] = self.local_position_y
                searchPoints[0][2] = self.local_position_z
                searchPoint.from_array(searchPoints)
                [ind, sqdist_nearest_point] = octree.nearest_k_search_for_cloud(searchPoint, 1)
                nearest_point=numpy.array((ripple_filter[ind[0][0]][0],ripple_filter[ind[0][0]][1],ripple_filter[ind[0][0]][2]))
                #euclidean_distance=sqdist_nearest_point[0][0]

                self.desired_position_x = ripple_filter[ind[0][0]][0]
                self.desired_position_y = ripple_filter[ind[0][0]][1]
                self.desired_position_z = ripple_filter[ind[0][0]][2]

                path_pointcloud = pcl.PointCloud()
                path_point_list = numpy.array((list(self.global_x_trajectory),list(self.global_y_trajectory),list(self.global_z_trajectory[0])),dtype=numpy.float32)
                path_point_list = path_point_list.T
                passthrough_height_choose_gloabl_or_not_at_beginning = passthrough_height.T
                path_pointcloud.from_array(passthrough_height_choose_gloabl_or_not_at_beginning)
                resolution = 0.2
                octree = path_pointcloud.make_octreeSearch(resolution)
                octree.add_points_from_input_cloud()
                path_point = pcl.PointCloud()
                path_points = numpy.zeros((1, 3), dtype=numpy.float32)
                global_path=0
                local_path=0
                for i in range(len(list(self.global_x_trajectory))):
                    path_points[0][0] = path_point_list[i][0]
                    path_points[0][1] = path_point_list[i][1]
                    path_points[0][2] = path_point_list[i][2]
                    path_point.from_array(path_points)
                    [ind, sqdist] = octree.nearest_k_search_for_cloud(path_point, 1)
                    euclidean_distance_at_the_beginning=sqdist[0][0]
                    if (euclidean_distance_at_the_beginning**2-self.target_position_z**2-self.drone_height_for_the_obstacle**2)**0.5>(self.drone_width_for_the_obstacle/2):
                        global_path = global_path+1
                    else:
                        local_path = local_path+1
                print("global_path: {}".format(global_path))
                print("local_path: {}".format(local_path))
                if local_path<=self.display_point*0.2:
                    self.update_new_path = True
                    self.use_the_second_part = False
                    passthrough_height_choose_gloabl_or_not_at_beginning = numpy.array(passthrough_height_choose_gloabl_or_not_at_beginning).T
                    nearest_point_circle = []
                    local_path_x_leftside = []
                    local_path_y_leftside = []
                    local_path_x_rightside = []
                    local_path_y_rightside = []
                    # find out the desired position under which cluster
                    desired_position_cluster = cluster_centroid_x_list.index(round(self.desired_position_x,3))
                    for i in range(6):
                        nearest_point_circle.append([(nearest_point[0]+(safety_region[desired_position_cluster]+self.drone_width_for_the_obstacle)*math.cos(math.radians((60)*i))), (nearest_point[1]+(safety_region[desired_position_cluster]+self.drone_width_for_the_obstacle)*math.sin(math.radians((60)*i))), self.desired_position_z])
                    nearest_point_circle.sort()
                    nearest_point_circle = numpy.array(nearest_point_circle)
                    #print(nearest_point_circle)
                    tmp_y=self.target_position_y-self.local_position_y
                    tmp_x=self.target_position_x-self.local_position_x
                    if (tmp_y==0)or(tmp_x==0):
                        yaw_radian = 0
                    else:
                        yaw_radian = math.atan2(tmp_y,tmp_x)
                    #print(yaw_radian)
                    if 0<=yaw_radian<=0.785398163:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[0][0], nearest_point_circle[2][0],nearest_point_circle[4][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[0][1], nearest_point_circle[2][1],nearest_point_circle[4][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[1][0], nearest_point_circle[3][0],nearest_point_circle[5][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[1][1], nearest_point_circle[3][1],nearest_point_circle[5][1]]
                        print("0 to 45")
                    elif 0.785398163<=yaw_radian<=1.57079633:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[1][0], nearest_point_circle[0][0],nearest_point_circle[2][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[1][1], nearest_point_circle[0][1],nearest_point_circle[2][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[1][0], nearest_point_circle[3][0],nearest_point_circle[5][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[1][1], nearest_point_circle[3][1],nearest_point_circle[5][1]]
                        print("45 to 90")
                    elif 1.57079633<=yaw_radian<=2.35619449:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[3][0], nearest_point_circle[1][0],nearest_point_circle[0][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[3][1], nearest_point_circle[1][1],nearest_point_circle[0][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[3][0], nearest_point_circle[5][0],nearest_point_circle[4][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[3][1], nearest_point_circle[5][1],nearest_point_circle[4][1]]
                        print("90 to 135")
                    elif 2.35619449<=yaw_radian<=3.14159265:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[3][0], nearest_point_circle[1][0],nearest_point_circle[0][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[3][1], nearest_point_circle[1][1],nearest_point_circle[0][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[5][0], nearest_point_circle[4][0],nearest_point_circle[2][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[5][1], nearest_point_circle[4][1],nearest_point_circle[2][1]]
                        print("135 to 180")
                    elif -3.14159265<=yaw_radian<=-2.35619449:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[5][0], nearest_point_circle[3][0],nearest_point_circle[1][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[5][1], nearest_point_circle[3][1],nearest_point_circle[1][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[4][0], nearest_point_circle[2][0],nearest_point_circle[0][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[4][1], nearest_point_circle[2][1],nearest_point_circle[0][1]]
                        print("180 to 225")
                    elif -2.35619449<=yaw_radian<=-1.57079633:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[4][0], nearest_point_circle[5][0],nearest_point_circle[3][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[4][1], nearest_point_circle[5][1],nearest_point_circle[3][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[4][0], nearest_point_circle[2][0],nearest_point_circle[0][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[4][1], nearest_point_circle[2][1],nearest_point_circle[0][1]]
                        print("225 to 270")
                    elif -1.57079633<=yaw_radian<=-0.785398163:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[2][0], nearest_point_circle[4][0],nearest_point_circle[5][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[2][1], nearest_point_circle[4][1],nearest_point_circle[5][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[2][0], nearest_point_circle[0][0],nearest_point_circle[1][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[2][1], nearest_point_circle[0][1],nearest_point_circle[1][1]]
                        print("270 to 315")
                    elif -0.785398163<=yaw_radian<=-0:
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[2][0], nearest_point_circle[4][0],nearest_point_circle[5][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[2][1], nearest_point_circle[4][1],nearest_point_circle[5][1]]
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[0][0], nearest_point_circle[1][0],nearest_point_circle[3][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[0][1], nearest_point_circle[1][1],nearest_point_circle[3][1]]
                        print("315 to 360")
                    if self.updated_path_time==0:
                        self.global_path_at_beginning = True
                        self.store_path_planner_result(passthrough_height,cluster_center_list,nearest_point_circle,local_path_x_leftside,local_path_y_leftside,local_path_x_rightside,local_path_y_rightside)
                        print("Choose to have global path at the beginning as the obstacle do not block the path")
                else:
                    # find out the desired position under which cluster
                    desired_position_cluster = cluster_centroid_x_list.index(round(self.desired_position_x,3))

                    for i in range(6):
                        nearest_point_circle.append([(nearest_point[0]+(safety_region[desired_position_cluster]+self.drone_width_for_the_obstacle)*math.cos(math.radians((60)*i))), (nearest_point[1]+(safety_region[desired_position_cluster]+self.drone_width_for_the_obstacle)*math.sin(math.radians((60)*i))), self.desired_position_z])
                    nearest_point_circle.sort()
                    nearest_point_circle = numpy.array(nearest_point_circle)
                    #print(nearest_point_circle)
                    tmp_y=self.target_position_y-self.local_position_y
                    tmp_x=self.target_position_x-self.local_position_x
                    if (tmp_y==0)or(tmp_x==0):
                        yaw_radian = 0
                    else:
                        yaw_radian = math.atan2(tmp_y,tmp_x)
                    #print(yaw_radian)
                    if 0<=yaw_radian<=0.785398163:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[0][0], nearest_point_circle[2][0],nearest_point_circle[4][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[0][1], nearest_point_circle[2][1],nearest_point_circle[4][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[1][0], nearest_point_circle[3][0],nearest_point_circle[5][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[1][1], nearest_point_circle[3][1],nearest_point_circle[5][1]]
                        print("0 to 45")
                    elif 0.785398163<=yaw_radian<=1.57079633:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[1][0], nearest_point_circle[0][0],nearest_point_circle[2][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[1][1], nearest_point_circle[0][1],nearest_point_circle[2][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[1][0], nearest_point_circle[3][0],nearest_point_circle[5][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[1][1], nearest_point_circle[3][1],nearest_point_circle[5][1]]
                        print("45 to 90")
                    elif 1.57079633<=yaw_radian<=2.35619449:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[3][0], nearest_point_circle[1][0],nearest_point_circle[0][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[3][1], nearest_point_circle[1][1],nearest_point_circle[0][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[3][0], nearest_point_circle[5][0],nearest_point_circle[4][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[3][1], nearest_point_circle[5][1],nearest_point_circle[4][1]]
                        print("90 to 135")
                    elif 2.35619449<=yaw_radian<=3.14159265:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[3][0], nearest_point_circle[1][0],nearest_point_circle[0][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[3][1], nearest_point_circle[1][1],nearest_point_circle[0][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[5][0], nearest_point_circle[4][0],nearest_point_circle[2][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[5][1], nearest_point_circle[4][1],nearest_point_circle[2][1]]
                        print("135 to 180")
                    elif -3.14159265<=yaw_radian<=-2.35619449:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[5][0], nearest_point_circle[3][0],nearest_point_circle[1][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[5][1], nearest_point_circle[3][1],nearest_point_circle[1][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[4][0], nearest_point_circle[2][0],nearest_point_circle[0][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[4][1], nearest_point_circle[2][1],nearest_point_circle[0][1]]
                        print("180 to 225")
                    elif -2.35619449<=yaw_radian<=-1.57079633:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[4][0], nearest_point_circle[5][0],nearest_point_circle[3][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[4][1], nearest_point_circle[5][1],nearest_point_circle[3][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[4][0], nearest_point_circle[2][0],nearest_point_circle[0][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[4][1], nearest_point_circle[2][1],nearest_point_circle[0][1]]
                        print("225 to 270")
                    elif -1.57079633<=yaw_radian<=-0.785398163:
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[2][0], nearest_point_circle[4][0],nearest_point_circle[5][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[2][1], nearest_point_circle[4][1],nearest_point_circle[5][1]]
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[2][0], nearest_point_circle[0][0],nearest_point_circle[1][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[2][1], nearest_point_circle[0][1],nearest_point_circle[1][1]]
                        print("270 to 315")
                    elif -0.785398163<=yaw_radian<=-0:
                        local_path_x_rightside = [self.local_position_x, nearest_point_circle[2][0], nearest_point_circle[4][0],nearest_point_circle[5][0]]
                        local_path_y_rightside = [self.local_position_y, nearest_point_circle[2][1], nearest_point_circle[4][1],nearest_point_circle[5][1]]
                        local_path_x_leftside = [self.local_position_x, nearest_point_circle[0][0], nearest_point_circle[1][0],nearest_point_circle[3][0]]
                        local_path_y_leftside = [self.local_position_y, nearest_point_circle[0][1], nearest_point_circle[1][1],nearest_point_circle[3][1]]
                        print("315 to 360")
                    if self.updated_path_time==0:
                        self.store_path_planner_result(passthrough_height,cluster_center_list,nearest_point_circle,local_path_x_leftside,local_path_y_leftside,local_path_x_rightside,local_path_y_rightside)
                        print("Choose to have local path at the beginning as the obstacle block the path")
                if self.updated_path_time!=0:
                    concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory=self.store_local_marker_array_result(None,None,None,None,None,None,None,None,None)
                    path_pointcloud_not_beginning = pcl.PointCloud()
                    path_point_list = numpy.array((global_x_trajectory,global_y_trajectory,global_z_trajectory),dtype=numpy.float32)
                    path_point_list = path_point_list.T
                    passthrough_height = passthrough_height.T
                    path_pointcloud_not_beginning.from_array(passthrough_height)
                    resolution = 0.2
                    octree = path_pointcloud_not_beginning.make_octreeSearch(resolution)
                    octree.add_points_from_input_cloud()
                    path_point = pcl.PointCloud()
                    path_points = numpy.zeros((1, 3), dtype=numpy.float32)
                    update_path=0
                    not_update_path=0
                    for i in range(len(global_x_trajectory)):
                        path_points[0][0] = path_point_list[i][0]
                        path_points[0][1] = path_point_list[i][1]
                        path_points[0][2] = path_point_list[i][2]
                        path_point.from_array(path_points)
                        [ind, sqdist] = octree.nearest_k_search_for_cloud(path_point, 1)
                        euclidean_distance=sqdist[0][0]
                        if (euclidean_distance**2-self.target_position_z**2-self.drone_height_for_the_obstacle**2)**0.5>(self.drone_width_for_the_obstacle/2):
                            not_update_path = not_update_path+1
                        else:
                            update_path = update_path+1
                    print("not_update_path: {}".format(not_update_path))
                    print("update_path: {}".format(update_path))
                    rightside_point = numpy.array((local_path_x_rightside[3],local_path_y_rightside[3]),dtype=numpy.float32)
                    leftside_point = numpy.array((local_path_x_leftside[3],local_path_y_leftside[3]),dtype=numpy.float32)
                    start_point = numpy.array((self.local_position_x,self.local_position_y),dtype=numpy.float32)
                    end_point = numpy.array((self.target_position_x,self.target_position_y),dtype=numpy.float32)
                    distance_from_point_to_line_rightside = self.shortest_distance_from_point_to_line(start_point,end_point,rightside_point)
                    distance_from_point_to_line_leftside = self.shortest_distance_from_point_to_line(start_point,end_point,leftside_point)

                    if distance_from_point_to_line_rightside>distance_from_point_to_line_leftside:
                        self.shift = False
                        print("Leftside choosed")
                    else:
                        self.shift = True
                        print("Rightside choosed")
                    
                    if update_path>=self.display_point*0.2:
                        self.update_new_path = True
                        self.use_the_second_part = False
                        self.store_path_planner_result(passthrough_height,cluster_center_list,nearest_point_circle,local_path_x_leftside,local_path_y_leftside,local_path_x_rightside,local_path_y_rightside)
                        print("Choose to update the path")
                    else:
                        self.update_new_path = False
                        self.use_the_second_part = True
                        past_passthrough_height,past_cluster_center_list,past_nearest_point_circle,past_local_path_x_leftside,past_local_path_y_leftside,past_local_path_x_rightside,past_local_path_y_rightside=self.store_path_planner_result(None,None,None,None,None,None,None)
                        passthrough_height = past_passthrough_height
                        cluster_center_list = past_cluster_center_list
                        nearest_point_circle = past_nearest_point_circle
                        local_path_x_leftside = past_local_path_x_leftside
                        local_path_y_leftside = past_local_path_y_leftside
                        local_path_x_rightside = past_local_path_x_rightside
                        local_path_y_rightside = past_local_path_y_rightside

                        print("Choose to not update the path")
                        print(past_local_path_x_leftside)
                    print("Finish check the current remaining path is near to the obstacle or not in after updated path one time situation")

                self.updated_path_time = self.updated_path_time+1
                return passthrough_height,cluster_center_list,nearest_point_circle,local_path_x_leftside,local_path_y_leftside,local_path_x_rightside,local_path_y_rightside
            else:
                #print("No obstacle detected")
                past_passthrough_height,past_cluster_center_list,past_nearest_point_circle,past_local_path_x_leftside,past_local_path_y_leftside,past_local_path_x_rightside,past_local_path_y_rightside=self.store_path_planner_result(None,None,None,None,None,None,None)
                print("Choose to not update the path")
                print("Finish check the current remaining path is near to the obstacle or not in No obstacle detected situation")
                print(past_local_path_x_leftside)
                self.updated_path_time = self.updated_path_time+1
                return past_passthrough_height,past_cluster_center_list,past_nearest_point_circle,past_local_path_x_leftside,past_local_path_y_leftside,past_local_path_x_rightside,past_local_path_y_rightside
        else:
            #print("No obstacle pointcloud")
            #self.use_the_second_part = True
            if self.updated_path_time==0:
                past_passthrough_height,past_cluster_center_list,past_nearest_point_circle,past_local_path_x_leftside,past_local_path_y_leftside,past_local_path_x_rightside,past_local_path_y_rightside=self.store_path_planner_result(None,None,None,None,None,None,None)
                self.update_new_path = True
                self.use_the_second_part = False
                self.global_path_at_beginning = True
                self.updated_path_time = self.updated_path_time+1
                print("Global path")
            else:
                past_passthrough_height,past_cluster_center_list,past_nearest_point_circle,past_local_path_x_leftside,past_local_path_y_leftside,past_local_path_x_rightside,past_local_path_y_rightside=self.store_path_planner_result(None,None,None,None,None,None,None)
                self.update_new_path = False
                self.use_the_second_part = True
                print("Choose to not update the path")
                print("Finish check the current remaining path is near to the obstacle or not in No obstacle pointcloud situation")
                self.updated_path_time = self.updated_path_time+1
                print(past_local_path_x_leftside)
            return past_passthrough_height,past_cluster_center_list,past_nearest_point_circle,past_local_path_x_leftside,past_local_path_y_leftside,past_local_path_x_rightside,past_local_path_y_rightside
        
    # Create marker in rviz
    def create_marker(self,cluster_center_list):
        marker = Marker()
        marker_array = MarkerArray()
        for i in range(len(cluster_center_list)):
            marker.header.frame_id = world_frame_name
            marker.header.stamp = rospy.Time.now()+rospy.Duration(0.01*i)
            marker.ns = "nearest_position_with_kmean_cluster_centroid_marker"
            marker.action = marker.ADD
            marker.type = Marker.SPHERE_LIST
            marker.pose.orientation.w = 1.0
            point = Point(cluster_center_list[i][0],cluster_center_list[i][1],cluster_center_list[i][2])
            #print(point)
            marker.id = i
            marker.points.append(point)
            marker.scale.x = 0.1
            marker.scale.z = 0.1
            marker.scale.y = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.marker_vis.publish(marker_array)
    
    # Create validation marker in rviz
    def create_validation_marker(self,nearest_point_circle):
        validation_marker = Marker()
        validation_marker_array = MarkerArray()
        for i in range(len(nearest_point_circle)):
            #local marker config
            validation_marker.header.frame_id = world_frame_name
            validation_marker.header.stamp = rospy.Time.now()+rospy.Duration(0.01*i)
            validation_marker.ns = "path_validation_marker"
            validation_marker.action = validation_marker.ADD
            validation_marker.type = Marker.SPHERE_LIST
            validation_marker.pose.orientation.w = 1.0
            point = Point(nearest_point_circle[i][0],nearest_point_circle[i][1],nearest_point_circle[i][2])
            validation_marker.id = i
            validation_marker.points.append(point)
            validation_marker.scale.x = 0.1
            validation_marker.scale.z = 0.1
            validation_marker.scale.y = 0.1
            validation_marker.color.a = 1.0
            validation_marker.color.r = 0.0
            validation_marker.color.g = 1.0
            validation_marker.color.b = 0.0
            validation_marker_array.markers.append(validation_marker)
    
        self.validation_marker_vis.publish(validation_marker_array)
        

    # Create local marker in rviz
    def create_local_marker_array(self,passthrough_height,local_path_x_leftside, local_path_y_leftside,local_path_x_rightside,local_path_y_rightside):
        if self.global_path_at_beginning==True:
            local_marker = Marker()
            for i in range(len(self.global_x_trajectory)):
                #local marker config
                local_marker.header.frame_id = world_frame_name
                local_marker.header.stamp = rospy.Time.now()+rospy.Duration(0.01*i)
                local_marker.ns = "local_marker"
                local_marker.action = local_marker.ADD
                local_marker.type = Marker.SPHERE_LIST
                local_marker.pose.orientation.w = 1.0
                point = Point(self.global_x_trajectory[i],self.global_y_trajectory[i],self.global_z_trajectory[0][i])
                local_marker.id = i
                local_marker.points.append(point)
                local_marker.scale.x = 0.1
                local_marker.scale.z = 0.1
                local_marker.scale.y = 0.1
                local_marker.color.a = 1.0
                local_marker.color.r = 1.0
                local_marker.color.g = 0.0
                local_marker.color.b = 0.0
                self.local_marker_array.markers.append(local_marker)
            concat_x = []
            concat_y = []
            concat_z = numpy.full((1, self.display_point*2), self.target_position_z)
            local_path_x_after_Bspline = []
            local_path_y_after_Bspline = []
            local_path_z_after_Bspline = numpy.full((1, len(self.global_x_trajectory)), self.target_position_z)
            global_x_trajectory = numpy.linspace(self.local_position_x,self.target_position_x,self.display_point*2)
            global_y_trajectory = numpy.linspace(self.local_position_y,self.target_position_y,self.display_point*2)
            global_z_trajectory = numpy.full((1, len(global_x_trajectory)), self.target_position_z)
            self.store_local_marker_array_result(concat_x,concat_y,concat_z[0],local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline[0],global_x_trajectory,global_y_trajectory,global_z_trajectory[0])
            print("create global path marker array")
        else:
            if self.shift==False:
                local_marker = Marker()
                local_path_x_after_Bspline, local_path_y_after_Bspline = self.interpolate_b_spline_path(local_path_x_leftside, local_path_y_leftside,self.display_point)
                nearest_global_x_index = (numpy.abs(self.global_x_trajectory-local_path_x_after_Bspline[-1])).argmin()
                nearest_global_y_index = (numpy.abs(self.global_y_trajectory-local_path_y_after_Bspline[-1])).argmin()
                nearest_global_x=self.global_x_trajectory[nearest_global_x_index]
                nearest_global_y=self.global_y_trajectory[nearest_global_y_index]
                global_x_trajectory = numpy.linspace(nearest_global_x,self.target_position_x,self.display_point)
                global_y_trajectory = numpy.linspace(nearest_global_y,self.target_position_y,self.display_point)
                global_z_trajectory = numpy.full((1, len(global_x_trajectory)), self.target_position_z)
                local_path_x_after_Bspline=numpy.array(local_path_x_after_Bspline)
                local_path_y_after_Bspline=numpy.array(local_path_y_after_Bspline)
                local_path_z_after_Bspline = numpy.full((1, len(local_path_x_after_Bspline)), self.target_position_z)
                concat_x = numpy.concatenate([local_path_x_after_Bspline, global_x_trajectory])
                concat_y = numpy.concatenate([local_path_y_after_Bspline, global_y_trajectory])
                concat_z = numpy.full((1, len(concat_x)), self.target_position_z)

                for i in range(len(concat_x)):
                    #local marker config
                    local_marker.header.frame_id = world_frame_name
                    local_marker.header.stamp = rospy.Time.now()+rospy.Duration(0.01*i)
                    local_marker.ns = "local_marker"
                    local_marker.action = local_marker.ADD
                    local_marker.type = Marker.SPHERE_LIST
                    local_marker.pose.orientation.w = 1.0
                    point = Point(concat_x[i],concat_y[i],concat_z[0][i])
                    local_marker.id = i
                    local_marker.points.append(point)
                    local_marker.scale.x = 0.1
                    local_marker.scale.z = 0.1
                    local_marker.scale.y = 0.1
                    local_marker.color.a = 1.0
                    local_marker.color.r = 1.0
                    local_marker.color.g = 0.0
                    local_marker.color.b = 0.0
                    self.local_marker_array.markers.append(local_marker)

                passthrough_height=list(passthrough_height)
                x_begin = max(concat_x[0], passthrough_height[0][0])     # 3
                x_end = min(concat_x[-1], passthrough_height[0][-1])     # 8
                points1 = [t for t in zip(concat_x, concat_y) if x_begin<=t[0]<=x_end]
                points2 = [t for t in zip(passthrough_height[0], passthrough_height[1]) if x_begin<=t[0]<=x_end]
                idx = 0
                nrof_points = len(points1)
                if len(points2)>=len(points1):
                    while idx < nrof_points-1:
                        # Iterate over two line segments
                        y1_new = numpy.linspace(points1[idx][1], points1[idx+1][1], 1000)  # e.g., (6, 7) corresponding to (240, 50) in y1
                        y2_new = numpy.linspace(points2[idx][1], points2[idx+1][1], 1000)  # e.g., (6, 7) corresponding to (67, 88) in y2
                        tmp_idx = numpy.argwhere(numpy.isclose(y1_new, y2_new, atol=0.1)).reshape(-1)
                        if tmp_idx.size!=0:
                            self.shift = True
                            
                        idx += 1
                self.store_local_marker_array_result(concat_x,concat_y,concat_z[0],local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline[0],global_x_trajectory,global_y_trajectory,global_z_trajectory[0])

            if self.shift==True:
                local_marker = Marker()
                local_path_x_after_Bspline, local_path_y_after_Bspline = self.interpolate_b_spline_path(local_path_x_rightside, local_path_y_rightside,self.display_point)
                nearest_global_x_index = (numpy.abs(self.global_x_trajectory-local_path_x_after_Bspline[-1])).argmin()
                nearest_global_y_index = (numpy.abs(self.global_y_trajectory-local_path_y_after_Bspline[-1])).argmin()
                nearest_global_x=self.global_x_trajectory[nearest_global_x_index]
                nearest_global_y=self.global_y_trajectory[nearest_global_y_index]
                global_x_trajectory = numpy.linspace(nearest_global_x,self.target_position_x,self.display_point)
                global_y_trajectory = numpy.linspace(nearest_global_y,self.target_position_y,self.display_point)
                global_z_trajectory = numpy.full((1, len(global_x_trajectory)), self.target_position_z)
                local_path_x_after_Bspline=numpy.array(local_path_x_after_Bspline)
                local_path_y_after_Bspline=numpy.array(local_path_y_after_Bspline)
                local_path_z_after_Bspline = numpy.full((1, len(local_path_x_after_Bspline)), self.target_position_z)
                concat_x = numpy.concatenate([local_path_x_after_Bspline, global_x_trajectory])
                concat_y = numpy.concatenate([local_path_y_after_Bspline, global_y_trajectory])
                concat_z = numpy.full((1, len(concat_x)), self.target_position_z)

                for i in range(len(concat_x)):
                    #local marker config
                    local_marker.header.frame_id = world_frame_name
                    local_marker.header.stamp = rospy.Time.now()+rospy.Duration(0.01*i)
                    local_marker.ns = "local_marker"
                    local_marker.action = local_marker.ADD
                    local_marker.type = Marker.SPHERE_LIST
                    local_marker.pose.orientation.w = 1.0
                    point = Point(concat_x[i],concat_y[i],concat_z[0][i])
                    local_marker.id = i
                    local_marker.points.append(point)
                    local_marker.scale.x = 0.1
                    local_marker.scale.z = 0.1
                    local_marker.scale.y = 0.1
                    local_marker.color.a = 1.0
                    local_marker.color.r = 1.0
                    local_marker.color.g = 0.0
                    local_marker.color.b = 0.0
                    self.local_marker_array.markers.append(local_marker)

                passthrough_height=list(passthrough_height)
                x_begin = max(concat_x[0], passthrough_height[0][0])     # 3
                x_end = min(concat_x[-1], passthrough_height[0][-1])     # 8
                points1 = [t for t in zip(concat_x, concat_y) if x_begin<=t[0]<=x_end]
                points2 = [t for t in zip(passthrough_height[0], passthrough_height[1]) if x_begin<=t[0]<=x_end]
                idx = 0
                nrof_points = len(points1)
                if len(points2)>=len(points1):
                    while idx < nrof_points-1:
                        # Iterate over two line segments
                        y1_new = numpy.linspace(points1[idx][1], points1[idx+1][1], 1000)  # e.g., (6, 7) corresponding to (240, 50) in y1
                        y2_new = numpy.linspace(points2[idx][1], points2[idx+1][1], 1000)  # e.g., (6, 7) corresponding to (67, 88) in y2
                        tmp_idx = numpy.argwhere(numpy.isclose(y1_new, y2_new, atol=0.1)).reshape(-1)
                        idx += 1
                self.store_local_marker_array_result(concat_x,concat_y,concat_z[0],local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline[0],global_x_trajectory,global_y_trajectory,global_z_trajectory[0])
            print("create local path marker array")
            
        return concat_x,concat_y,concat_z[0],local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline[0],global_x_trajectory,global_y_trajectory,global_z_trajectory[0]


    def store_local_marker_array_result(self,concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory):
        if concat_x is not None:
            if (self.update_new_path==False)&(self.use_the_second_part==False):
                self.past_concat_x = list(self.past_concat_x)
                self.past_concat_y = list(self.past_concat_y)
                self.past_concat_z = list(self.past_concat_z)
                self.past_local_path_x_after_Bspline = list(self.past_local_path_x_after_Bspline)
                self.past_local_path_y_after_Bspline = list(self.past_local_path_y_after_Bspline)
                self.past_local_path_z_after_Bspline = list(self.past_local_path_z_after_Bspline)
                self.past_global_x_trajectory = list(self.past_global_x_trajectory)
                self.past_global_y_trajectory = list(self.past_global_y_trajectory)
                self.past_global_z_trajectory = list(self.past_global_z_trajectory)
                del self.past_concat_x[:]
                del self.past_concat_y[:]
                del self.past_concat_z[:]
                del self.past_local_path_x_after_Bspline[:]
                del self.past_local_path_y_after_Bspline[:]
                del self.past_local_path_z_after_Bspline[:]
                del self.past_global_x_trajectory[:]
                del self.past_global_y_trajectory[:]
                del self.past_global_z_trajectory[:]

                self.past_concat_x = concat_x
                self.past_concat_y = concat_y
                self.past_concat_z = concat_z
                self.past_local_path_x_after_Bspline = local_path_x_after_Bspline
                self.past_local_path_y_after_Bspline = local_path_y_after_Bspline
                self.past_local_path_z_after_Bspline = local_path_z_after_Bspline

                self.past_global_x_trajectory = global_x_trajectory
                self.past_global_y_trajectory = global_y_trajectory
                self.past_global_z_trajectory = global_z_trajectory
                print("store marker result!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif (self.update_new_path==False)&(self.use_the_second_part==True):
                self.past_concat_x = list(self.past_concat_x)
                self.past_concat_y = list(self.past_concat_y)
                self.past_concat_z = list(self.past_concat_z)
                self.past_local_path_x_after_Bspline = list(self.past_local_path_x_after_Bspline)
                self.past_local_path_y_after_Bspline = list(self.past_local_path_y_after_Bspline)
                self.past_local_path_z_after_Bspline = list(self.past_local_path_z_after_Bspline)
                self.past_global_x_trajectory = list(self.past_global_x_trajectory)
                self.past_global_y_trajectory = list(self.past_global_y_trajectory)
                self.past_global_z_trajectory = list(self.past_global_z_trajectory)
                del self.past_concat_x[:]
                del self.past_concat_y[:]
                del self.past_concat_z[:]
                del self.past_local_path_x_after_Bspline[:]
                del self.past_local_path_y_after_Bspline[:]
                del self.past_local_path_z_after_Bspline[:]
                del self.past_global_x_trajectory[:]
                del self.past_global_y_trajectory[:]
                del self.past_global_z_trajectory[:]

                self.past_concat_x = concat_x
                self.past_concat_y = concat_y
                self.past_concat_z = concat_z
                self.past_local_path_x_after_Bspline = local_path_x_after_Bspline
                self.past_local_path_y_after_Bspline = local_path_y_after_Bspline
                self.past_local_path_z_after_Bspline = local_path_z_after_Bspline

                self.past_global_x_trajectory = global_x_trajectory
                self.past_global_y_trajectory = global_y_trajectory
                self.past_global_z_trajectory = global_z_trajectory
                print("update marker result as the drone moving along the second part of path")
                return self.past_concat_x,self.past_concat_y,self.past_concat_z,self.past_local_path_x_after_Bspline,self.past_local_path_y_after_Bspline,self.past_local_path_z_after_Bspline,self.past_global_x_trajectory,self.past_global_y_trajectory,self.past_global_z_trajectory

            elif (self.update_new_path==True)&(self.use_the_second_part==False):
                self.past_concat_x = list(self.past_concat_x)
                self.past_concat_y = list(self.past_concat_y)
                self.past_concat_z = list(self.past_concat_z)
                self.past_local_path_x_after_Bspline = list(self.past_local_path_x_after_Bspline)
                self.past_local_path_y_after_Bspline = list(self.past_local_path_y_after_Bspline)
                self.past_local_path_z_after_Bspline = list(self.past_local_path_z_after_Bspline)
                self.past_global_x_trajectory = list(self.past_global_x_trajectory)
                self.past_global_y_trajectory = list(self.past_global_y_trajectory)
                self.past_global_z_trajectory = list(self.past_global_z_trajectory)
                del self.past_concat_x[:]
                del self.past_concat_y[:]
                del self.past_concat_z[:]
                del self.past_local_path_x_after_Bspline[:]
                del self.past_local_path_y_after_Bspline[:]
                del self.past_local_path_z_after_Bspline[:]
                del self.past_global_x_trajectory[:]
                del self.past_global_y_trajectory[:]
                del self.past_global_z_trajectory[:]

                self.past_concat_x = concat_x
                self.past_concat_y = concat_y
                self.past_concat_z = concat_z
                self.past_local_path_x_after_Bspline = local_path_x_after_Bspline
                self.past_local_path_y_after_Bspline = local_path_y_after_Bspline
                self.past_local_path_z_after_Bspline = local_path_z_after_Bspline

                self.past_global_x_trajectory = global_x_trajectory
                self.past_global_y_trajectory = global_y_trajectory
                self.past_global_z_trajectory = global_z_trajectory
                print("update marker result as the drone moving along the path")
                return self.past_concat_x,self.past_concat_y,self.past_concat_z,self.past_local_path_x_after_Bspline,self.past_local_path_y_after_Bspline,self.past_local_path_z_after_Bspline,self.past_global_x_trajectory,self.past_global_y_trajectory,self.past_global_z_trajectory
        else:
            print("global path at beginning: {}".format(self.global_path_at_beginning))
            print("update new path: {}".format(self.update_new_path))
            print("use the second part: {}".format(self.use_the_second_part))
            print("updated path time: {}".format(self.updated_path_time))
            print("output marker result!")
            return self.past_concat_x,self.past_concat_y,self.past_concat_z,self.past_local_path_x_after_Bspline,self.past_local_path_y_after_Bspline,self.past_local_path_z_after_Bspline,self.past_global_x_trajectory,self.past_global_y_trajectory,self.past_global_z_trajectory


    # Moving in local position
    def local_position(self,concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory):
        self.model = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        self.callback_local_position()
        
        local_trajectory = Path()
        local_path_x_after_Bspline=list(local_path_x_after_Bspline)
        local_path_y_after_Bspline=list(local_path_y_after_Bspline)
        local_path_z_after_Bspline=list(local_path_z_after_Bspline)
        local_path_x_after_Bspline = [round(num, 3) for num in local_path_x_after_Bspline]
        local_path_y_after_Bspline = [round(num, 3) for num in local_path_y_after_Bspline]
        local_path_z_after_Bspline = [round(num, 3) for num in local_path_z_after_Bspline]

        global_x_trajectory=list(global_x_trajectory)
        global_y_trajectory=list(global_y_trajectory)
        global_z_trajectory=list(global_z_trajectory)
        global_x_trajectory = [round(num, 3) for num in global_x_trajectory]
        global_y_trajectory = [round(num, 3) for num in global_y_trajectory]
        global_z_trajectory = [round(num, 3) for num in global_z_trajectory]

        if self.global_path_at_beginning == True:
            if  (self.update_new_path == True)&(self.use_the_second_part == False):
                local_trajectory_pose = PoseStamped()
                local_trajectory_pose.header.frame_id = world_frame_name
                local_trajectory_pose.header.stamp = rospy.Time.now()
                local_trajectory_pose.pose.position.x=global_x_trajectory[0]
                local_trajectory_pose.pose.position.y=global_y_trajectory[0]
                local_trajectory_pose.pose.position.z=global_z_trajectory[0]
                local_trajectory_pose.pose.orientation.w = 1.0
                local_trajectory.header.frame_id = world_frame_name
                local_trajectory.header.stamp = rospy.Time.now()
                local_trajectory.poses.append(local_trajectory_pose)
                self.local_trajectory_vis.publish(local_trajectory)

                posestamped = PoseStamped()
                posestamped.header.stamp = rospy.Time.now()
                posestamped.header.frame_id = base_link_name
                posestamped.pose.position.x = global_x_trajectory[0]
                posestamped.pose.position.y = global_y_trajectory[0]
                posestamped.pose.position.z = global_z_trajectory[0]
                tmp_y=self.target_position_y-self.local_position_y
                tmp_x=self.target_position_x-self.local_position_x
                if len(global_x_trajectory)==20:
                    self.local_position_x_at_20 = self.local_position_x
                    self.local_position_y_at_20 = self.local_position_y
                if len(global_x_trajectory)<=20:
                    tmp_y=self.target_position_y-self.local_position_y_at_20
                    tmp_x=self.target_position_x-self.local_position_x_at_20
                quatrernion=[]
                if (tmp_x<0) & (tmp_y<0):
                    yaw_radian =  math.atan(tmp_x/tmp_y)*-1-self.pi/2
                elif (tmp_x<0) & (tmp_y>0):
                    yaw_radian =  math.atan(tmp_x/tmp_y)*-1+self.pi/2
                elif (tmp_x>0) & (tmp_y>0):
                    yaw_radian =  self.pi/2-math.atan(tmp_x/tmp_y)
                elif (tmp_x>0) & (tmp_y<0):
                    yaw_radian =  self.pi*1.5-math.atan(tmp_x/tmp_y)
                else:
                    yaw_radian = 0
                quatrernion = quaternion_from_euler(0, 0, yaw_radian)
                posestamped.pose.orientation.x = quatrernion[0]
                posestamped.pose.orientation.y = quatrernion[1]
                posestamped.pose.orientation.z = quatrernion[2]
                posestamped.pose.orientation.w = quatrernion[3]
                self.pass_position_to_gui_pub.publish(posestamped)
                self.update_new_path = True
                self.use_the_second_part = False
                rospy.sleep(rospy.Duration(0.2))
                if len(global_x_trajectory)==1:
                    self.hover_in_goal_position_x = global_x_trajectory[0]
                    self.hover_in_goal_position_y = global_y_trajectory[0]
                    self.hover_in_goal_position_z = global_z_trajectory[0]
                    self.hover_in_goal_quatrernion_x = quatrernion[0]
                    self.hover_in_goal_quatrernion_y = quatrernion[1]
                    self.hover_in_goal_quatrernion_z = quatrernion[2]
                    self.hover_in_goal_quatrernion_w = quatrernion[3]
                global_x_trajectory.pop(0)
                global_y_trajectory.pop(0)
                global_z_trajectory.pop(0)
                print("global_x_trajectory len: {}".format(len(global_x_trajectory)))
                print("global_y_trajectory len: {}".format(len(global_y_trajectory)))
                print("global_z_trajectory len: {}".format(len(global_z_trajectory)))
                if len(global_x_trajectory)==0:
                    self.update_new_path = True
                    self.use_the_second_part = True
                self.store_local_marker_array_result(concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory)
                print("moving global path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            else:
                posestamped = PoseStamped()
                posestamped.header.stamp = rospy.Time.now()
                posestamped.header.frame_id = base_link_name
                posestamped.pose.position.x = self.hover_in_goal_position_x
                posestamped.pose.position.y = self.hover_in_goal_position_y
                posestamped.pose.position.z = self.hover_in_goal_position_z
                posestamped.pose.orientation.x = self.hover_in_goal_quatrernion_x
                posestamped.pose.orientation.y = self.hover_in_goal_quatrernion_y
                posestamped.pose.orientation.z = self.hover_in_goal_quatrernion_z
                posestamped.pose.orientation.w = self.hover_in_goal_quatrernion_w
                self.pass_position_to_gui_pub.publish(posestamped)
                print("Reach the goal of global pathing!!!!!!!!!!!!!!!")
        else:    
            if self.use_the_second_part == False:
                local_trajectory_pose = PoseStamped()
                local_trajectory_pose.header.frame_id = world_frame_name
                local_trajectory_pose.header.stamp = rospy.Time.now()
                local_trajectory_pose.pose.position.x=local_path_x_after_Bspline[0]
                local_trajectory_pose.pose.position.y=local_path_y_after_Bspline[0]
                local_trajectory_pose.pose.position.z=local_path_z_after_Bspline[0]
                local_trajectory_pose.pose.orientation.w = 1.0
                local_trajectory.header.frame_id = world_frame_name
                local_trajectory.header.stamp = rospy.Time.now()
                local_trajectory.poses.append(local_trajectory_pose)
                self.local_trajectory_vis.publish(local_trajectory)

                posestamped = PoseStamped()
                posestamped.header.stamp = rospy.Time.now()
                posestamped.header.frame_id = base_link_name
                posestamped.pose.position.x = local_path_x_after_Bspline[0]
                posestamped.pose.position.y = local_path_y_after_Bspline[0]
                posestamped.pose.position.z = local_path_z_after_Bspline[0]

                if len(local_path_x_after_Bspline)<=20:
                    tmp_y=self.target_position_y-local_path_y_after_Bspline[0]
                    tmp_x=self.target_position_x-local_path_x_after_Bspline[0]
                else:
                    tmp_y=local_path_y_after_Bspline[1]-local_path_y_after_Bspline[0]
                    tmp_x=local_path_x_after_Bspline[1]-local_path_x_after_Bspline[0]
                quatrernion=[]        
                yaw_radian =  math.atan(tmp_x/tmp_y)
                if (tmp_x<0) & (tmp_y<0):
                    yaw_radian =  math.atan(tmp_x/tmp_y)*-1-self.pi/2
                elif (tmp_x<0) & (tmp_y>0):
                    yaw_radian =  math.atan(tmp_x/tmp_y)*-1+self.pi/2
                elif (tmp_x>0) & (tmp_y>0):
                    yaw_radian =  self.pi/2-math.atan(tmp_x/tmp_y)
                elif (tmp_x>0) & (tmp_y<0):
                    yaw_radian =  self.pi*1.5-math.atan(tmp_x/tmp_y)
                else:
                    yaw_radian = 0
                quatrernion = quaternion_from_euler(0, 0, yaw_radian)
                posestamped.pose.orientation.x = quatrernion[0]
                posestamped.pose.orientation.y = quatrernion[1]
                posestamped.pose.orientation.z = quatrernion[2]
                posestamped.pose.orientation.w = quatrernion[3]

                self.pass_position_to_gui_pub.publish(posestamped)
                self.update_new_path = True
                rospy.sleep(rospy.Duration(0.2))
                local_path_x_after_Bspline.pop(0)
                local_path_y_after_Bspline.pop(0)
                local_path_z_after_Bspline.pop(0)
                print("local_path_x_after_Bspline len: {}".format(len(local_path_x_after_Bspline)))
                print("local_path_y_after_Bspline len: {}".format(len(local_path_y_after_Bspline)))
                print("local_path_z_after_Bspline len: {}".format(len(local_path_z_after_Bspline)))
                if len(local_path_x_after_Bspline)==0:
                    self.update_new_path = False
                    self.use_the_second_part = False
                self.store_local_marker_array_result(concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory)
                print("moving!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            elif self.use_the_second_part == True:
                local_trajectory_pose = PoseStamped()
                local_trajectory_pose.header.frame_id = world_frame_name
                local_trajectory_pose.header.stamp = rospy.Time.now()
                local_trajectory_pose.pose.position.x=global_x_trajectory[0]
                local_trajectory_pose.pose.position.y=global_y_trajectory[0]
                local_trajectory_pose.pose.position.z=global_z_trajectory[0]
                local_trajectory_pose.pose.orientation.w = 1.0
                local_trajectory.header.frame_id = world_frame_name
                local_trajectory.header.stamp = rospy.Time.now()
                local_trajectory.poses.append(local_trajectory_pose)
                self.local_trajectory_vis.publish(local_trajectory)

                posestamped = PoseStamped()
                posestamped.header.stamp = rospy.Time.now()
                posestamped.header.frame_id = base_link_name
                posestamped.pose.position.x = global_x_trajectory[0]
                posestamped.pose.position.y = global_y_trajectory[0]
                posestamped.pose.position.z = global_z_trajectory[0]

                tmp_y=self.target_position_y-global_y_trajectory[0]
                tmp_x=self.target_position_x-global_x_trajectory[0]
                if len(global_x_trajectory)==20:
                    self.local_position_x_at_20 = self.local_position_x
                    self.local_position_y_at_20 = self.local_position_y
                if len(global_x_trajectory)<=20:
                    tmp_y=self.target_position_y-self.local_position_y_at_20
                    tmp_x=self.target_position_x-self.local_position_x_at_20

                quatrernion=[]        
                if (tmp_x<0) & (tmp_y<0):
                    yaw_radian =  math.atan(tmp_x/tmp_y)*-1-self.pi/2
                elif (tmp_x<0) & (tmp_y>0):
                    yaw_radian =  math.atan(tmp_x/tmp_y)*-1+self.pi/2
                elif (tmp_x>0) & (tmp_y>0):
                    yaw_radian =  self.pi/2-math.atan(tmp_x/tmp_y)
                elif (tmp_x>0) & (tmp_y<0):
                    yaw_radian =  self.pi*1.5-math.atan(tmp_x/tmp_y)
                else:
                    yaw_radian = 0
                quatrernion = quaternion_from_euler(0, 0, yaw_radian)
                posestamped.pose.orientation.x = quatrernion[0]
                posestamped.pose.orientation.y = quatrernion[1]
                posestamped.pose.orientation.z = quatrernion[2]
                posestamped.pose.orientation.w = quatrernion[3]
                self.pass_position_to_gui_pub.publish(posestamped)
                self.update_new_path = False
                rospy.sleep(rospy.Duration(0.2))
                global_x_trajectory.pop(0)
                global_y_trajectory.pop(0)
                global_z_trajectory.pop(0)
                print("global_x_trajectory len: {}".format(len(global_x_trajectory)))
                print("global_y_trajectory len: {}".format(len(global_y_trajectory)))
                print("global_z_trajectory len: {}".format(len(global_z_trajectory)))
                if len(global_x_trajectory)==1:
                    self.hover_in_goal_position_x = global_x_trajectory[0]
                    self.hover_in_goal_position_y = global_y_trajectory[0]
                    self.hover_in_goal_position_z = global_z_trajectory[0]
                    self.hover_in_goal_quatrernion_x = quatrernion[0]
                    self.hover_in_goal_quatrernion_y = quatrernion[1]
                    self.hover_in_goal_quatrernion_z = quatrernion[2]
                    self.hover_in_goal_quatrernion_w = quatrernion[3]
                if len(global_x_trajectory)==0:
                    self.update_new_path = False
                    self.use_the_second_part = False
                    self.global_path_at_beginning = True
                self.store_local_marker_array_result(concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory)
            
                print("moving with second part!!!!!!!!!!!!!!!!!")

    def current_position(self):
        tmp_y=self.target_position_y-self.local_position_y
        tmp_x=self.target_position_x-self.local_position_x
        quatrernion=[]
        yaw_radian = 0
        if (tmp_x<0) & (tmp_y<0):
            yaw_radian =  math.atan(tmp_x/tmp_y)*-1-self.pi/2
        elif (tmp_x<0) & (tmp_y>0):
            yaw_radian =  math.atan(tmp_x/tmp_y)*-1+self.pi/2
        elif (tmp_x>0) & (tmp_y>0):
            yaw_radian =  self.pi/2-math.atan(tmp_x/tmp_y)
        elif (tmp_x>0) & (tmp_y<0):
            yaw_radian =  self.pi*1.5-math.atan(tmp_x/tmp_y)
        else:
            yaw_radian = 0
        quatrernion = quaternion_from_euler(0, 0, yaw_radian)
        self.target_orientation_x = quatrernion[0]
        self.target_orientation_y = quatrernion[1]
        self.target_orientation_z = quatrernion[2]
        self.target_orientation_w = quatrernion[3]
        posestamped = PoseStamped()
        posestamped.header.stamp = rospy.Time.now()
        posestamped.header.frame_id = base_link_name
        posestamped.pose.position.x = self.local_position_x
        posestamped.pose.position.y = self.local_position_y
        posestamped.pose.position.z = self.local_position_z
        posestamped.pose.orientation.x = self.target_orientation_x
        posestamped.pose.orientation.y = self.target_orientation_y
        posestamped.pose.orientation.z = self.target_orientation_z
        posestamped.pose.orientation.w = self.target_orientation_w
        self.pass_position_to_gui_pub.publish(posestamped)

    # Create marker in rviz
    def clear_marker(self):
        marker = Marker()
        marker_array = MarkerArray()
        marker.id = 0
        marker.ns = "nearest_position_with_kmean_cluster_centroid_marker"
        marker.action = marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_vis.publish(marker_array)
        validation_marker = Marker()
        validation_marker_array = MarkerArray()
        validation_marker.id = 0
        validation_marker.ns = "nearest_position_with_kmean_cluster_centroid_marker"
        validation_marker.action = validation_marker.DELETEALL
        validation_marker_array.markers.append(validation_marker)
        self.validation_marker_vis.publish(validation_marker_array)
        local_marker = Marker()
        local_marker_array = MarkerArray()
        local_marker.id = 0
        local_marker.ns = "local_marker"
        local_marker.action = local_marker.DELETEALL
        local_marker_array.markers.append(local_marker)
        self.local_marker_vis.publish(local_marker_array)

    def arm_and_offboard(self):
        self.model = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        self.callback_local_position()
        if (self.update_new_path == False)&(self.use_the_second_part == False):
            passthrough_height,cluster_center_list,nearest_point_circle,local_path_x_leftside,local_path_y_leftside,local_path_x_rightside,local_path_y_rightside=self.path_planner()
            self.create_marker(cluster_center_list)
            self.create_validation_marker(nearest_point_circle)
            concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory=self.create_local_marker_array(passthrough_height,local_path_x_leftside, local_path_y_leftside,local_path_x_rightside,local_path_y_rightside)
            self.local_marker_vis.publish(self.local_marker_array)
        
        concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory=self.store_local_marker_array_result(None,None,None,None,None,None,None,None,None)
        self.local_position(concat_x,concat_y,concat_z,local_path_x_after_Bspline,local_path_y_after_Bspline,local_path_z_after_Bspline,global_x_trajectory,global_y_trajectory,global_z_trajectory)        
        
        if (round(self.past_target_position_x,1)!=round(self.target_position_x,1))or(round(self.past_target_position_y,1)!=round(self.target_position_y,1))or(round(self.past_target_position_z,1)!=round(self.target_position_z,1)):
            self.past_target_position_x=self.target_position_x
            self.past_target_position_y=self.target_position_y
            self.past_target_position_z=self.target_position_z
            self.update_new_path = False
            self.use_the_second_part = False
            self.updated_path_time = 0
            self.global_path_at_beginning = False
            self.finish_ready_position = False
            self.clear_marker()
            print("Start new journal")

if __name__=="__main__":
    start = moving()
    #start.cmd()
    while not rospy.is_shutdown():
        start.model = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        print("auto mode: {}".format(start.auto_mode))

        if start.auto_mode==True:
            # Control the drone
            while start.finish_ready_position==False:
                for i in range(20):
                    start.model = rospy.wait_for_message('/gazebo/model_states', ModelStates)
                    start.callback_local_position()
                    start.clear_marker()
                    start.current_position()
                    rospy.sleep(rospy.Duration(0.1))
                    print(i)
                start.past_target_position_x=start.target_position_x
                start.past_target_position_y=start.target_position_y
                start.past_target_position_z=start.target_position_z
                start.finish_ready_position=True
                del start.pointcloud_x_list[:]
                del start.pointcloud_y_list[:]
                del start.pointcloud_z_list[:]
                break
            start.arm_and_offboard()
        else:
            start.model = rospy.wait_for_message('/gazebo/model_states', ModelStates)
            start.callback_local_position()
            #start.clear_marker()
            start.current_position()
            start.update_new_path = False
            start.use_the_second_part = False
            start.updated_path_time = 0
            start.global_path_at_beginning = False
            start.finish_ready_position = False
        del start.pointcloud_x_list[:]
        del start.pointcloud_y_list[:]
        del start.pointcloud_z_list[:]

