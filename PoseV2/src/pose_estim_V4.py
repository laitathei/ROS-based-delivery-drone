#!/usr/bin/env python3
from __future__ import print_function

import cv2
import time  
import Pose_util.PoseModule as pm
import rospy
import math
import PIL.Image
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from Pose_util.hand_angle_dataset import hand_angle_dataset
from yolov4_tiny.yolo import YOLO

class total_operation():

    def __init__(self):
        self.cvImage_Subscriber = rospy.Subscriber('/camera/color/image_raw',Image,self.cvImage_Subscriber_Callback)
        self.depthImage_Subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,self.depthImage_Subscriber_Callback)
        self.cameraInfo_subscriber = rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.Info_Subscriber_Callback)
        self.detect_result_subscriber = rospy.Subscriber('/detected_result',String,self.detection_Subscriber_Callback)

        self.human_image_Publisher = rospy.Publisher('/detected_human',Image,queue_size=10)
        self.human_gesture_Publisher = rospy.Publisher('/detected_human_gesture',String,queue_size=10)

        self.color_image = None
        self.depth_image = None
        self.image_length = None
        self.image_width = None
        self.depth = None
        self.bridge = CvBridge()
        self.PIL_img = None
        self.cv_img = None
        self.num_of_people = 0

        self.detector = pm.poseDetector()
        self.angle_data = hand_angle_dataset()



    def detection_Subscriber_Callback(self,data):
        # print(data.data)
        if data.data != '':
            boxes = self.decode(data.data)
            minimum_data = [-1,100000,-1]
            current_color_image = self.color_image 
            current_depth_image = self.depth_image
            depth_data = []
            if len(boxes) != 0:
                for id, box in enumerate(boxes):
                    try:
                        top,left,bottom,right = box[:4]
                        # cropped_img = self.color_image[box[0]:box[2],box[1]:box[3]]
                        centre_y = (top*3+bottom*2)//5
                        centre_x = (left+right)//2
                        cv2.circle(current_color_image,(centre_x,centre_y), 5, (0,100,200),cv2.FILLED)
                        cropped_img = current_depth_image[centre_y-25:centre_y+25,centre_x-25:centre_x+25]
                        avg = np.mean(cropped_img)
                        # avg = self.depth_image[centre_y,centre_x]
                        cv2.putText(current_color_image,"%.2f m" %(avg/1000),(left+10 ,top+20),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,128,0),3)
                        depth_data.append(avg)
                    except: 
                        print('detection{} error'.format(id))

                print(depth_data)
                if len(depth_data) != 0:
                    depth_data = np.array(depth_data)
                    minimum_index = np.argmin(depth_data)
                    # minimum_index = 0
                    box = boxes[minimum_index] 
                    top,left,bottom,right = self.angle_data.regulation_box(box,self.image_length,self.image_width)
                    try:
                        cropped_img = current_color_image[top:bottom,left:right]
                        cropped_img = self.detector.findPose(cropped_img,True)
                        lmList = self.detector.findPosition(cropped_img,True)
                        Left_straight, Right_straight, Left_angle, Right_angle, Gesture =self.angle_data.cal_angle(lmList)
                        depth_of_human,cx,cy = self.angle_data.depth_calculation(lmList,self.depth_image,left,top)
                        cv2.putText(current_color_image,"depth: %.2f m" %(depth_of_human/1000),(10 ,10),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,0,128),3)
                        cv2.putText(current_color_image,"gesture" + str(Gesture),(10,30),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,0,128),3)

                    except:
                        print('pose detection error')

                    cv2.imshow('img_detect',cropped_img)
                
                # print('depth: {} gesture: {}'.format(depth_of_human/1000,Gesture))
                cv2.imshow('image',self.color_image)
                cv2.waitKey(1)

    def cvImage_Subscriber_Callback(self,data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
    
    def Info_Subscriber_Callback(self,data):
        self.image_length = int(data.P[2]*2)
        self.image_width = int(data.P[6]*2)
        # rospy.loginfo(" length:%s width:%s",data.s[2],data.P[6])

    def depthImage_Subscriber_Callback(self,data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

    def decode(self,data):
        output = data.split(';')
        output = list(map(lambda x: x.split(','),output))
        for index, line in enumerate(output):
            line = list(map(lambda x:int(x),line))
            output[index] = line
        return output


def main():
    rospy.init_node('PostDetectV4', anonymous=True)
    glo_detection = total_operation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # while not rospy.is_shutdown():
    #     rospy.sleep(100)
    

if __name__== "__main__":
    main()
