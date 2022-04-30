#!/usr/bin/env python3
from __future__ import print_function
from mimetypes import guess_all_extensions

import cv2
import time  
import Pose_util.PoseModule as pm
import rospy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from Pose_util.hand_angle_dataset     import hand_angle_dataset

class gesture_detect:
    def __init__(self):
        self.cvImage_Subscriber = rospy.Subscriber('/camera/color/image_raw',Image,self.cvImage_Subscriber_Callback)
        self.depthImage_Subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,self.depthImage_Subscriber_Callback)
        self.cameraInfo_subscriber = rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.Info_Subscriber_Callback)

        self.human_image_Publisher = rospy.Publisher('/detected_human',Image,queue_size=10)
        self.human_gesture_Publisher = rospy.Publisher('/detected_human_gesture',String,queue_size=10)

        self.color_image = None
        self.depth_image = None
        self.image_length = None
        self.image_width = None
        self.depth = None
        self.bridge = CvBridge()


        self.detector = pm.poseDetector()
        self.angle_data = hand_angle_dataset()

    def cvImage_Subscriber_Callback(self,data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        img = self.detector.findPose(self.color_image,True)
        lmList = self.detector.findPosition(img,True)

        Left_straight, Right_straight, Left_angle, Right_angle, Gesture =self.angle_data.cal_angle(lmList)
        depth_of_human,cx,cy = self.angle_data.depth_calculation(lmList,self.depth_image)


        cv2.circle(img,(cx,cy), 5, (0,100,200),cv2.FILLED)
        cv2.putText(img,"depth: %.2f m" %(depth_of_human/1000),(self.image_length - 400 ,self.image_width - 160),cv2.FONT_HERSHEY_PLAIN,2,(0,0,128),3)
        cv2.putText(img,"gesture: " + str(Gesture),(self.image_length - 400 ,self.image_width - 130),cv2.FONT_HERSHEY_PLAIN,2,(0,0,128),3)
        cv2.putText(img,"left straight: " + str(Left_straight),(self.image_length - 400 ,self.image_width - 100),cv2.FONT_HERSHEY_PLAIN,2,(0,0,128),3)
        cv2.putText(img,"right straight: " + str(Right_straight),(self.image_length - 400 ,self.image_width - 70),cv2.FONT_HERSHEY_PLAIN,2,(0,0,128),3)
        cv2.putText(img,"left angle: " + str(Left_angle),(self.image_length - 400 ,self.image_width - 40),cv2.FONT_HERSHEY_PLAIN,2,(0,0,128),3)
        cv2.putText(img,"right angle: " + str(Right_angle),(self.image_length - 400 ,self.image_width - 10),cv2.FONT_HERSHEY_PLAIN,2,(0,0,128),3)

        cv2.imshow('image',img)
        cv2.imshow('depth',self.depth_image)
        self.human_gesture_Publisher.publish(str(Gesture))
        try:
            ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.human_image_Publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)
        if cv2.waitKey(3) & 0xFF == ord('d'):
            print(self.color_image.shape)
            print(self.depth_image.shape)


    def Info_Subscriber_Callback(self,data):
        self.image_length = int(data.P[2]*2)
        self.image_width = int(data.P[6]*2)
        # rospy.loginfo(" length:%s width:%s",data.s[2],data.P[6])

    def depthImage_Subscriber_Callback(self,data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

def main():
    rospy.init_node('PostDetectV2', anonymous=True)
    glo_RHD = gesture_detect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__== "__main__":
    main()
