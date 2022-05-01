#!/usr/bin/env python3
from __future__ import print_function

import cv2
import time  
import Pose_util.PoseModule as pm
import rospy
import math
import PIL.Image
import numpy as np
import torch
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from Pose_util.hand_angle_dataset import hand_angle_dataset

model = torch.hub.load('YOLOv5/yolov5','custom', path='YOLOv5/yolov5/runs/train/exp16/weights/best.pt', source='local') 

class V5_detection():

    def __init__(self):
        self.cvImage_Subscriber = rospy.Subscriber('/camera/color/image_raw',Image,self.cvImage_Subscriber_Callback)
        self.depthImage_Subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,self.depthImage_Subscriber_Callback)
        self.cameraInfo_subscriber = rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.Info_Subscriber_Callback)

        self.human_image_Publisher = rospy.Publisher('/detected_human',Image,queue_size=10)
        self.human_gesture_Publisher = rospy.Publisher('/detected_human_gesture',String,queue_size=10)
        self.human_pos_Publisher = rospy.Publisher('/detected_human_pos',String,queue_size=10)

        self.color_image = None
        self.depth_image = None
        self.image_length = None
        self.image_width = None
        self.depth = None
        self.bridge = CvBridge()
        self.PIL_img = None
        self.cv_img = None
        self.num_of_people = 0
        self.detection_label = ['human','injury']

        self.detector = pm.poseDetector()
        self.angle_data = hand_angle_dataset()

    def cvImage_Subscriber_Callback(self,data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            depth_data = []
        except CvBridgeError as e:
            print(e)
        else:
            global model
            result = model(self.color_image)
            boxes = result.xyxy[0].tolist()
            # boxesP = result.pandas().xyxy[0]
            # print(boxesP)
            for index, box in enumerate(boxes):
                if box[4] >= 0.3:
                    self.draw_box_label_in_cvImg(box)

                    if box[5] == 0:
                        left,top,right,bottom = np.array(box[:4],dtype='int32')
                        centre_y = int((top*3+bottom*2)//5)
                        centre_x = int((left+right)//2)
                        cv2.circle(self.cv_img,(centre_x,centre_y), 5, (0,100,200),cv2.FILLED)
                        cropped_depth_img = self.depth_image[centre_y-25:centre_y+25,centre_x-25:centre_x+25]
                        avg = np.mean(cropped_depth_img)
                        cv2.putText(self.color_image,"%.2f m" %(avg/1000),(int(left+10) ,int(top+20)),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,128,0),3)
                        depth_data.append(avg)

            print(depth_data)

            if len(depth_data) != 0:
                    depth_data = np.array(depth_data)
                    minimum_index = np.argmin(depth_data)
                    box = boxes[minimum_index] 
                    left,top,right,bottom = self.angle_data.regulation_box_v2(box,self.image_length,self.image_width)

                    
                    cropped_img = self.color_image[top:bottom,left:right]
                    cropped_img = self.detector.findPose(cropped_img,True)
                    lmList = self.detector.findPosition(cropped_img,True)
                    # Left_straight, Right_straight, Left_angle, Right_angle, Gesture =self.angle_data.cal_angle(lmList)
                    Gesture = self.angle_data.cal_angle_v2(lmList,int(bottom-top),int(right-left))
                    cv2.putText(self.color_image,"gesture: " + str(Gesture),(10,30),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,0,128),3)
                    self.human_gesture_Publisher.publish(str(Gesture))
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(cropped_img, "bgr8")
                        self.human_image_Publisher.publish(ros_image)
                    except CvBridgeError as e:
                        print(e)

                    # cv2.imshow('cropped_img',cropped_img)


            cv2.imshow('orginal',self.color_image)
            cv2.waitKey(1)

    def Info_Subscriber_Callback(self,data):
        self.image_length = int(data.P[2]*2)
        self.image_width = int(data.P[6]*2)
        # rospy.loginfo(" length:%s width:%s",data.s[2],data.P[6])

    def depthImage_Subscriber_Callback(self,data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

    def draw_box_label_in_cvImg(self, box, color=(0, 128, 0), txt_color=(255, 255, 255)):
        p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
        cv2.rectangle(self.color_image, p1, p2, color, thickness=2, lineType=cv2.LINE_AA)

        tf = 1  # font thickness  max(self.lw - 1, 1)
        w, h = cv2.getTextSize(str(self.detection_label[int(box[5])]), 0, fontScale=2 / 3, thickness=tf)[0]  # text width, height fontScale=self.lw / 3
        outside = p1[1] - h - 3 >= 0  # label fits outside box
        p2 = p1[0] + w, p1[1] - h - 3 if outside else p1[1] + h + 3
        cv2.rectangle(self.color_image, p1, p2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(self.color_image, "{} {}".format(str(self.detection_label[int(box[5])]),str(round(box[4],2))), (p1[0], p1[1] - 2 if outside else p1[1] + h + 2), 0, 2 / 3, txt_color,
                    thickness=tf, lineType=cv2.LINE_AA)


def main():
    rospy.init_node('PostDetectV5', anonymous=True)
    glo_detection = V5_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:   
        print("Shutting down")
    # while not rospy.is_shutdown():
    #     rospy.sleep(100)
    

if __name__== "__main__":
    main()
