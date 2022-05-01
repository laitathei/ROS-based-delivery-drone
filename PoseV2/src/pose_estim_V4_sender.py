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



class ArHey_simulate():

    def __init__(self):
        self.cvImage_Subscriber = rospy.Subscriber('/camera/color/image_raw',Image,self.cvImage_Subscriber_Callback)

        self.detection_result_Publisher = rospy.Publisher('/detected_result',String,queue_size=10)

        self.bridge = CvBridge()
        self.PIL_img = None
        self.cv_img = None
        self.yolo = YOLO()

    def cvImage_Subscriber_Callback(self,data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        else:
            try:
                self.PIL_img = PIL.Image.fromarray(cv2.cvtColor(self.color_image,cv2.COLOR_BGR2RGB))
                self.PIL_img,boxs = self.yolo.detect_image(self.PIL_img)
                self.cv_img = cv2.cvtColor(np.array(self.PIL_img), cv2.COLOR_RGB2BGR)
            except:
                cv2.imshow('image',self.color_image)
            else:
                cv2.imshow('image',self.cv_img)
                # print(self.encode(boxs))
                data_str = self.encode(boxs)
                self.detection_result_Publisher.publish(data_str)
        
        cv2.waitKey(1)

    def encode(self,input):
        output = ''
        for index, line in enumerate(input):
            for id, data in enumerate(line):
                output += str(data)
                if id != len(line)-1:
                    output += ','  
            if index != len(input)-1:  
                output += ';'
        return output



def main():
    rospy.init_node('Arhey',anonymous= True)
    setup = ArHey_simulate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shut down')

if __name__ == '__main__':
    main()
