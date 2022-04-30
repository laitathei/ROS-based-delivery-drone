#!/usr/bin/env python3

import cv2
import mediapipe as mp 
import time

class poseDetector():

    def __init__(self, mode=False,complexity = 1, smooth=True, segmentation = False, smoothSem = True, detectionCon=0.5, trackCon=0.5):

        self.mode = mode
        self.complexity = complexity
        self.smooth = smooth
        self.segmentation = segmentation
        self.smoothSem = smoothSem
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(self.mode, self.complexity, self.smooth,self.segmentation, self.smoothSem, self.detectionCon, self.trackCon)
        # (self,
        #        static_image_mode=False,
        #        upper_body_only=False,
        #        smooth_landmarks=True,
        #        min_detection_confidence=0.5,
        #        min_tracking_confidence=0.5):
        # self.pose = self.mpPose.Pose(self.mode,self.segmentation,True,self.detectionCon,self.trackCon)

    def findPose(self, img, draw =True):
        
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)

        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks,self.mpPose.POSE_CONNECTIONS)
        
        return img

    def findPose_multiple(self, img,trimmed_img,x_offset, y_offset, draw =True):
        
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        trimmed_imgRGB = cv2.cvtColor(trimmed_img, cv2.COLOR_BGR2RGB)

        self.results = self.pose.process(trimmed_img)

        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks,self.mpPose.POSE_CONNECTIONS)
                print(self.results.pose_landmarks.shape)

    def findPosition(self, img, draw = True):

        lmList = []
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h,w,c = img.shape
                cx,cy = int(lm.x*w), int(lm.y*h)
                lmList.append([id,cx,cy])
                if draw:
                    cv2.circle(img,(cx,cy), 5, (0,0,255),cv2.FILLED)
        return lmList


    

def main():
    cap = cv2.VideoCapture('/home/ox05/catkin_ws/src/pose/src/pose_estimation_testing_camera_color_image_raw.mp4')
    pTime = 0
    detector = poseDetector()



    while True:
        success, img = cap.read()
        img = detector.findPose(img,True)
        lmList = detector.findPosition(img,True)
        print(lmList)

        # if len(lmList) != 0:
        #     cv2.circle(img,(lmList[14][1],lmList[14][2]), 15,(0,0,255), cv2.FILLED)

        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
        text = 'Fps : ' + str((int)(fps))
        cv2.putText(img,text,(70,50), cv2.FONT_HERSHEY_PLAIN,2,(160,160,0),3)

        
        cv2.imshow("Image",img)

        cv2.waitKey(10)



if __name__== "__main__":
    main()
# img = cv2.imread('/home/jeremy/Documents/Test/Poseestim/testing.jpg',1)
# img1 = cv2.resize(img,(960,540))
# print(img)
# cv2.imshow('image',img1)
# cv2.waitKey(10000)
