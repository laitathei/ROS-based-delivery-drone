import sys
sys.path.append('src/Pose_util')

import Pose_util.PoseModule as pm
from knn import knn
import csv
import numpy as np
import cv2
import math



def main():
    e = 0.00001
    detector = pm.poseDetector()
    glo_knn = knn.KNN()

    csv_data = list(read_csv('knn/data.csv'))
    csv_data = list(map (lambda x: x.split(','),csv_data))
    data_angle = np.array(list(map(lambda x: [int(x[0]),int(x[1])],csv_data)))
    data_result = np.array(list(map(lambda x: int(x[2]),csv_data)))

    img = cv2.imread('image.jpg')
    img = detector.findPose(img)
    data = detector.findPosition(img)
    raw_left_shoulderToHand_angle = math.atan((data[15][2]-data[11][2])/((data[15][1]-data[11][1]) + e)) /math.pi * 180 
    raw_right_shoulderToHand_angle = math.atan((data[16][2]-data[12][2])/((data[16][1]-data[12][1]) + e)) /math.pi * 180
    input_data = [[int(raw_left_shoulderToHand_angle),int(raw_right_shoulderToHand_angle)]]
    
    glo_knn.fit(data_angle,data_result)
    result = glo_knn.predict(input_data)
    cv2.putText(img,"gesture: " + str(result[0]),(10,100),cv2.FONT_HERSHEY_PLAIN,2,(0,0,128),3)
    cv2.imshow('img',img)
    cv2.waitKey(0)
    cv2.waitKey(0)
    cv2.waitKey(0)



def read_csv(link):
    with open(link,'r',newline='') as f:
        rows = csv.reader(f)

        datalist = list(map(lambda x: ','.join(x),rows))

    return datalist

if __name__== "__main__":
    main()