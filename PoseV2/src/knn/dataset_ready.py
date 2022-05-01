import Pose_util.PoseModule as pm
import cv2
import math
import csv

def main():
    path =  '/home/ox05/catkin_ws/src/pose/src/knn/data'
    photo = '/pose_{:d}.png'
    e = 0.0000001
    detector = pm.poseDetector()
    dataset = []
    for i in range(1,10):
        for j in range(1,6):
            index = i*10 + j
            img = cv2.imread('{}{}'.format(path,photo.format(index)))
            img = detector.findPose(img)
            data = detector.findPosition(img)
            raw_left_shoulderToHand_angle = math.atan((data[15][2]-data[11][2])/((data[15][1]-data[11][1]) + e)) /math.pi * 180 
            raw_right_shoulderToHand_angle = math.atan((data[16][2]-data[12][2])/((data[16][1]-data[12][1]) + e)) /math.pi * 180
            prediction = [int(raw_left_shoulderToHand_angle),int(raw_right_shoulderToHand_angle),i]
            dataset.append(prediction)
            
            # cv2.imshow('img',img)
            # cv2.waitKey(1)
    
    print(dataset)
    
    with open('data.csv','w',newline='') as f:
        write = csv.writer(f)

        write.writerows(dataset)


if __name__ == '__main__':
    main()