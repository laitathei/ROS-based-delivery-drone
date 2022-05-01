import os
from sre_constants import SUCCESS
import cv2
from cv2 import imwrite

def main():
    path = '.'
    files =  os.listdir(path)
    cnt = 0
    index = 1196
    for file in files:
        name = file.split('.')
        if name[-1] == 'MOV' or name[-1] == 'mp4':
            cap = cv2.VideoCapture(file)
            success, img = cap.read()
            while success:
                success, img = cap.read()
                # cv2.imshow('img',img)
                # cv2.waitKey(10)
                if cnt%15 == 0:
                    cv2.imwrite('../Documents/image_set/human_{}.jpg'.format(index),img)
                    index += 1
                cnt = (cnt +1)%15


if __name__ == '__main__':
    main()