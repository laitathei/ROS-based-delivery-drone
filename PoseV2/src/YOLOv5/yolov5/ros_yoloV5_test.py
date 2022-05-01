#!/usr/bin/env python3
import os, sys

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import numpy as np
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative


from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

class V5_test():
    
    def __init__(self):
        weights=ROOT / 'runs/train/exp16/weights/best.pt',  # model.pt path(s)
        self.source=ROOT / 'data/images',  # file/dir/URL/glob, 0 for webcam
        self.data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        self.imgsz=(640, 640),  # inference size (height, width)
        self.conf_thres=0.25,  # confidence threshold
        self.iou_thres=0.45,  # NMS IOU threshold
        self.max_det=1000,  # maximum detections per image
        self.device='cuda:0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.view_img=False,  # show results
        self.save_txt=False,  # save results to *.txt
        self.save_conf=False,  # save confidences in --save-txt labels
        self.save_crop=False,  # save cropped prediction boxes
        self.nosave=False,  # do not save images/videos
        self.classes=None,  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False,  # class-agnostic NMS
        self.augment=False,  # augmented inference
        self.visualize=False,  # visualize features
        self.update=False,  # update all models
        self.project=ROOT / 'runs/detect',  # save results to project/name
        self.name='exp',  # save results to project/name
        self.exist_ok=False,  # existing project/name ok, do not increment
        self.line_thickness=3,  # bounding box thickness (pixels)
        self.hide_labels=False,  # hide labels
        self.hide_conf=False,  # hide confidences
        self.half=False,  # use FP16 half-precision inference
        self.dnn=False,  # use OpenCV DNN for ONNX inference
        
        self.cvImage_Subscriber = rospy.Subscriber('/camera/color/image_raw',Image,self.cvImage_Subscriber_Callback)

        self.source = str(self.source)

        # Load model
        # self.device = select_device(self.device)
        self.device = torch.device('cuda:0')
        # self.model = DetectMultiBackend(weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)
        # self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        # self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size

        print(weights)
    
    def cvImage_Subscriber_Callback():
        pass



def main():
    rospy.init_node('V5test', anonymous=True)
    glo_RHD =  V5_test()
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')


if __name__ == '__main__':
    main()