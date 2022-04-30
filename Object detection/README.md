### Training process (darknet folder) come from https://github.com/AlexeyAB/darknet
### TensorRT speed up (tensorrt_demos folder) come from https://github.com/jkjung-avt/tensorrt_demos

### 1. Software Prerequisite
### 1.1 Realsense-viewer installation (Jetson version)
```
git clone https://github.com/IntelRealSense/librealsense.git
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

### 1.2 Realsense ROS-Wrapper installation (Jetson version)
```
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
cd /opt/ros/melodic/share/cv_bridge/cmake
sudo gedit cv_bridgeConfig.cmake      ## change the opencv directory to opencv4
python3 -m pip install empy
```

### 1.3 Realsense-viewer installation (PC version)
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

### 1.4 Realsense ROS-Wrapper installation (PC version)
```
cd ~/catkin_ws
cd src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout ***the most stable version and support with your realsense-viewer version***
cd ..

git clone https://github.com/pal-robotics/ddynamic_reconfigure
cd ddynamic_reconfigure/
git checkout ***the most stable version***
cd ..

catkin_make     or      catkin build
```

### 1.5 jsk_pcl_ros installation
```
# Ubuntu 18.04
sudo apt-get install ros-melodic-jsk-pcl-ros
sudo apt-get install ros-melodic-jsk-rviz-plugins
sudo apt-get install ros-melodic-ros-numpy

# Ubuntu 20.04
sudo apt-get install ros-noetic-jsk-pcl-ros
sudo apt-get install ros-noetic-jsk-rviz-plugins
sudo apt-get install ros-noetic-ros-numpy
```

### 1.6 Nvidia-driver installation (PC version)
```
# Remove existing CuDA versions
sudo apt --purge remove "cublas*" "cuda*"
sudo apt --purge remove "nvidia*"
sudo rm -rf /usr/local/cuda*
sudo apt-get autoremove && sudo apt-get autoclean

# Reboot to remove cached files 
reboot

# After reboot
sudo apt-get clean

# check all available nvidia driver version in the repository
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo apt install ubuntu-drivers-common

# check which nvidia driver version available for your own graphic card
ubuntu-drivers devices

# Remember do not install the nvidia-driver-server
# xxx is the nvidia-driver version that you want to install
sudo apt install nvidia-driver-xxx
reboot
```

### 1.7 CUDA installation (PC version)
```
# Go to https://developer.nvidia.com/cuda-toolkit-archive to search the cuda version that you want to install
# Refer to https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html to check which cuda version match with nvidia-driver version
# Higher nvidia-driver version is compatible install lower CUDA version
# Remember not to choose deb(local) for Installer Type if you want to downgrade your CUDA version
# Choose runfile(local) for Installer Type if you want to downgrade your CUDA version
# Then follow the instruction from website

# runfile (local)
(Example for install CUDA 10.1 when you want to downgrade the CUDA after install nvidia-driver-460)
wget https://developer.download.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda_10.1.243_418.87.00_linux.run
# If it show the gcc version fail to verify, please add --override such as sudo sh cuda_10.1.243_418.87.00_linux.run --override
sudo sh cuda_10.1.243_418.87.00_linux.run

Choose continue
Do you accept the above EULA? (accept/decline/quit):
accept

│ CUDA Installer                                                               │
│ - [ ] Driver                                                                 │
│      [ ] 418.87.00                                                           │
│ + [X] CUDA Toolkit 10.1                                                      │
│   [X] CUDA Samples 10.1                                                      │
│   [X] CUDA Demo Suite 10.1                                                   │
│   [X] CUDA Documentation 10.1                                                │
│   Options                                                                    │
│   Install                                                                    │

Choose install
# After installation
nvcc -V (should be show CUDA 10.1)
nvidia-smi (should be show NVIDIA-SMI 460.91.03    Driver Version: 460.91.03    CUDA Version: 11.2)

gedit ~/.bashrc
export PATH=/usr/local/cuda-10.1/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-10.1/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export CUDA_HOME=/usr/local/cuda
source ~/.bashrc


# Deb(local) (CUDA 11.4)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.4.4/local_installers/cuda-repo-ubuntu2004-11-4-local_11.4.4-470.82.01-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-4-local_11.4.4-470.82.01-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-4-local/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda

gedit ~/.bashrc
export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export CUDA_HOME=/usr/local/cuda
source ~/.bashrc
```

### 1.8 cuDNN installation (PC version)
```
# Go to https://developer.nvidia.com/rdp/cudnn-archive to find out the correct Platform and correct CUDA version for your own situation
# Download cuDNN Runtime Library + cuDNN Developer Library by clicking them
# cuDNN Code Samples and User Guide is optional
# Navigate to your <cudnnpath> directory containing the cuDNN Debian file.
# Replace x.x and 8.x.x.x with your specific CUDAand cuDNN versions and package date

sudo dpkg -i libcudnn8_x.x.x-1+cudax.x_amd64.deb
sudo dpkg -i libcudnn8-dev_8.x.x.x-1+cudax.x_amd64.deb
sudo dpkg -i libcudnn8-samples_8.x.x.x-1+cudax.x_amd64.deb (optional)

(Example for install cuDNN 8.0.5 which compatible with CUDA 10.1 in Ubuntu 18.04)
# The following codes must be installed in order
sudo dpkg -i libcudnn8_8.0.5.39-1+cuda10.1_amd64.deb
sudo dpkg -i libcudnn8-dev_8.0.5.39-1+cuda10.1_amd64.deb
sudo dpkg -i libcudnn8-samples_8.0.5.39-1+cuda10.1_amd64.deb (optional)

sudo cp /usr/include/cudnn.h /usr/local/cuda/include
sudo chmod a+x /usr/local/cuda/include/cudnn.h
cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A 2

# Show below information means install cuDNN success
  #define CUDNN_MAJOR 8
  #define CUDNN_MINOR 0
  #define CUDNN_PATCHLEVEL 5
  --
  #define CUDNN_VERSION (CUDNN_MAJOR * 1000 + CUDNN_MINOR * 100 + CUDNN_PATCHLEVEL)

  #endif /* CUDNN_VERSION_H */
```

### 1.9 TensorRT installation (PC version)
```
## pip install
python3 -m pip install numpy
python3 -m pip install 'pycuda<2021.1'
python3 -m pip install --upgrade setuptools pip
python3 -m pip install nvidia-pyindex
python3 -m pip install --upgrade nvidia-tensorrt

## deb install
# Go to https://developer.nvidia.com/nvidia-tensorrt-8x-download find out the suitable TensorRT version that correspond with your cuda version
# After finish download
os="ubuntuxx04"
tag="cudax.x-trt8.x.x.x-yyyymmdd"
sudo dpkg -i nv-tensorrt-repo-${os}-${tag}_1-1_amd64.deb
sudo apt-key add /var/nv-tensorrt-repo-${os}-${tag}/7fa2af80.pub

sudo apt-get update
sudo apt-get install tensorrt
sudo apt-get install python3-libnvinfer-dev
sudo apt-get install uff-converter-tf
sudo apt-get install onnx-graphsurgeon
dpkg -l | grep TensorRT
# it should be shown below if you install successfully
ii  graphsurgeon-tf	8.2.0-1+cuda11.4	amd64	GraphSurgeon for TensorRT package
ii  libnvinfer-bin		8.2.0-1+cuda11.4	amd64	TensorRT binaries
ii  libnvinfer-dev		8.2.0-1+cuda11.4	amd64	TensorRT development libraries and headers
ii  libnvinfer-doc		8.2.0-1+cuda11.4	all	TensorRT documentation
ii  libnvinfer-plugin-dev	8.2.0-1+cuda11.4	amd64	TensorRT plugin libraries
ii  libnvinfer-plugin8	8.2.0-1+cuda11.4	amd64	TensorRT plugin libraries
ii  libnvinfer-samples	8.2.0-1+cuda11.4	all	TensorRT samples
ii  libnvinfer8		8.2.0-1+cuda11.4	amd64	TensorRT runtime libraries
ii  libnvonnxparsers-dev		8.2.0-1+cuda11.4	amd64	TensorRT ONNX libraries
ii  libnvonnxparsers8	8.2.0-1+cuda11.4	amd64	TensorRT ONNX libraries
ii  libnvparsers-dev	8.2.0-1+cuda11.4	amd64	TensorRT parsers libraries
ii  libnvparsers8	8.2.0-1+cuda11.4	amd64	TensorRT parsers libraries
ii  python3-libnvinfer	8.2.0-1+cuda11.4	amd64	Python 3 bindings for TensorRT
ii  python3-libnvinfer-dev	8.2.0-1+cuda11.4	amd64	Python 3 development package for TensorRT
ii  tensorrt		8.2.0.x-1+cuda11.4 	amd64	Meta package of TensorRT
ii  uff-converter-tf	8.2.0-1+cuda11.4	amd64	UFF converter for TensorRT package
ii  onnx-graphsurgeon   8.2.0-1+cuda11.4  amd64 ONNX GraphSurgeon for TensorRT package

# if you find the below problem, you should install the correct CUDA version
The following packages have unmet dependencies:
 tensorrt : Depends: libnvinfer8 (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvinfer-plugin8 (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvparsers8 (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvonnxparsers8 (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvinfer-bin (= 8.0.3-1+cuda11.3) but it is not going to be installed
            Depends: libnvinfer-dev (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvinfer-plugin-dev (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvparsers-dev (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvonnxparsers-dev (= 8.0.3-1+cuda11.3) but 8.2.3-1+cuda11.4 is to be installed
            Depends: libnvinfer-samples (= 8.0.3-1+cuda11.3) but it is not going to be installed
            Depends: libnvinfer-doc (= 8.0.3-1+cuda11.3) but it is not going to be installed
E: Unable to correct problems, you have held broken packages.
```

### YOLOv4-Darknet-TensorRT
### 2. Training in Darknet
### 2.1 Download YOLOv4-Darknet
```
git clone https://github.com/AlexeyAB/darknet
```

### 2.2 Change YOLOv4-Darknet configuration
```
cd darknet/
gedit Makefile


GPU=1
CUDNN=1
CUDNN_HALF=1
OPENCV=1
AVX=0
OPENMP=0
LIBSO=1
ZED_CAMERA=0
ZED_CAMERA_v2_8=0

# set GPU=1 and CUDNN=1 to speedup on GPU
# set CUDNN_HALF=1 to further speedup 3 x times (Mixed-precision on Tensor Cores) GPU: Volta, Xavier, Turing and higher
# set AVX=1 and OPENMP=1 to speedup on CPU (if error occurs then set AVX=0)
# set ZED_CAMERA=1 to enable ZED SDK 3.0 and above
# set ZED_CAMERA_v2_8=1 to enable ZED SDK 2.X

make
```

### 2.3 Build the data folder structure
```
cd /home/laitathei/Desktop/darknet
mkdir VOCdevkit
cd VOCdevkit
mkdir VOC2007
cd VOC2007
mkdir Annotations
mkdir ImageSets
mkdir JPEGImages
cd ImageSets
mkdir Main
```

### 2.4 Seperate dataset to train,test,val and Change the VOC format to YOLO format
```
mv /home/laitathei/Desktop/darknet/voc2yolo4.py /home/laitathei/Desktop/darknet/VOCdevkit/VOC2007
cd /home/laitathei/Desktop/darknet/VOCdevkit/VOC2007
python3 voc2yolo4.py
mv /home/laitathei/Desktop/darknet/voc_annotation.py /home/laitathei/Desktop/darknet/VOCdevkit
cd /home/laitathei/Desktop/darknet/VOCdevkit
python3 voc_annotation.py
```

### 2.5 Change ```voc.names``` setting
```
cp /home/laitathei/Desktop/darknet/data/voc.names /home/laitathei/Desktop/darknet/VOCdevkit
gedit voc.names

obstacle
human
injury
```

### 2.6 Change ```voc.data``` setting
```
cp /home/laitathei/Desktop/darknet/cfg/voc.data /home/laitathei/Desktop/darknet/VOCdevkit
gedit voc.data

classes= 3
train  = /home/laitathei/Desktop/darknet/VOCdevkit/2007_train.txt
valid  = /home/laitathei/Desktop/darknet/VOCdevkit/2007_test.txt
names = /home/laitathei/Desktop/darknet/VOCdevkit/voc.names
backup = /home/laitathei/Desktop/darknet/backup/
```

### 2.7 Change ```yolov4-tiny.cfg``` setting, Remember change [convolutional] & [yolo] in line 226 and 270
```
cp /home/laitathei/Desktop/darknet/cfg/yolov4-tiny.cfg /home/laitathei/Desktop/darknet/VOCdevkit
gedit yolov4-tiny.cfg

[net]
# Testing
#batch=1
#subdivisions=1
# Training
batch=64
subdivisions=16
width=416
height=416
channels=3
momentum=0.9
decay=0.0005
angle=0
saturation = 1.5
exposure = 1.5
hue=.1

learning_rate=0.00261
burn_in=1000

max_batches = 6000    # classes*2000
policy=steps
steps=4800,5400       # 80% and 90% of max_batches
scales=.1,.1

[convolutional]
size=1
stride=1
pad=1
filters=255          # 3*(classes +5)
activation=linear



[yolo]
mask = 3,4,5
anchors = 10,14,  23,27,  37,58,  81,82,  135,169,  344,319
classes=3            # your dataset classes
num=6
jitter=.3
scale_x_y = 1.05
cls_normalizer=1.0
iou_normalizer=0.07
iou_loss=ciou
ignore_thresh = .7
truth_thresh = 1
random=0
resize=1.5
nms_kind=greedynms
beta_nms=0.6
#new_coords=1
#scale_x_y = 2.0
```

### 2.8 Download YOLO weight
```
# YOLOv4-tiny
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.conv.29
# YOLOv4
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.conv.137
```

### 2.9 Data folder structure
```
├── VOCdevkit
   ├── VOC2007
       ├── Annotations
       │   ├── xxx.xml
       │   ├──   ...
       │   
       ├── ImageSets
       │   ├── Main
       │       ├── test.txt
       │       ├── train.txt
       │       ├── trainval.txt
       │       ├── val.txt
       │   
       ├── labels
       │   ├── xxx.txt
       |   ├──   ...
       │   
       ├── JPEGImages
       │   ├── xxx.jpg
       |   ├──   ...
       │   
       └── voc2yolo4.py
   ├── 2007_train.txt
   ├── 2007_test.txt
   ├── 2007_valid.txt
   ├── train.all.txt
   ├── train.txt
   ├── voc.data
   ├── voc.names
   ├── voc_annotation.py
   └── yolov4-tiny.cfg
```

### 2.10 Training
```
cd..
./darknet partial cfg/yolov4-tiny.cfg yolov4-tiny.weights yolov4-tiny.conv.29 29    # optional
./darknet detector train VOCdevkit/voc.data VOCdevkit/yolov4-tiny.cfg yolov4-tiny.conv.29

## Below content will show if program success
 Tensor Cores are used.

 6000: 0.062273, 0.062858 avg loss, 0.000026 rate, 0.380254 seconds, 384000 images, 0.010664 hours left
Saving weights to /home/laitathei/Desktop/darknet/backup//yolov4-tiny_6000.weights
Saving weights to /home/laitathei/Desktop/darknet/backup//yolov4-tiny_last.weights
Saving weights to /home/laitathei/Desktop/darknet/backup//yolov4-tiny_final.weights
If you want to train from the beginning, then use flag in the end of training command: -clear
```

### 2.11 Evaluates Trained weight performance
```
./darknet detector map VOCdevkit/voc.data VOCdevkit/yolov4-tiny.cfg backup/yolov4-tiny_last.weights

## Below content will show if program success
class_id = 0, name = obstacle, ap = 0.00%   	 (TP = 0, FP = 0) 
class_id = 1, name = human, ap = 34.97%   	 (TP = 239, FP = 2) 
class_id = 2, name = injury, ap = 34.86%   	 (TP = 41, FP = 0) 

 for conf_thresh = 0.25, precision = 0.99, recall = 0.29, F1-score = 0.44 
 for conf_thresh = 0.25, TP = 280, FP = 2, FN = 698, average IoU = 83.42 % 

 IoU threshold = 50 %, used Area-Under-Curve for each unique Recall 
 mean average precision (mAP@0.50) = 0.232763, or 23.28 % 
Total Detection Time: 3 Seconds
```

### 2.12 Inference with C++
```
# YOLOv4-tiny Video
./darknet detector demo VOCdevkit/voc.data VOCdevkit/yolov4-tiny.cfg backup/yolov4-tiny_last.weights /home/laitathei/Desktop/video_camera_color_image_raw.mp4 -out_filename /home/laitathei/Desktop/results1.mp4

# YOLOv4-tiny image
./darknet detector test ./cfg/coco.data cfg/yolov4-tiny.cfg yolov4-tiny.weights data/dog.jpg
```

### 2.13 Inference with Python
```
# YOLOv4-tiny Video
python3 darknet_video.py --input /home/laitathei/Desktop/video_camera_color_image_raw.mp4 --out_filename /home/laitathei/Desktop/results1.mp4 --weights backup/yolov4-tiny_last.weights --config_file VOCdevkit/yolov4-tiny.cfg --data_file VOCdevkit/voc.data

# YOLOv4-tiny Image
python3 darknet_images.py --input /home/laitathei/Desktop/darknet/data/dog.jpg --weights yolov4-tiny.weights --config_file VOCdevkit/yolov4-tiny.cfg --data_file cfg/coco.data
```

### 2.14 Inference with ROS, Realsense, Python
```
python3 inference_ros.py --weights backup/yolov4-tiny_last.weights --config_file VOCdevkit/yolov4-tiny.cfg --data_file VOCdevkit/voc.data
```

### 3. TensorRT conversion
### 3.1 Download dependency
```
sudo apt-get update
sudo apt-get install -y build-essential libatlas-base-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python3-pip
pip3 install numpy
pip3 install Cython
pip3 install pycuda --user
pip3 install onnx==1.4.1

git clone https://github.com/jkjung-avt/tensorrt_demos
```

### 3.2 Convert Darknet model to ONNX
```
cd /home/laitathei/Desktop/tensorrt_demos/plugins
make
cd /home/laitathei/Desktop/tensorrt_demos/yolo
cp /home/laitathei/Desktop/darknet/backup/yolov4-tiny_last.weights /home/laitathei/Desktop/tensorrt_demos/yolo/yolov4-tiny_last.weights
cp /home/laitathei/Desktop//darknet/VOCdevkit/yolov4-tiny.cfg /home/laitathei/Desktop/tensorrt_demos/yolo/yolov4-tiny_last.cfg
python3 yolo_to_onnx.py -m yolov4-tiny_last

## Below content will show if program success
Checking ONNX model...
Saving ONNX file...
Done.
```

### 3.3 Convert ONNX to TensorRT
```
python3 onnx_to_tensorrt.py -m yolov4-tiny_last

## Below content will show if program success
Completed creating engine.
Serialized the TensorRT engine to file: yolov4-tiny_last.trt
```

### 3.4 Inference with Python
```
# YOLOv4-tiny webcam
python3 trt_yolo.py --usb 0 -m yolov4-tiny_last
```

### 3.5 Inference with ROS, Realsense, Python
```
# YOLOv4-tiny
python3 inference_ros_trt.py -m yolov4-tiny_last -c 3
```
