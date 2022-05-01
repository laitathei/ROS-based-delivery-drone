#!/usr/bin/env python3
import math
import rospy
import threading
import tkinter as tk
import tkinter.font as tkfont
from PIL import ImageTk
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
import PIL.Image
import rospkg
from ros_circuitpython_servokit_msgs.msg import AllServoAngle

rospack = rospkg.RosPack()
rospack.list()
pkg_path = rospack.get_path('control_gui')
icon_path = pkg_path+"/Icon"

class control_node(object):
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.ps = PoseStamped()

        self.angle = AllServoAngle()

        self.ALT_SP = 0
        # update the setpoint message with the required altitude
        self.ps.pose.position.z = self.ALT_SP
  
    def gui(self):
        root = tk.Tk()
        root.title("Control GUI")

        #title 
        titlef = tk.Frame(root)
        menubar = tk.Menu(root)
        menuList = tk.Menu(menubar, tearoff=0)
        menuList.add_command(label="Exit", command=lambda:root.destroy())
        menubar.add_cascade(label="Menu", menu=menuList)
        root.config(menu=menubar)

        #title style
        title_font = tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")

        #title body
        title = tk.Label(titlef, text="Control GUI", font=title_font, anchor="e" )
        title.grid(row=0, column=1,sticky="")
        titlef.grid(row=0, column=0, columnspan=5,sticky="")

        row1 = tk.LabelFrame(root, width=200)
        row1.grid(row=2, column=0, columnspan=1,sticky="")

        ModeLF = tk.LabelFrame(row1, width=200)
        ModeLF.grid(row=0, column=0, columnspan=1,sticky="W")

        #Control modes
        conMode = tk.LabelFrame(ModeLF, text="Control Modes", width=200)
        conMode.grid(row=0, column=0, columnspan=1,sticky="W")

        offbt = tk.Button(conMode, text="Manual", width=10,bd=2, cursor="exchange", command = lambda: self.manual())
        offbt.grid(row=0, column=1, columnspan=1)

        autobt = tk.Button(conMode, text="Auto", width=10,bd=2, cursor="exchange", command = lambda: self.auto())
        autobt.grid(row=0, column=0, columnspan=1)

        self.initbt = tk.Button(conMode, text="Reinitate", width=10, bd=2, cursor="exchange", command = lambda: self.reinit())
        self.initbt.grid(row=0, column=2, columnspan=1)

        self.posebt = tk.Button(conMode, text="Pose", width=10, bd=2, cursor="exchange", command = lambda: self.pose())
        self.posebt.grid(row=0, column=3, columnspan=1)

        
        #manualState
        manualSate = tk.LabelFrame(ModeLF, text="manualControl", width=200)
        manualSate.grid(row=1, column=0, columnspan=1,sticky="W")

        droneCon = tk.LabelFrame(manualSate, text="DroneControl", width=200)
        droneCon.grid(row=0, column=0, columnspan=1,sticky="W")

        gripCon = tk.LabelFrame(manualSate, text="GripperControl", width=200)
        gripCon.grid(row=0, column=1, columnspan=1,sticky="W")

        timg = PIL.Image.open(icon_path+'/Takeoff.jpg')
        tics = timg.resize((30, 30))
        takeic = ImageTk.PhotoImage(tics)
        self.takeoffbt = tk.Button(droneCon,image = takeic, text="Takeoff", width=100,bd=2, compound="left", cursor="exchange", command = lambda: self.takeoff())
        self.takeoffbt.pack()
        self.takeoffbt.grid(row=0, column=0, columnspan=1)
        
        limg = PIL.Image.open(icon_path+'/Land.jpg')
        lics = limg.resize((30, 30))
        landic = ImageTk.PhotoImage(lics)
        landbt = tk.Button(droneCon, text="Landing",image = landic, width=100,bd=2, compound="left", cursor="exchange", command = lambda: self.landing())
        landbt.grid(row=0, column=1, columnspan=1)

        fimg = PIL.Image.open(icon_path+'/For.jpg')
        fics = fimg.resize((30, 30))
        foric = ImageTk.PhotoImage(fics)
        self.forwardbt = tk.Button(droneCon, text="Forword", image = foric, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.y_forward(),state="disabled")
        self.forwardbt.grid(row=1, column=0, columnspan=1)

        bimg = PIL.Image.open(icon_path+'/Back.jpg')
        bics = bimg.resize((30, 30))
        backic = ImageTk.PhotoImage(bics)
        self.backwardbt = tk.Button(droneCon, text="Backward", image = backic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.y_back(),state="disabled")
        self.backwardbt.grid(row=1, column=1, columnspan=1)

        rimg = PIL.Image.open(icon_path+'/right.jpg')
        rics = rimg.resize((30, 30))
        rightic = ImageTk.PhotoImage(rics)
        self.rightbt = tk.Button(droneCon, text="Right", image = rightic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.x_right(),state="disabled")
        self.rightbt.grid(row=2, column=0, columnspan=1)

        leimg = PIL.Image.open(icon_path+'/left.jpg')
        leics = leimg.resize((30, 30))
        leftic = ImageTk.PhotoImage(leics)
        self.leftbt = tk.Button(droneCon, text="Left", image = leftic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.x_left(),state="disabled")
        self.leftbt.grid(row=2, column=1, columnspan=1)

        simg = PIL.Image.open(icon_path+'/hold.jpg')
        sics = simg.resize((30, 30))
        stopic = ImageTk.PhotoImage(sics)
        self.stopbt = tk.Button(droneCon, text="Hold", image = stopic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.hold(),state="disabled")
        self.stopbt.grid(row=5, column=0, columnspan=1)

        eimg = PIL.Image.open(icon_path+'/exit.jpg')
        eics = eimg.resize((30, 30))
        exitic = ImageTk.PhotoImage(eics)
        self.exitbt = tk.Button(droneCon, text="Exit", image = exitic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: root.destroy())
        self.exitbt.grid(row=5, column=1, columnspan=1)

        rrimg = PIL.Image.open(icon_path+'/rotate_right.png')
        rrics = rrimg.resize((30, 30))
        rric = ImageTk.PhotoImage(rrics)
        self.rrightbt = tk.Button(droneCon, text="Rotate right", image = rric, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.z_right(),state="disabled")
        self.rrightbt.grid(row=4, column=0, columnspan=1)

        rlimg = PIL.Image.open(icon_path+'/rotate_left.png')
        rlics = rlimg.resize((30, 30))
        rlic = ImageTk.PhotoImage(rlics)
        self.rleftbt = tk.Button(droneCon, text="Rotate left", image = rlic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.z_left(),state="disabled")
        self.rleftbt.grid(row=4, column=1, columnspan=1)

        upimg = PIL.Image.open(icon_path+'/up.png')
        upics = upimg.resize((30, 30))
        upic = ImageTk.PhotoImage(upics)
        self.upbt = tk.Button(droneCon, text="Up", image = upic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.z_up(),state="disabled")
        self.upbt.grid(row=3, column=0, columnspan=1)

        downimg = PIL.Image.open(icon_path+'/down.jpg')
        downics = downimg.resize((30, 30))
        downic = ImageTk.PhotoImage(downics)
        self.downbt = tk.Button(droneCon, text="Down", image = downic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.z_down(),state="disabled")
        self.downbt.grid(row=3, column=1, columnspan=1)
        
        self.gripbt = tk.Button(gripCon, text="Grip", width=10, bd=2, cursor="exchange", command = lambda: self.grip())
        self.gripbt.grid(row=0, column=2, columnspan=1)

        self.dropbt = tk.Button(gripCon, text="Drop Crago", width=10, bd=2, cursor="exchange", command = lambda: self.drop())
        self.dropbt.grid(row=1, column=2, columnspan=1)


        # Label for show state
        self.var = tk.StringVar()
        label = tk.Label(root,textvariable= self.var, bg = "white" ,height = 2, width = 50, relief = "solid",cursor="exchange")
        label.grid(row=1, column=0, columnspan=5,sticky="W")
        
        # Label for show Position
        positionSate = tk.LabelFrame(ModeLF, text="Show Current Position", width=60)
        positionSate.grid(row=2, column=0, columnspan=1,sticky="W")
        posscroll = tk.Scrollbar(positionSate) 
        posscroll.pack(side = "right", fill = "y") 
        self.poslist = tk.Listbox(positionSate, width=51, height = 8, yscrollcommand = posscroll.set )  
        self.poslist.pack(side = "left", fill = "both")    
        posscroll.config(command = self.poslist.yview) 

        # Label for show Quaternion
        oriSate = tk.LabelFrame(root, text="Show Current Quaternion", width=85)
        oriSate.grid(row=3, column=0, columnspan=1,sticky="W")
        oriscroll = tk.Scrollbar(oriSate) 
        oriscroll.pack(side = "right", fill = "y") 
        self.orilist = tk.Listbox(oriSate, width=80,height = 8, yscrollcommand = oriscroll.set )  
        self.orilist.pack(side = "left", fill = "both")    
        oriscroll.config(command = self.orilist.yview) 

        # Set Position
        setposSate = tk.LabelFrame(root, text="Set Position", width=200)
        setposSate.grid(row=6, column=0, columnspan=5,sticky="W")
        xlabel = tk.Label(setposSate, text="x:")
        xlabel.grid(row=0, column=0, columnspan=1)
        self.xentry = tk.Entry(setposSate,state="disabled") 
        self.xentry.grid(row=0, column=1, columnspan=1)
        ylabel = tk.Label(setposSate, text="y:")
        ylabel.grid(row=0, column=2, columnspan=1)
        self.yentry = tk.Entry(setposSate,state="disabled") 
        self.yentry.grid(row=0, column=3, columnspan=1)
        zlabel = tk.Label(setposSate, text="z:")
        zlabel.grid(row=0, column=4, columnspan=1)
        self.zentry = tk.Entry(setposSate,state="disabled") 
        self.zentry.grid(row=0, column=5, columnspan=1)
        rzlabel = tk.Label(setposSate, text=" Rotation z:")
        rzlabel.grid(row=0, column=6, columnspan=1)
        self.rzentry = tk.Entry(setposSate,state="disabled") 
        self.rzentry.grid(row=0, column=7, columnspan=1)

        self.setposbt = tk.Button(setposSate, text="Set Position", width=10, bd=2, cursor="exchange", command = lambda: self.setpos(),state="disabled")
        self.setposbt.grid(row=1, column=3, columnspan=1)
        
        self.clearbt = tk.Button(setposSate, text="Clear", width=10, bd=2, cursor="exchange", command = lambda: self.clear(),state="disabled")
        self.clearbt.grid(row=1, column=1, columnspan=1)

        # Add image from detection
        poseimage = tk.LabelFrame(row1, text="Show the Pose", width=320, height = 480)
        poseimage.grid(row=0, column=2, columnspan=5,sticky="W")
        self.poseimg = tk.Canvas(poseimage, width = 320, height = 480)  
        self.poseimg.grid(row=0, column=0, columnspan=1)

        deteimage = tk.LabelFrame(row1, text="Show the Detection", width=640, height = 480)
        deteimage.grid(row=0, column=1, columnspan=1,sticky="W")
        self.detimg = tk.Canvas(deteimage, width = 640, height = 480)  
        self.detimg.grid(row=0, column=0, columnspan=1)

        self.setArm()
        k=0
        while k<10:
            self.sp_pub.publish(self.ps)
            k = k + 1
            print("init setpoint")

        # activate OFFBOARD mode
        self.setOffboardMode()
        self.first()
        root.mainloop()

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            print("offboard")
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)
	# Callbacks
    # local position callback

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)

    def pos_cb(self, msg):
        self.local_pos_x = round(msg.pose.position.x,1)
        self.local_pos_y = round(msg.pose.position.y,1)
        self.local_pos_z = round(msg.pose.position.z,1)
        self.local_quat_x = round(msg.pose.orientation.x,1)
        self.local_quat_y = round(msg.pose.orientation.y,1)
        self.local_quat_z = round(msg.pose.orientation.z,1)
        self.local_quat_w = round(msg.pose.orientation.w,1)

    # Drone State callback
    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self,data):
        
        try:
            self.cv_image = np.asarray(self.bridge.imgmsg_to_cv2(data, "8UC3"))
        except CvBridgeError as e:
            print(e)
        cv2.waitKey(1)

        blue,green,red = cv2.split(self.cv_image)
        self.cv_image = cv2.merge((red,green,blue))
        self.cvimg = PIL.Image.fromarray(self.cv_image)
        self.cvimg  = self.cvimg.resize((320, 480))

        self.cvimgtk = ImageTk.PhotoImage(image = self.cvimg)
        self.poseimg.create_image(20,20,anchor = "nw", image = self.cvimgtk)

    def detect_cb(self,data):
        try:
            self.dt_image = np.asarray(self.bridge.imgmsg_to_cv2(data, "8UC3"))
        except CvBridgeError as e:
            print(e)
        cv2.waitKey(1)

        blue,green,red = cv2.split(self.dt_image)
        self.dt_image = cv2.merge((red,green,blue))
        self.dtimg = PIL.Image.fromarray(self.dt_image)
        self.dtimg  = self.dtimg.resize((640, 480))
        
        self.dtimg = ImageTk.PhotoImage(image = self.dtimg)
        self.detimg.create_image(20,20, anchor = "nw", image = self.dtimg)

    def num_cb(self, data):
        data = data.data 
        
        if(self.modes == "Pose"):
            if(data == "1"):
                self.st = "F"
            elif(data == "2"):
                self.st = "L"
            elif(data == "3"):
                self.st = "L"
            elif(data == "4"):
                self.st = "R"
            elif(data == "5"):
                self.st = "B"
            elif(data == "6"):
                self.st = "F"
            elif(data == "7"):
                self.st = "R"
            elif(data == "8"):
                self.st = "B"
            else:
                self.st = "X"

    def movenum_cb(self, data):
        data = data.data.split(';')
        d = int(data[1])/1000
        z = self.local_pos_z
        distance = (d**2+(z-0.8)**2)**0.5
        if(data[0] == "1"):
            self.st="S"
        elif(data[0] == "0"):
            self.st = "X"
            if(distance > 1):
                self.st="F"
            elif(distance < 1):
                self.st = "Q"
        elif(data[0] == "-1"):
            self.st="A"
        else:
            self.st = "X"

    def hold(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("Hold")
        self.var.set("Hold the current position")
        self.st = "X"

    def takeoff(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = 0.8
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = 0
        self.ps.pose.orientation.w = self.local_quat_w
        print("takeoff")
        self.st="T"
        self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
        self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
        if (self.local_pos_z >= 0.8):
            self.var.set("Takeoff successfully")
        else:
            self.var.set("Takeoff to the hight of 0.8 meters")

    def x_right(self):
        self.ps.pose.position.x = self.local_pos_x + 0.2
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Move to the right")
        print("right")
        self.st="R"

    def x_left(self):
        self.ps.pose.position.x = self.local_pos_x - 0.2
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("left")
        self.var.set("Move to the left")
        self.st="L"

    def y_forward(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y + 0.2
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("forward")
        self.var.set("Move forward")
        self.st="F"

    def y_back(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y - 0.2  
        self.ps.pose.position.z = self.local_pos_z     
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Move backward")
        print("backward")
        self.st="B"

    def landing(self):
        self.setAutoLandMode()
        self.setDisarm()
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = 0
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Landing")
        print("landing")
        self.st="D"
        if (self.local_pos_z <= 0.4):
            self.st= "E"
            
            if not self.state.MODE_PX4_LAND:
                self.setAutoLandMode()
                self.setDisarm()
            else:
                print("Exit")
    
    def cargo(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = 0.5
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("Drop cargo")
        self.var.set("Drop the cargo")
        self.st= "O"

    def z_right(self):
        self.ps.pose.position.x = self.local_pos_x 
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z + 0.1
        self.ps.pose.orientation.w = self.local_quat_w
        print("rotate right")
        self.var.set("rotate right")
        self.st="A"

    def z_left(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z - 0.1
        self.ps.pose.orientation.w = self.local_quat_w
        print("rotate left")
        self.var.set("rotate left")
        self.st="S"

    def z_up(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = (self.local_pos_z + 0.2)
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("up")
        self.var.set("Move upward")
        self.st= "W"

    def z_down(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = (self.local_pos_z - 0.2)
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("Down")
        self.var.set("Move downward")
        self.st= "N"

    def reinit(self):
        self.st= "I"
        self.setArm()
        self.sp_pub.publish(self.ps)
        self.setOffboardMode()
        print("init setpoint")

    def grip(self):
        self.angle.all16servoPWM = [10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.servo_pub.publish(self.angle)
        print("Grip")
        self.var.set("Grip")
        self.st= "G"

    def drop(self):
        self.angle.all16servoPWM = [120, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.servo_pub.publish(self.angle)
        print("Drop Cargo")
        self.st= "Q"

    def setpos(self):
        setx = self.xentry.get()
        sety = self.yentry.get()
        setz = self.zentry.get()
        setrz = self.rzentry.get()
        if setx == "" or sety == "" or setz == "" or setrz == "":
            self.ps.pose.position.x = self.local_pos_x
            self.ps.pose.position.y = self.local_pos_y
            self.ps.pose.position.z = self.local_pos_z
            self.ps.pose.orientation.z = self.local_quat_z
        else:
            self.ps.pose.position.x = float(setx)
            self.ps.pose.position.y = float(sety)
            self.ps.pose.position.z = float(setz)
            self.ps.pose.orientation.z = float(setrz)
            
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Setting Position")
        self.st= "P"       

    def clear(self):
        self.xentry.delete(0, 'end')
        self.yentry.delete(0, 'end')
        self.zentry.delete(0, 'end')
        self.rzentry.delete(0, 'end')

    def auto(self):
        self.st= "U"
        self.modes = "auto"
        self.leftbt["state"] = "disabled"
        self.rightbt["state"] = "disabled"
        self.backwardbt["state"] = "disabled"  
        self.forwardbt["state"] = "disabled"
        self.downbt["state"] = "disabled"
        self.upbt["state"] = "disabled" 
        self.rrightbt["state"] = "disabled"
        self.rleftbt["state"] = "disabled"
        self.stopbt["state"] = "disabled"
        self.xentry["state"] = "disabled"
        self.yentry["state"] = "disabled"
        self.zentry["state"] = "disabled"
        self.rzentry["state"] = "disabled"
        self.setposbt["state"] = "disabled"
        self.clearbt["state"] = "disabled"
        self.var.set("Auto loop")

    def manual(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.modes = "manual"
        self.st = "M"  
        self.leftbt["state"] = "normal" 
        self.rightbt["state"] = "normal"  
        self.backwardbt["state"] = "normal"  
        self.forwardbt["state"] = "normal"  
        self.downbt["state"] = "normal"
        self.upbt["state"] = "normal"  
        self.rrightbt["state"] = "normal"
        self.rleftbt["state"] = "normal"  
        self.stopbt["state"] = "normal" 
        self.xentry["state"] = "normal"
        self.yentry["state"] = "normal"
        self.zentry["state"] = "normal"
        self.rzentry["state"] = "normal"
        self.setposbt["state"] = "normal"
        self.clearbt["state"] = "normal"

    def pose(self):
        self.modes="Pose"
        self.st = "H"
        self.leftbt["state"] = "disabled"
        self.rightbt["state"] = "disabled"
        self.backwardbt["state"] = "disabled"  
        self.forwardbt["state"] = "disabled"
        self.downbt["state"] = "disabled"
        self.upbt["state"] = "disabled" 
        self.rrightbt["state"] = "disabled"
        self.rleftbt["state"] = "disabled"
        self.stopbt["state"] = "disabled"
        self.xentry["state"] = "disabled"
        self.yentry["state"] = "disabled"
        self.zentry["state"] = "disabled"
        self.rzentry["state"] = "disabled"
        self.setposbt["state"] = "disabled"
        self.clearbt["state"] = "disabled"
        self.var.set("Pose Control mode")

    def first(self):
        self.st = "Y"
        self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
        self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
        
        if self.state.armed == True:
            self.var.set("Select a mode")
        else:
            self.var.set("Need Reinitiate")

    def main(self): 
        self.rate = rospy.Rate(20.0)
        # Subscribe to drone state
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        # Subscribe to drone's local position
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_cb)
        # Setpoint publisher  
        self.sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.servo_pub = rospy.Publisher('/servo/angle', AllServoAngle, queue_size=10)
        # Subscribe to the cv2 image and pose
        self.pose_subscriber = rospy.Subscriber('/detected_human',Image,self.pose_cb)
        self.detect_subscriber = rospy.Subscriber('/detection_result/image',Image,self.detect_cb)
        self.posnm_subscriber = rospy.Subscriber('/detected_human_gesture',String,self.num_cb)
        self.human_pos_subscriber = rospy.Subscriber('/detected_human_pos',String,self.movenum_cb)
        # un doc change
        self.automode_pub = rospy.Publisher("/auto_mode/status", Bool, queue_size=1)
        self.bridge = CvBridge()
        self.cv_image = None
        self.rate = rospy.Rate(20.0)
        self.modes="manual"
        self.st = "C"

        while not rospy.is_shutdown():
            rospy.loginfo("Mode: %s State: %s",self.modes, self.st)
            if self.st == "C":
                print("init")   
            elif self.st == "Y":
                self.first()             
            elif self.st == "T": #Takeoff
                self.takeoff()
                self.sp_pub.publish(self.ps)
            elif self.st == "D": #Landing
                self.landing()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z)) 
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "F": #forward
                self.y_forward()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z)) 
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "B": #Backward
                self.y_back()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z)) 
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "R": #right
                self.x_right()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "L": #left
                self.x_left()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps) 
            elif self.st == "X": #Hold
                self.hold()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "A":
                self.z_right()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "S":
                self.z_left()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "O":
                self.cargo()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "E":
                print("Exit")
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                if (self.state.armed == False):
                    self.var.set("Landing successfully and disarmed ")
                else:
                    self.var.set("Landing successfully")
            elif self.st == "W":
                self.z_up()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "N":
                self.z_down()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)     
            elif self.st == "I":
                self.reinit()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
                if (self.state.armed == True):
                    self.var.set("Select a mode")
                else:
                    self.var.set("Need Reinitiate") 
            elif self.st == "G":
                self.grip()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)  
            elif self.st == "P":
                self.setpos()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps) 
            elif self.st == "Q":
                self.drop ()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps) 
            elif self.st == "H":
                self.pose ()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps) 
            elif self.modes == "auto" and self.st == "U":
                self.auto()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                status = bool()
                status=True
                self.automode_pub.publish(status)
            elif self.modes == "manual" and self.st == "M":
                self.manual()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
                if (self.state.armed == True):
                    self.var.set("Change to Manual mode")
                else:
                    self.var.set("Need Reinitiate")

if __name__ == '__main__':
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True, disable_signals=True)
    my_node = control_node()
    t = threading.Thread(target = my_node.gui,daemon = True)
    t.start()
    my_node.main()

    
