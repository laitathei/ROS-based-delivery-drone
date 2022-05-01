#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String

class drone_control():

    def __init__(self):

        self.current_state = State()
        self.drone_position = PoseStamped()
        self.target_position = PoseStamped()

        self.target_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.drone_state_Subscriber = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.drone_position_Subscriber = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.current_location_callback)
        self.gesture_Subscriber = rospy.Subscriber('/detected_human_gesture',String,self.gesture_callback)


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

    def goto_initial_position(self):
        self.target_position.pose.position.x = 0
        self.target_position.pose.position.y = 0
        self.target_position.pose.position.z = 0.8
        self.target_position.pose.orientation.x = 0 
        self.target_position.pose.orientation.y = 0
        self.target_position.pose.orientation.z = 0
        self.target_position.pose.orientation.w = 0

    def pose_control(self,data):
        self.target_position = self.drone_position 
        if self.drone_position.pose.position.z >= 0.7:
            if data in (1,6):
                self.target_position.pose.position.y = 1
            elif data in (2,3):
                self.target_position.pose.position.x = -1
            elif data in (4,7):
                self.target_position.pose.position.x = 1
            elif data in (5,8):
                self.target_position.pose.position.y = -1


    def state_callback(self,data):
        self.current_state = data

    def current_location_callback(self,data):
        self.drone_position = data

    def gesture_callback(self,data):
        gesture = int(data.data)
        print(gesture)
        self.pose_control(gesture)

def main():
    rospy.init_node('Basic_control', anonymous=True)
    r = rospy.Rate(20)
    glo_drone_control = drone_control()
    glo_drone_control.setArm()
    glo_drone_control.setOffboardMode()
    glo_drone_control.goto_initial_position()

    while not rospy.is_shutdown():
        if glo_drone_control.current_state.armed == True:
            if glo_drone_control.target_position.pose.position.z < 0.8:
                glo_drone_control.target_position.pose.position.z = 0.8
            glo_drone_control.target_publisher.publish(glo_drone_control.target_position)
        else:
            glo_drone_control.setArm()



if __name__ == '__main__':
    main()