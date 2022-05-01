from pickle import TRUE
from Pose_util.Filter_data import Filter_data
import math
import numpy as np
class hand_angle_dataset:

    def __init__(self):
        self.left_front_arm_angle = Filter_data()
        self.right_front_arm_angle = Filter_data()
        self.left_arm_angle = Filter_data()
        self.right_arm_angle = Filter_data()
        self.left_shoulderToHand_angle = Filter_data()
        self.right_shoulderToHand_angle = Filter_data()
        self.depth_of_human = Filter_data()

    def cal_angle(self,data):
        e = 0.0000001
        if(len(data) > 20):
            raw_left_front_arm_angle = math.atan((data[15][2]-data[13][2])/((data[15][1]-data[13][1]) + e)) /math.pi * 180 
            raw_left_arm_angle = math.atan((data[13][2]-data[11][2])/((data[13][1]-data[11][1]) + e)) /math.pi * 180
            raw_right_front_arm_angle = math.atan((data[16][2]-data[14][2])/((data[16][1]-data[14][1]) + e)) /math.pi * 180
            raw_right_arm_angle = math.atan((data[14][2]-data[12][2])/((data[14][1]-data[12][1]) + e)) /math.pi * 180
            raw_left_shoulderToHand_angle = math.atan((data[15][2]-data[11][2])/((data[15][1]-data[11][1]) + e)) /math.pi * 180 
            raw_right_shoulderToHand_angle = math.atan((data[16][2]-data[12][2])/((data[16][1]-data[12][1]) + e)) /math.pi * 180

            if (data[15][2]-data[11][2] > 0) and (data[15][1]-data[11][1] < 0) and (raw_left_shoulderToHand_angle < 0):
                raw_left_shoulderToHand_angle = 89
            elif (data[15][2]-data[11][2] < 0) and (data[15][1]-data[11][1] < 0) and (raw_left_shoulderToHand_angle > 0):
                raw_left_shoulderToHand_angle = -89

            if (data[16][2]-data[12][2] > 0) and (data[16][1]-data[12][1] > 0) and (raw_right_shoulderToHand_angle > 0):
                raw_left_shoulderToHand_angle = -89
            elif (data[16][2]-data[12][2] < 0) and (data[16][1]-data[12][1] > 0) and (raw_right_shoulderToHand_angle < 0):
                raw_left_shoulderToHand_angle = 89
            
            smooth_left_front_arm_angle = self.left_front_arm_angle.smoothed_data(raw_left_front_arm_angle)
            smooth_left_arm_angle = self.left_arm_angle.smoothed_data(raw_left_arm_angle)
            smooth_right_front_arm_angle = self.right_front_arm_angle.smoothed_data(raw_right_front_arm_angle)
            smooth_right_arm_angle = self.right_arm_angle.smoothed_data(raw_right_arm_angle)
            smooth_left_shoulderToHand_angle = self.left_shoulderToHand_angle.smoothed_data(raw_left_shoulderToHand_angle)
            smooth_right_shoulderToHand_angle = self.right_shoulderToHand_angle.smoothed_data(raw_right_shoulderToHand_angle)

            left_coeff,right_coeff, straight =  self.hand_straight(smooth_left_arm_angle,smooth_left_front_arm_angle,smooth_right_arm_angle,smooth_right_front_arm_angle)

            if straight:
                gesture = self.gesture_identify(smooth_left_shoulderToHand_angle,smooth_right_shoulderToHand_angle)
            else:
                gesture = -1


            return left_coeff,right_coeff,int(smooth_left_shoulderToHand_angle),int(smooth_right_shoulderToHand_angle), gesture
        else:
            return False,False,-200,-200,-1

    def cal_angle_v2(self,data,box_height=0,box_width=0):
        e = 0.0000001
        if(len(data) > 20):
            if box_height == 0 and box_width == 0:
                box_height = (data[2][2] + data[5][2] - data[27][2] - data[28][2])//2
                box_width = (data[11][1] + data[15][1] - data[12][1] -data[16][1])//2

            raw_left_front_arm_angle = math.atan((data[15][2]-data[13][2])/((data[15][1]-data[13][1]) + e)) /math.pi * 180 
            raw_left_arm_angle = math.atan((data[13][2]-data[11][2])/((data[13][1]-data[11][1]) + e)) /math.pi * 180
            raw_right_front_arm_angle = math.atan((data[16][2]-data[14][2])/((data[16][1]-data[14][1]) + e)) /math.pi * 180
            raw_right_arm_angle = math.atan((data[14][2]-data[12][2])/((data[14][1]-data[12][1]) + e)) /math.pi * 180
            raw_left_shoulderToHand_angle = math.atan((data[15][2]-data[11][2])/((data[15][1]-data[11][1]) + e)) /math.pi * 180 
            raw_right_shoulderToHand_angle = math.atan((data[16][2]-data[12][2])/((data[16][1]-data[12][1]) + e)) /math.pi * 180

            if (data[15][2]-data[11][2] > 0) and (data[15][1]-data[11][1] < 0) and (raw_left_shoulderToHand_angle < 0):
                raw_left_shoulderToHand_angle = 89
            elif (data[15][2]-data[11][2] < 0) and (data[15][1]-data[11][1] < 0) and (raw_left_shoulderToHand_angle > 0):
                raw_left_shoulderToHand_angle = -89

            if (data[16][2]-data[12][2] > 0) and (data[16][1]-data[12][1] > 0) and (raw_right_shoulderToHand_angle > 0):
                raw_left_shoulderToHand_angle = -89
            elif (data[16][2]-data[12][2] < 0) and (data[16][1]-data[12][1] > 0) and (raw_right_shoulderToHand_angle < 0):
                raw_left_shoulderToHand_angle = 89
            
            smooth_left_front_arm_angle = self.left_front_arm_angle.smoothed_data(raw_left_front_arm_angle)
            smooth_left_arm_angle = self.left_arm_angle.smoothed_data(raw_left_arm_angle)
            smooth_right_front_arm_angle = self.right_front_arm_angle.smoothed_data(raw_right_front_arm_angle)
            smooth_right_arm_angle = self.right_arm_angle.smoothed_data(raw_right_arm_angle)
            smooth_left_shoulderToHand_angle = self.left_shoulderToHand_angle.smoothed_data(raw_left_shoulderToHand_angle)
            smooth_right_shoulderToHand_angle = self.right_shoulderToHand_angle.smoothed_data(raw_right_shoulderToHand_angle)

            gesture = self.skeleton_validationV2(data,smooth_left_shoulderToHand_angle,smooth_right_shoulderToHand_angle,smooth_left_arm_angle,smooth_left_front_arm_angle,smooth_right_arm_angle,smooth_right_front_arm_angle,box_height,box_width)

            return gesture
        
        else:
            return -1



    def skeleton_validation(self,data,overall_left,overall_right,left_angle1,left_angle2,right_angle1,right_angle2,box_height,box_width):

        left_different = max(left_angle1, left_angle2) - min(left_angle1,left_angle2)
        right_different = max(right_angle1,right_angle2) - min(right_angle1,right_angle2)
        left_coeff = self.trapezium_fuzzy(left_different,0,10,20)
        right_coeff = self.trapezium_fuzzy(right_different,0,10,20)
        
        straight = True if (left_coeff*right_coeff > 0.4) else False

        

        if straight:
            return (self.gesture_identify(overall_left,overall_right))
        else:
            return -1

    def skeleton_validationV2(self,data,overall_left,overall_right,left_angle1,left_angle2,right_angle1,right_angle2,box_height,box_width):

        left_different = max(left_angle1, left_angle2) - min(left_angle1,left_angle2)
        right_different = max(right_angle1,right_angle2) - min(right_angle1,right_angle2)

        left_coeff = self.trapezium_fuzzy(left_different,0,10,20)
        right_coeff = self.trapezium_fuzzy(right_different,0,10,20)

        shoulder_coeff = self.step_fuzzy(data[11][1]-data[12][1],box_width*0.1,box_width*0.2)
        hip_coeff = self.step_fuzzy(data[23][1]-data[24][1],box_width*0.05,box_width*0.1)
        left_body_coeff = self.step_fuzzy(data[23][2]-data[11][2],box_height*0.2,box_height*0.3)
        right_body_coeff = self.step_fuzzy(data[24][2]-data[12][2],box_height*0.2,box_height*0.3)
        left_leg_coeff = self.step_fuzzy(data[27][2]-data[23][2],box_height*0.2,box_height*0.4)
        right_leg_coeff = self.step_fuzzy(data[28][2]-data[24][2],box_height*0.2,box_height*0.4)

        # print(data[11][1]-data[12][1],data[23][1]-data[24][1],data[11][2]-data[23][2],data[12][2]-data[24][2],data[23][2]-data[27][2],data[24][2]-data[28][2])
        # print(shoulder_coeff,hip_coeff,left_body_coeff,right_body_coeff,left_leg_coeff,right_leg_coeff)

        stright_coeff = min(left_coeff,right_coeff)
        top_coeff = min(shoulder_coeff,hip_coeff,left_body_coeff,right_body_coeff)
        bottom_coeff = min(left_leg_coeff,right_leg_coeff)

        first_statement = min((1-stright_coeff),(1-top_coeff),(1-bottom_coeff))
        second_statement = min(stright_coeff,top_coeff)
        third_statement = min(stright_coeff,top_coeff,bottom_coeff)
        
        print(first_statement,second_statement,third_statement)

        first_centroid = self.trapezium_centroid(20*first_statement,70*(1-first_statement),70) - 20
        second_centroid = self.trapezium_centroid(50*second_statement,100*(1-second_statement),100)
        third_centroid = self.trapezium_centroid(50*third_statement,70*(1-third_statement),70) + 50

        final_centroid = (first_centroid + second_centroid + third_centroid)/3
        print(first_centroid,second_centroid,third_centroid,final_centroid)
        if final_centroid > 50:
            return (self.gesture_identify(overall_left,overall_right))
        else:
            return -1   


    def hand_straight(self,left_angle1,left_angle2,right_angle1,right_angle2): 
        left_different = max(left_angle1, left_angle2) - min(left_angle1,left_angle2)
        right_different = max(right_angle1,right_angle2) - min(right_angle1,right_angle2)
        left_coeff = self.trapezium_fuzzy(left_different,0,10,20)
        right_coeff = self.trapezium_fuzzy(right_different,0,10,20)
        
        # print(left_angle1,left_angle2,right_angle1,right_angle2)

        straight = True if (left_coeff*right_coeff > 0.4) else False

        return left_coeff, right_coeff, straight

    def gesture_identify(self,left,right):
        post = -1
        if right > 40 and right < 90:
            if left > -90 and left < -40:
                post = '1'
            elif left < 30:
                post = '2'
            elif left < 90:
                post = '3'     
        elif right > -30:
            if left >-90 and left < -40:
                post = '4'
            elif left < 30:
                post = '5'
            elif left < 90:
                post = '6'   
        
        elif right > -90:
            if left >-90 and left < -40:
                post = '7'
            elif left < 30:
                post = '8'
            elif left < 90:
                post = '9' 

        return post
    
    def lay_down_check(self,left_shoulder,right_shoulder,left_hip,right_hip,depth1,depth2,depth3,depth4):
        lay = False
        if min(left_shoulder/depth1,right_shoulder/depth2) > 0.6:
            lay = True
        if min(left_hip/depth3,right_hip/depth4) > 0.4:
            lay = True

        return lay

    def depth_calculation(self,data,depth_image,x_offset = 0, y_offset = 0):
        if(len(data) >20):
            centre_x = int((data[11][1] + data[12][1] + data[23][1] + data[24][1])/4)
            centre_y = int((data[11][2] + data[12][2] + data[23][2] + data[24][2])/4)
            dimension = depth_image.shape
            if (centre_x in range(0,dimension[1]) and centre_y in range(0,dimension[0])):
                return self.depth_of_human.smoothed_data(depth_image[centre_y+y_offset][centre_x+x_offset]),centre_x,centre_y
            else:
                return -100,-1,-1   
        else:
            return -100,-1,-1

    def trapezium_fuzzy(self,value, centre, upper_length,lower_length):

        if (lower_length <= upper_length):
            return -1
        
        diff_to_centre = abs(value - centre)

        if (diff_to_centre < upper_length):
            return 1
        elif(diff_to_centre < lower_length):
            return (diff_to_centre-lower_length)/(upper_length - lower_length)
        else:
            return 0

    def step_fuzzy(self,value,low_limit,upper_limit):

        if value < low_limit:
            return 0
        elif value > upper_limit:
            return 1
        else:
            return 1/(upper_limit - low_limit)*(value - low_limit)

    def fall_step_fuzzy(self,value,low_limit,upper_limit):

        if value < low_limit:
            return 1
        elif value > upper_limit:
            return 0
        else:
            return 1/(low_limit - upper_limit)*(value - low_limit)

    def trapezium_centroid(self,c,a,b):

        return (2*a*c + a**2 + c*b + a*b + b**2)/(3*(a+b))

    def regulation_box(self,box,x_max,y_max):
        top, left, bottom, right = box[:4]
        top     = max(0, top)
        left    = max(0, left)
        bottom  = min(y_max, bottom)
        right   = min(x_max, right)
        return top, left, bottom, right

    def regulation_box_v2(self,box,x_max,y_max):
        left,top,right,bottom = np.array(box[:4],dtype='int32')
        top     = max(0, top)
        left    = max(0, left)
        bottom  = min(y_max, bottom)
        right   = min(x_max, right)
        return left,top,right,bottom
