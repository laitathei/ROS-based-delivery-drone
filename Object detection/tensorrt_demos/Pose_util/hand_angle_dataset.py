from Pose_util.Filter_data import Filter_data
import math
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
            
            smooth_left_front_arm_angle = self.left_front_arm_angle.smoothed_data(raw_left_front_arm_angle)
            smooth_left_arm_angle = self.left_arm_angle.smoothed_data(raw_left_arm_angle)
            smooth_right_front_arm_angle = self.right_front_arm_angle.smoothed_data(raw_right_front_arm_angle)
            smooth_right_arm_angle = self.right_arm_angle.smoothed_data(raw_right_arm_angle)
            smooth_left_shoulderToHand_angle = self.left_shoulderToHand_angle.smoothed_data(raw_left_shoulderToHand_angle)
            smooth_right_shoulderToHand_angle = self.right_shoulderToHand_angle.smoothed_data(raw_right_shoulderToHand_angle)

            # if(smooth_left_front_arm_angle > 0 and smooth_left_arm_angle > 0 or smooth_left_front_arm_angle < 0 and smooth_left_arm_angle < 0):
            #     left_straight = True if abs(smooth_left_arm_angle - smooth_left_front_arm_angle) < 10 else False
            # else: 
            #     left_straight = False

            # if(smooth_right_front_arm_angle > 0 and smooth_right_arm_angle > 0 or smooth_right_front_arm_angle < 0 and smooth_right_arm_angle < 0):
            #     right_straight = True if abs(smooth_right_arm_angle - smooth_right_front_arm_angle) < 10 else False
            # else:
            #     right_straight = False

            left_coeff,right_coeff, straight =  self.hand_straight(smooth_left_arm_angle,smooth_left_front_arm_angle,smooth_right_arm_angle,smooth_right_front_arm_angle)

            if straight:
                gesture = self.gesture_identify(smooth_left_shoulderToHand_angle,smooth_right_shoulderToHand_angle)
            else:
                gesture = -1


            return left_coeff,right_coeff,int(smooth_left_shoulderToHand_angle),int(smooth_right_shoulderToHand_angle), gesture
        else:
            return False,False,-200,-200,-1

    def hand_straight(self,left_angle1,left_angle2,right_angle1,right_angle2): 
        left_different = max(left_angle1, left_angle2) - min(left_angle1,left_angle2)
        right_different = max(right_angle1,right_angle2) - min(right_angle1,right_angle2)
        left_coeff = self.trapezium_fuzzy(left_different,0,10,20)
        right_coeff = self.trapezium_fuzzy(right_different,0,10,20)
        
        # print(left_angle1,left_angle2,right_angle1,right_angle2)

        straight = True if (left_coeff*right_coeff > 0.4) else False

        return left_coeff, right_coeff, straight

    def gesture_identify(self,left,right):
        post = None
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

    def depth_calculation(self,data,depth_image,x_offset = 0, y_offset = 0):
        if(len(data) >20):
            centre_x = int((data[11][1] + data[12][1] + data[23][1] + data[24][1])/4)
            centre_y = int((data[11][2] + data[12][2] + data[23][2] + data[24][2])/4)
            dimension = depth_image.shape
            if (centre_x+x_offset in range(0,dimension[1]) and centre_y+y_offset in range(0,dimension[0])):
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

    def regulation_box(self,box,x_max,y_max):
        left,top,right,bottom  = box[:4]
        top     = max(0, top)
        left    = max(0, left)
        bottom  = min(y_max, bottom)
        right   = min(x_max, right)
        return top, left, bottom, right
