from control import controller
import numpy as np
import math

class Kinematics:
    control = controller()
    left_encoder_tics = []
    right_encoder_tics = []
    time_point = []
    l = 15 #distance between wheels 
    wheel_circumference = 18.85 #circumference of wheel
    x_y_theta = [0, 0, 0]
    

    def setCenterOfWheelBase(self, pwm):
        return (self.l / self.wheel_circumference) * self.setTicsToRotate(pwm)
    

    def setRotationMatrix(self, i, pwm):     #Rotation matrix
        V_l = self.left_encoder_tics[i] / self.time_point[i]
        V_r = self.right_encoder_tics[i] / self.time_point[i]
        center_of_wheel_base = self.setCenterOfWheelBase(pwm)
        omega = (V_r - V_l) / center_of_wheel_base
        dt = self.time_point[i]

        rotation_matrix = np.array([math.cos(omega * dt), -math.sin(omega * dt) , 0],
                                   [math.sin(omega * dt), math.cos(omega * dt) , 0],
                                   [0, 0 , 1])
        
        return rotation_matrix
    

    def setTranslationVector(self, i, pwm):
        V_l = self.left_encoder_tics[i] / self.time_point[i]
        V_r = self.right_encoder_tics[i] / self.time_point[i]
        center_of_wheel_base = self.setCenterOfWheelBase(pwm)
        R = (center_of_wheel_base / 2) * ((V_l + V_r) / (V_r - V_l))

        if V_r - V_l == 0:
            R = 0
        
        ICC_x = self.x_y_theta[0] - R * math.sin(self.x_y_theta[2])
        ICC_y = self.x_y_theta[1] + R * math.cos(self.x_y_theta[2])

        translation_vector = np.array([self.x_y_theta[0] - ICC_x, self.x_y_theta[1] - ICC_y, self.x_y_theta[2]])

        return translation_vector
    

    def setICCVector(self, i, pwm):
        V_l = self.left_encoder_tics[i] / self.time_point[i]
        V_r = self.right_encoder_tics[i] / self.time_point[i]
        center_of_wheel_base = self.setCenterOfWheelBase(pwm)
        omega = (V_r - V_l) / center_of_wheel_base
        dt = self.time_point[i]
        R = (center_of_wheel_base / 2) * ((V_l + V_r) / (V_r - V_l))
        
        if V_r - V_l == 0:
            R = 0

        ICC_x = self.x_y_theta[0] - R * math.sin(self.x_y_theta[2])
        ICC_y = self.x_y_theta[1] + R * math.cos(self.x_y_theta[2])

        ICC_vector = np.array([ICC_x, ICC_y, omega * dt])

        return ICC_vector
    

    def positionDirection(self, pwm):
        rotation_matrix = np.empty([3,3])
        translation = np.empty(3)
        ICC = np.empty(3)
        
        for i in self.left_encoder_tics:
            rotation_matrix = self.setRotationMatrix(i, pwm)

            translation = self.setTranslationVector(i, pwm)

            ICC = self.setICCVector(i, pwm)

            self.x_y_theta = rotation_matrix * translation + ICC


    def directionVector(self):
        direction_vector = []
        tic_length = math.sqrt(math.pow(self.x_y_theta[0], 2) + math.pow(self.x_y_theta[1], 2))

        angle_to_origin = math.atan2(self.x_y_theta[1], self.x_y_theta[0])

        if angle_to_origin < 0:
            angle_to_origin = angle_to_origin + math.pi * 2
        
        direction_vector.append(tic_length)
        direction_vector.append(angle_to_origin)

        return direction_vector


        



    


