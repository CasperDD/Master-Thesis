from control import controller
import numpy as np

class Kinematics:
    control = controller()
    left_encoder_tics = []
    right_encoder_tics = []
    time_point = []
    l = 15 #distance between wheels 
    wheel_circumference = 18.85 #circumference of wheel
    x_y_theta = np.zeros(3)
    

    def setCenterOfWheelBase(self, pwm):
        return (self.l / self.wheel_circumference) * self.control.setTicsToRotate(pwm)
    

    def setRotationMatrix(self, i, pwm):     #Rotation matrix
        V_l = self.left_encoder_tics[i] / self.time_point[i]
        V_r = self.right_encoder_tics[i] / self.time_point[i]
        center_of_wheel_base = self.setCenterOfWheelBase(pwm)
        omega = (V_r - V_l) / center_of_wheel_base
        dt = self.time_point[i]

        rotation_matrix = np.array([[np.cos(omega * dt), -np.sin(omega * dt) , 0],
                            [np.sin(omega * dt), np.cos(omega * dt) , 0],
                            [0, 0 , 1]])
        
        return rotation_matrix
    

    def setTranslationVector(self, i, pwm):
        V_l = self.left_encoder_tics[i] / self.time_point[i]
        V_r = self.right_encoder_tics[i] / self.time_point[i]
        center_of_wheel_base = self.setCenterOfWheelBase(pwm)
        R = (center_of_wheel_base / 2) * ((V_l + V_r) / (V_r - V_l))

        if V_r - V_l == 0:
            R = 0
        
        ICC_x = self.x_y_theta[0] - R * np.sin(self.x_y_theta[2])
        ICC_y = self.x_y_theta[1] + R * np.cos(self.x_y_theta[2])

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

        ICC_x = self.x_y_theta[0] - R * np.sin(self.x_y_theta[2])
        ICC_y = self.x_y_theta[1] + R * np.cos(self.x_y_theta[2])

        ICC_vector = np.array([ICC_x, ICC_y, omega * dt])

        return ICC_vector
    

    def positionDirection(self, pwm):
        rotation_matrix = np.empty([3,3])
        translation = np.empty(3)
        ICC = np.empty(3)
        
        for i in range(len(self.left_encoder_tics)):
            rotation_matrix = self.setRotationMatrix(i, pwm)

            translation = self.setTranslationVector(i, pwm)
            # print("translatio: ", translation)

            ICC = self.setICCVector(i, pwm)

            self.x_y_theta = translation @ rotation_matrix + ICC
            # print("x y theta: ", self.x_y_theta)


    def directionVector(self):
        direction_vector = []
        # print("x y theta: ", self.x_y_theta)
        x_y_theta = np.array(self.x_y_theta)
        # print("x y theta: ", x_y_theta)
        tic_length = np.sqrt(np.power(x_y_theta[0], 2) + np.power(x_y_theta[1], 2))

        angle_to_origin = np.arcsin(x_y_theta[1] / tic_length)

        turn_angle = np.pi + (x_y_theta[2] - angle_to_origin)

        if turn_angle > np.pi:
            turn_angle = turn_angle - np.pi * 2

        direction_vector.append(tic_length)
        direction_vector.append(turn_angle)

        return direction_vector


        



    


