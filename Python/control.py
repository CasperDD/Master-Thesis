import pigpio
import time
import math


class controller:
    pi = pigpio.pi()
    frequency = 1000
    range = 255
    pins = (5, 6, 12, 13, 16, 26, 17, 18, 22, 23)
    n = 6
    datalog = []
    timer = time.time()
    minSpeed = 200
    maxSpeed = 255
    speed = 180


    def __init__(self):
        self.pi.set_PWM_range(self.pins[2], self.range)
        self.pi.set_PWM_range(self.pins[3], self.range)

        self.pi.set_PWM_frequency(self.pins[2], self.frequency)
        self.pi.set_PWM_frequency(self.pins[3], self.frequency)


    def setTicsToRotate(self, pwm):
        """ Depending on the pwm, set the amount of tics that is required for one rotation of the wheel """
        return -0.74 * pwm + 600.15
    

    def setTics180(self, pwm):
        """ Depending on the pwm, set the amount of tics for 180 degree turn"""
        return -0.36 * pwm + 757.73
    

    def initGPIOPins(self):
        """ Initialize gpio pins """
        for i in self.pins[:self.n]:
            self.pi.set_mode(i, 1)
        
        for i in self.pins[:self.n]:
            self.pi.set_mode(i, 0)


    def setLeftMotor(self, speed, dir):
        if dir > 0:  # Forward
            self.pi.write(self.pins[4], 1)
            self.pi.write(self.pins[5], 0)

        if dir <= 0:  # Backward
            self.pi.write(self.pins[4], 0)
            self.pi.write(self.pins[5], 1)

        self.pi.set_PWM_dutycycle(self.pins[2], speed)


    def setRightMotor(self, speed, dir):
        if dir > 0:  # Forward
            self.pi.write(self.pins[0], 1)
            self.pi.write(self.pins[1], 0)

        if dir <= 0:  # Backward
            self.pi.write(self.pins[0], 0)
            self.pi.write(self.pins[1], 1)
        
        self.pi.set_PWM_dutycycle(self.pins[3], speed)


    def get_encode_values(self):
        left_A = self.pi.read(self.pins[8])
        left_B = self.pi.read(self.pins[9])
        right_A = self.pi.read(self.pins[6])
        right_B = self.pi.read(self.pins[7])

        temp = []
        temp.append(left_A)
        temp.append(left_B)
        temp.append(right_A)
        temp.append(right_B)

        # print('Left: ', left_A, left_B)
        # print('Right: ', right_A, right_B)

        return temp


    def setMotorSpeedDirection(self, SpeedL, SpeedR, dirL, dirR, log):
        self.setLeftMotor(SpeedL, dirL)
        self.setRightMotor(SpeedR, dirR)

        if log == True:
            self.logging(SpeedL, SpeedR, dirL, dirR)
        


    def logging(self, SpeedL, SpeedR, dirL, dirR):
        temp = []
        temp.append(SpeedL)
        temp.append(SpeedR)
        temp.append(dirL)
        temp.append(dirR)
        if len(self.datalog) < 1:
            self.timer = time.time()
        else:
            end = self.timer
            self.timer = time.time()
            time_elapsed = self.timer - end
            self.datalog[len(self.datalog) - 1].append(time_elapsed)
        
        self.datalog.append(temp)
    
    
    def get_logging(self):
        return self.datalog
    

    def goStraight(self, tics_to_go):
        tics_l = 0
        tics_r = 0
        pwm_factor = 1  # This value need to be adjusted and tested
        current_encoder = self.get_encode_values()

        while tics_l <= tics_to_go and tics_r <= tics_to_go:
            temp = self.get_encode_values()

            if temp[0] != current_encoder[0] or temp[1] != current_encoder[1]:
                tics_l += 1

            if temp[2] != current_encoder[2] or temp[3] != current_encoder[3]:
                tics_r += 1

            current_encoder = temp

            if tics_r <= tics_to_go: 
                diff_r = 0
                
                if tics_r < tics_l:
                    diff_r = (tics_l - tics_r) * pwm_factor

                self.setRightMotor(self.speed + diff_r, 1)
            
            else:
                self.setRightMotor(0, 1)
                self.setLeftMotor(0, 1)
                tics_l = tics_to_go + 1

            if tics_l <= tics_to_go:
                diff_l = 0
                
                if tics_l < tics_r:
                    diff_l = (tics_r - tics_l) * pwm_factor

                self.setLeftMotor(self.speed + diff_l, 1)
            
            else:
                self.setRightMotor(0, 1)
                self.setLeftMotor(0, 1)
                tics_r = tics_to_go + 1

    
    def turn(self, theta):
        tics_l = 0
        tics_r = 0
        tics_turn = self.setTics180(self.speed)

        tics_to_rotate = (tics_turn / math.pi) * (abs(theta))
        # print("Tics to rotate: ", tics_to_rotate)

        current_encoder = self.get_encode_values()

        while tics_l <= tics_to_rotate and tics_r <= tics_to_rotate:
            temp = self.get_encode_values()

            if temp[0] != current_encoder[0] or temp[1] != current_encoder[1]:
                tics_l += 1

            if temp[2] != current_encoder[2] or temp[3] != current_encoder[3]:
                tics_r += 1

            current_encoder = temp

            if theta < 0:
                if tics_r <= tics_to_rotate:
                    self.setRightMotor(self.speed, 1)
                else:
                    self.setRightMotor(0, 1)

                if tics_l <= tics_to_rotate:
                    self.setLeftMotor(self.speed, 0)
                else:
                    self.setLeftMotor(0, 0)
            else:
                if tics_r <= tics_to_rotate:
                    self.setRightMotor(self.speed, 0)
                else:
                    self.setRightMotor(0, 0)

                if tics_l <= tics_to_rotate:
                    self.setLeftMotor(self.speed, 1)
                else:
                    self.setLeftMotor(0, 1)
    
    
    def stopMotor(self):
        self.setLeftMotor(0, 0)
        self.setRightMotor(0, 0)
        self.pi.stop() 