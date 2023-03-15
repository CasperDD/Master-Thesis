import pigpio
import time


class controller:
    pi = pigpio.pi()
    frequency = 1000
    range = 255
    pins = (5, 6, 12, 13, 16, 26, 17, 18, 22, 23)
    n = 6
    datalog = []
    timer = time.time()


    def __init__(self):

        self.pi.set_PWM_range(self.pins[2], self.range)
        self.pi.set_PWM_range(self.pins[3], self.range)

        self.pi.set_PWM_frequency(self.pins[2], self.frequency)
        self.pi.set_PWM_frequency(self.pins[3], self.frequency)

    
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
    

    def stopMotor(self):
        self.pi.stop() 

