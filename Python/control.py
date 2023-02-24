import pigpio


class controller:
    pi = pigpio.pi()
    
    def __init__(self):
        pins = (5, 6, 12, 13, 16, 26)
        frequency = 1000
        range = 255

        self.pi.set_PWM_range(pins[2], range)
        self.pi.set_PWM_range(pins[3], range)

        self.pi.set_PWM_frequency(pins[2], frequency)
        self.pi.set_PWM_frequency(pins[3], frequency)
        
        for i in pins:
            self.pi.set_mode(i, 1)


    def setLeftMotor(self, speed, dir):
        if dir > 0:  # Forward
            self.pi.write(16, 1)
            self.pi.write(26, 0)

        if dir <= 0:  # Backward
            self.pi.write(16, 0)
            self.pi.write(26, 1)

        self.pi.set_PWM_dutycycle(12, speed)

    def setRightMotor(self, speed, dir):
        if dir > 0:  # Forward
            self.pi.write(5, 1)
            self.pi.write(6, 0)

        if dir <= 0:  # Backward
            self.pi.write(5, 0)
            self.pi.write(6, 1)
        
        self.pi.set_PWM_dutycycle(13, speed)
    
    def stopMotor(self):
        self.pi.stop() 

