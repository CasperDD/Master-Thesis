from control import controller
from pynput import keyboard


class rcCar:
    control = controller()
    forward = 1
    backward = 0
    
    # def __init__(self):
        

    def forwards(self):
        self.control.setLeftMotor(255, self.forward)
        self.control.setRightMotor(255, self.forward)

    def backwards(self):
        self.control.setLeftMotor(255, self.backward)
        self.control.setRightMotor(255, self.backward)

    def left(self):
        self.control.setLeftMotor(255, self.backward)
        self.control.setRightMotor(255, self.forward)

    def right(self):
        self.control.setLeftMotor(255, self.forward)
        self.control.setRightMotor(255, self.backward)

    def stop(self):
        self.control.setLeftMotor(0, self.forward)
        self.control.setRightMotor(0, self.backward)


    def on_press(self, key):
        try:
            print('alphanumeric key {0} pressed'.format(
                key.char))
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
                

    def on_release(self, key):
        if key == keyboard.KeyCode.from_char('w'):
            self.forwards()
        elif key == keyboard.KeyCode.from_char('s'):
            self.backwards()
        elif key == keyboard.KeyCode.from_char('a'):
            self.left()
        elif key == keyboard.KeyCode.from_char('d'):
            self.right()
        elif key == keyboard.KeyCode.from_char('q'):
            self.stop()
        elif key == keyboard.KeyCode.from_char('e'):
            self.stop()
            self.control.stopMotor()
            # Stop listener
            return False

    def key(self):
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()


# #Code to control the robot with wasd
# #Paste into main to run 
# from rccar import rcCar

# rc_car = rcCar()

# while True:
#     rc_car.key()
