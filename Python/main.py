# import time
# from control import controller


# forward = 1
# backward = 0

# frequency = 1000
# range = 255

# pins = (5, 6, 12, 13, 16, 26)

# control = controller(frequency, range, pins)


# print("before")
# control.setLeftMotor(200, forward)
# control.setRightMotor(200, backward)

# time.sleep(5)
# print("after")

# control.setLeftMotor(0, forward)
# control.setRightMotor(0, backward)

# control.stopMotor()

# from camera import camera

# video = camera()
# video.setup()
# video.getImage()
# # video.getVideo()


from rccar import rcCar

rc_car = rcCar()

while True:
    rc_car.key()