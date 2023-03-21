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
# video.getVideo()


from rccar import rcCar

rc_car = rcCar()

while True:
    rc_car.key()


# from control import controller
# import time

# duration = 5
# start_time = time.time()

# control = controller()
# control.initGPIOPins()
# control.setMotorSpeedDirection(200, 200, 1, 1, True)
# time.sleep(duration)

# control.setMotorSpeedDirection(0, 0, 1, 1, True)
# control.stopMotor()
# logged_data = control.get_logging()
# print("Done")

# print(logged_data)

# file = open("Test_logging", "w")

# for i in logged_data:
#     file.write(str(i) + "\n")

# file.close()


# from control import controller

# control = controller()
# control.initGPIOPins()

# last_run = control.get_encode_values()
# tics_r = 0
# tics_to_go = 452 # 452 for 200 - 412 for 255 - 432 for 227 // one rotation for different speeds

# while tics_r <= tics_to_go:
#     temp = control.get_encode_values()

#     if temp[2] != last_run[2] or temp[3] != last_run[3]:
#         tics_r += 1
#         print("Right: ", temp[2], temp[3])
    
#     last_run = temp

#     if tics_r <= tics_to_go:
#         control.setLeftMotor(200, 1)
#         control.setRightMotor(200, 1)
#     else:
#         control.setLeftMotor(0, 1)
#         control.setRightMotor(0,1)

# print("Done")

# from learningwalk import learningWalk
# import math

# test = learningWalk()
# test.turn(math.pi)

# test.kinematic.control.setLeftMotor(0, 1)
# test.kinematic.control.setRightMotor(0, 1)
# test.kinematic.control.stopMotor()


# from yolo import YOLO
# from camera import Camera 
# import cv2

# picam = Camera()
# object_det = YOLO()

# i = 0

# # image = picam.getImage()


# while True:

#     image = picam.getImage()
#     det_obj = object_det.run_yolo(image)
#     # cv2.imshow("Image", image)

#     print(det_obj)
#     # i += 1

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# from control import controller
# import time
# import math

# cont = controller()
# cont.goStraight(5000)
# cont.setLeftMotor(0, 0)
# cont.setRightMotor(0, 0)
# time.sleep(5)
# cont.turn(4 * math.pi)
# cont.stopMotor()

