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


# from control import controller
# import time

# control = controller()
# start = time.perf_counter()
# speed = 170
# while True:
#     end = time.perf_counter()
#     time_elapsed = end - start
#     if time_elapsed > 0.3:
#         control.setLeftMotor(0, 1)
#         control.setRightMotor(0, 1)
#         time.sleep(2)
#         start = time.perf_counter()
#     else:
#         control.setLeftMotor(speed, 1)
#         control.setRightMotor(speed, 1)
    



# from rccar import rcCar

# rc_car = rcCar()

# while True:
#     rc_car.key()


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
# cont.goStraight(50)
# cont.setLeftMotor(0, 0)
# cont.setRightMotor(0, 0)
# time.sleep(5)
# cont.turn(4 * math.pi)
# cont.stopMotor()


# import threading
# import time

# def background_task():
#     while True:
#         print("Background task is running...")
#         time.sleep(1)

# if __name__ == '__main__':
#     # Create a thread for the background task
#     background_thread = threading.Thread(target=background_task)
#     # Start the thread
#     background_thread.start()

#     # Main code here
#     while True:
#         print("Main code is running...")
#         time.sleep(1)

# import cv2
# cap = cv2.VideoCapture(0, cv2.CAP_ANY)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# while True:
#     ret, frame = cap.read()
#     cv2.imshow("frame", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# cap.release()
# cv2.destroyAllWindows()


# import cv2
# import numpy as np
# from apriltag import Detector, DetectorOptions

# imagepath = 'test.jpg'
# image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

# options = DetectorOptions(families='tag36h11') # use tag36h11 tag family
# detector = Detector(options)
# detections = detector.detect(image)

# family_name = options.families[0] # get the first family name
# print('Using tag family:', family_name)

# for det in detections:
#     print('Detection ID:', det.tag_id)
#     print('Detection Hamming distance:', det.hamming)
#     print('Detection center:', det.center)
#     print('Detection corners:', det.corners)
#     print('Detection homography:', det.homography)
#     print('Detection decision margin:', det.decision_margin)

#     # Compute the dimensions
#     corner_pts = det.corners.astype(np.float32)
#     width = np.linalg.norm(corner_pts[0] - corner_pts[1])
#     height = np.linalg.norm(corner_pts[1] - corner_pts[2])

#     print('Detection dimensions:', (width, height))
#     print('Detection success probability:', det.goodness)
#     print('--------------------------')

