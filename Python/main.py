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




# import pigpio
# import time
# from control import controller

# control = controller()

# pi = pigpio.pi()
# frequency = 1000
# range = 255
# pins = (5, 6, 12, 13, 16, 26, 17, 18, 22, 23)

# pi.set_PWM_range(pins[2], range)
# pi.set_PWM_range(pins[3], range)

# pi.set_PWM_frequency(pins[2], frequency)
# pi.set_PWM_frequency(pins[3], frequency)

# count = 0

# right_A = pi.read(pins[8])
# right_B = pi.read(pins[9])

# prev_state = (right_A, right_B)

# control.setLeftMotor(0, 1)
# control.setRightMotor(0, 1)
   
# start = time.perf_counter()
# time_elapsed = 0

# while True:
#     start = time.perf_counter()
#     right_A_prev = right_A
#     right_B_prev = right_B
#     left_A = pi.read(pins[8])
#     left_B = pi.read(pins[9])
#     right_A = pi.read(pins[6])
#     right_B = pi.read(pins[7])

#     curr_state = (right_A, right_B)

#     prev_state = curr_state

#     end = time.perf_counter()
#     time_elapsed = end - start
#     print("Time elapsed: ", time_elapsed)





import pigpio
import time
import numpy as np
from control import controller

control = controller()

pi = pigpio.pi()
frequency = 1000
range = 255
pins = (5, 6, 12, 13, 16, 26, 17, 18, 22, 23)

pi.set_PWM_range(pins[2], range)
pi.set_PWM_range(pins[3], range)

pi.set_PWM_frequency(pins[2], frequency)
pi.set_PWM_frequency(pins[3], frequency)

count = 0

right_A = pi.read(pins[8])
right_B = pi.read(pins[9])

prev_state = (right_A, right_B)

control.setLeftMotor(200, 1)
control.setRightMotor(200, 1)
   
start = time.perf_counter()
time_elapsed = 0

# control.goStraight(1000)


while time_elapsed < 2.13: #True: #time_elapsed < 2.13: 
    # start = time.perf_counter()
    right_A_prev = right_A
    right_B_prev = right_B
    left_A = pi.read(pins[8])
    left_B = pi.read(pins[9])
    right_A = pi.read(pins[6])
    right_B = pi.read(pins[7])

    curr_state = (right_A, right_B) #, left_A, left_B
    print(curr_state)

    if prev_state == (0, 0):
        if curr_state == (0, 1):
            count += 1
        elif curr_state == (1, 0):
            count -= 1
    elif prev_state == (0, 1):
        if curr_state == (0, 0):
            count -= 1
        elif curr_state == (1, 1):
            count += 1
    elif prev_state == (1, 0):
        if curr_state == (1, 1):
            count -= 1
        elif curr_state == (0, 0):
            count += 1
    elif prev_state == (1, 1):
        if curr_state == (1, 0):
            count += 1
        elif curr_state == (0, 1):
            count -= 1

    prev_state = curr_state

    end = time.perf_counter()
    time_elapsed = end - start
    # print("Time elapsed: ", time_elapsed)
    # print("Count: ", count)
    # time.sleep(0.001)

control.setLeftMotor(0, 1)
control.setRightMotor(0, 1)



    # if (right_A_prev != right_A) or (right_B_prev != right_B):
    #     if (right_A_prev == 0 and right_A == 0) or (right_B_prev == 0 and right_B == 1):
    #         count += 1
    #     elif (right_A_prev == 0 and right_A == 1) or (right_B_prev == 1 and right_B == 1):
    #         count += 1
    #     elif (right_A_prev == 1 and right_A == 1) or (right_B_prev == 1 and right_B == 0):
    #         count += 1
    #     elif (right_A_prev == 1 and right_A == 0) or (right_B_prev == 0 and right_B == 0):
    #         count += 1
        
    #     elif (right_A_prev == 0 and right_A == 1) or (right_B_prev == 0 and right_B == 0):
    #         count += -1
    #     elif (right_A_prev == 1 and right_A == 1) or (right_B_prev == 0 and right_B == 1):
    #         count += -1
    #     elif (right_A_prev == 1 and right_A == 0) or (right_B_prev == 1 and right_B == 1):
    #         count += -1
    #     elif (right_A_prev == 0 and right_A == 0) or (right_B_prev == 1 and right_B == 0):
    #         count += -1




# import pigpio
# import time
# import sys
# import select  # Import select for cross-platform input detection
# import numpy as np
# from control import controller

# control = controller()

# pi = pigpio.pi()
# frequency = 1000
# range = 255
# pins = (5, 6, 12, 13, 16, 26, 17, 18, 22, 23)

# pi.set_PWM_range(pins[2], range)
# pi.set_PWM_range(pins[3], range)

# pi.set_PWM_frequency(pins[2], frequency)
# pi.set_PWM_frequency(pins[3], frequency)

# count = 0

# right_A = pi.read(pins[8])
# right_B = pi.read(pins[9])

# control.setLeftMotor(200, 1)
# control.setRightMotor(200, 1)
   
# time_elapsed_list = []  # List to store time elapsed values

# i = 0

# while i < 1000:
#     start = time.perf_counter()
#     temp = control.get_encode_values()

#     print("Encode values left: ", temp[0], temp[1])
#     # print("Encode values right: ", temp[2], temp[3])

#     end = time.perf_counter()
#     time_elapsed = end - start
#     # print("Time elapsed: ", time_elapsed)
    
#     time_elapsed_list.append(time_elapsed)  # Append time elapsed to the list
#     time.sleep(0.0005)

#     i += 1
#     # if select.select([sys.stdin], [], [], 0)[0]:  # Check if input is available
#     #     if sys.stdin.readline().strip() == 'q':  # Check if "q" is pressed
#     #         break

# control.setLeftMotor(0, 1)
# control.setRightMotor(0, 1)

# average_time_elapsed = sum(time_elapsed_list) / len(time_elapsed_list)  # Calculate average time elapsed
# print("Average time elapsed: ", average_time_elapsed)