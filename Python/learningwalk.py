from kinematics import Kinematics
from yolo import YOLO
from camera import Camera
from MobileNet import MobileNetSSD
# from qr_code import QR
from qr_code import QRCodeDetector

import numpy as np
import cv2
import time
import threading
from pyzbar import pyzbar
from typing import List
from concurrent.futures import Future
import concurrent.futures


class learningWalk:

    def __init__(self):
        self.kinematic = Kinematics()
        self.yolo = YOLO()
        self.picam = Camera()
        # self.qr = QR()
        self.qr_detector = QRCodeDetector()
        

        self.center = self.picam.width/2
        self.w = None
        self.h = None
        self.x = None
        
        self.log_thread = False
        self.vision_thread = False
        self.x_coord_thread = None

        self.encoder_values = []
        self.push_back_encode = False

        self.learn_rate = 0.1
        self.pred_weight = 0
        self.reflex_weight = 1
        self.reflex_old = 0
        self.ico_out = 0

        self.in_center = False
        self.goal_found = False

        self.encoder_tics = []
    
    def ticCount(self, tail, head):
        left_count, right_count = 0, 0
        for i in range(tail, head):
            if (self.encoder_values[i][0] != self.encoder_values[i - 1][0] or self.encoder_values[i][1] != self.encoder_values[i - 1][1]):
                
                left_count += 1
            
            if (self.encoder_values[i][2] != self.encoder_values[i - 1][2] or self.encoder_values[i][3] != self.encoder_values[i - 1][3]):
            
                right_count += 1
        
        temp = []
        temp.append(left_count)
        temp.append(right_count)

        return temp


    def loggingThread(self):
        async_vec: List[Future[List[int]]] = []
        head, tail = 1, 1
        # timer = time.time()
        start = time.perf_counter()
        # print("start time: ", start)
        while self.log_thread == True:
            print("Logging thread")
            self.encoder_values.append(self.kinematic.control.get_encode_values())
            if self.push_back_encode == True:

                tail = head
                head = len(self.encoder_values) 
                end = time.perf_counter()
                # print("start time: ", end)
                time_elapsed = end - start
                # end = timer
                # timer = time.time()
                # time_elapsed = timer - end
                self.kinematic.time_point.append(time_elapsed)
                
                async_vec.append(concurrent.futures.ThreadPoolExecutor().submit(self.ticCount, tail, head))
                self.push_back_encode = False

        for i in range(len(async_vec)):
            temp = async_vec[i].result()
            self.kinematic.left_encoder_tics.append(temp[0])
            self.kinematic.right_encoder_tics.append(temp[1])

        print("Left encoder tics: ", self.kinematic.left_encoder_tics)
        print("Right encoder tics: ", self.kinematic.right_encoder_tics)


    def ICO(self, predict, reflex):
        
        deri_reflex = reflex - self.reflex_old
        
        self.pred_weight += self.learn_rate * predict * deri_reflex
        output = self.pred_weight * predict + self.reflex_weight * reflex

        print("ICO output: ", output)

        self.reflex_old = reflex

        return output
    
    
    def updateDirVec(self):
        self.kinematic.positionDirection(self.kinematic.control.speed)
        dir_vec = self.kinematic.directionVector()
        theta = dir_vec[1]

        return theta


    def pirouette(self, theta):
        self.kinematic.control.turn(theta)
        
        x_coord = self.x_coord_yolo()

        while self.in_center == False:
            self.visionTurn(x_coord)

    
    def visionTurn(self, x):
        if x == None:
            self.kinematic.control.setLeftMotor(0, 1)
            self.kinematic.control.setRightMotor(0, 1)
            self.in_center = False
        elif x < self.center - 10:
            self.kinematic.control.setLeftMotor(self.kinematic.control.speed, 0)
            self.kinematic.control.setRightMotor(self.kinematic.control.speed, 1)
            self.in_center = False
        elif x > self.center + 10:
            self.kinematic.control.setLeftMotor(self.kinematic.control.speed, 1)
            self.kinematic.control.setRightMotor(self.kinematic.control.speed, 0)
            self.in_center = False
        else:
            self.kinematic.control.setLeftMotor(0, 1)
            self.kinematic.control.setRightMotor(0, 1)
            self.in_center = True
        
        # return in_center


    def visionTarget(self, x):
        if x < self.center - 10:
            self.kinematic.control.setLeftMotor(self.kinematic.control.minSpeed, 1)
            self.kinematic.control.setRightMotor(self.kinematic.control.maxSpeed, 1)
            self.in_center = False
        elif x > self.center + 10:
            self.kinematic.control.setLeftMotor(self.kinematic.control.maxSpeed, 1)
            self.kinematic.control.setRightMotor(self.kinematic.control.minSpeed, 1)
            self.in_center = False
        else:
            self.kinematic.control.setLeftMotor(self.kinematic.control.speed, 1)
            self.kinematic.control.setRightMotor(self.kinematic.control.speed, 1)



    def x_coord_object(self):
        while self.vision_thread:
            print("Qr code thread")
            scene = self.picam.getImage()

            box_center = self.qr.run_qr(scene)
            
            if len(box_center) > 0:
                self.x = box_center[0][0]
                self.w = box_center[0][2]
                self.h = box_center[0][3]
            else:
                self.x = None
                self.w = None
                self.h = None
        

    def detect_qr_code(image):
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect QR codes in the image
        barcodes = pyzbar.decode(gray)

        # Loop over detected QR codes
        for barcode in barcodes:
            # Extract QR code data
            data = barcode.data.decode("utf-8")

            # Print QR code data
            print("QR Code detected:", data)

            # Return QR code data
            return data

        # If no QR codes were detected, return None
        return None


    def qrCodeThread(self):
        cap = cv2.VideoCapture(0)  # 0 is used to select the default camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set the frame width to 640 pixels
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set the frame height to 480 pixels

        while self.log_thread:
            ret, frame = cap.read()
            if not ret:
                continue

            # Detect QR codes in the image
            qr_code_data = self.detect_qr_code(frame)

            # Update QR code data
            self.qr_code_data = qr_code_data


    #Make the vision part run in a seperate thread
    def searchGoal(self):
        qr_thread = threading.Thread(target=self.qrCodeThread)
        qr_thread.start()
        time.sleep(10)
        self.log_thread = True
        encoder_thread = threading.Thread(target=self.loggingThread)
        encoder_thread.start()

        
        start = time.perf_counter()
        while True:

            end = time.perf_counter()
            time_elapsed = end - start
            print("Time elapsed: ", time_elapsed)
            if time_elapsed > 10:
                time.sleep(5)
                print("Time for pirouette")
                self.push_back_encode = True
                self.log_thread = False

                predict = self.updateDirVec() + self.ico_out
                # print("Init predict: ", predict)

                self.log_thread = True

                self.kinematic.control.turn(predict)
                self.visionTurn(self.x)

                new_dir_vec = self.updateDirVec()
                reflex = new_dir_vec - predict

                self.ico_out = self.ICO(predict, reflex)


                # check if it is still logging here
                # Need information about how many tics it turns
                # Also include ICO learning in this
                start = time.perf_counter()
            else:
                # print("I did not do pirouette")
                if self.x == None:
                    self.kinematic.control.goStraight(500) # Replace with search algorithm
                else:
                    if self.w > 200 or self.h > 200:
                        self.kinematic.control.setLeftMotor(0, 1)
                        self.kinematic.control.setRightMotor(0, 1)
                        self.goal_found = True
                    else:
                        self.visionTarget(self.x)

            if self.goal_found == True:
                self.push_back_encode = True
                self.log_thread = False
                self.vision_thread = False
                time.sleep(0.1)
                encoder_thread.join()
                self.clear()

                print("I ended the while loop")

                break
       
    
    
    def clear(self):
        for i in range(len(self.kinematic.left_encoder_tics)):
            temp = []
            temp.append(self.kinematic.left_encoder_tics[i])
            temp.append(self.kinematic.right_encoder_tics[i])
            self.encoder_tics.append(temp)

        self.encoder_values.clear()
        self.kinematic.left_encoder_tics.clear()
        self.kinematic.right_encoder_tics.clear()
        self.kinematic.time_point.clear()



from learningwalk import learningWalk

walk = learningWalk()
walk.kinematic.control.initGPIOPins()

i = 0
while i < 5:
    walk.searchGoal()

    i+= 1

walk.kinematic.control.stopMotor()