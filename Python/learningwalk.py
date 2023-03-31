from kinematics import Kinematics
from yolo import YOLO
from camera import Camera
# from qr_code import QR
from qr_code import QRCodeDetector
from myapriltag import AprilTagDetector

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
        # self.picam = Camera()
        # self.qr = QR()
        self.qr_code = QRCodeDetector()
        self.april_tag_detector = AprilTagDetector()
        

        self.center = self.qr_code.camera.width/2
        self.w = None
        self.h = None
        self.x = None
        
        self.log_thread = False
        self.pause_event = threading.Event()
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
                if self.kinematic.control
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
        start = time.perf_counter()
        while True:
            if not self.log_thread:
                break
            # print("Logging thread")
            self.encoder_values.append(self.kinematic.control.get_encode_values())
            if self.push_back_encode:
                tail = head
                head = len(self.encoder_values) 
                end = time.perf_counter()
                time_elapsed = end - start
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
        # print("Predict: ", predict)
        # print("Reflex: ", reflex)
        
        deri_reflex = reflex - self.reflex_old
        # print("deri reflex: ", deri_reflex)
        
        self.pred_weight += self.learn_rate * predict * deri_reflex
        # print("pred weight: ", self.pred_weight)

        output = self.pred_weight * predict + self.reflex_weight * reflex

        print("ICO output: ", output)

        self.reflex_old = reflex

        return output
    
    
    def updateDirVec(self):
        self.kinematic.positionDirection(self.kinematic.control.speed)
        dir_vec = self.kinematic.directionVector()
        theta = dir_vec[1]

        print("Theta: ", theta)

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
        while self.vision_thread == True:
            scene = self.qr_code.camera.getImage()

            box_center = self.qr.run_qr(scene)
            
            if len(box_center) > 0:
                self.x = box_center[0][0]
                self.w = box_center[0][2]
                self.h = box_center[0][3]
            else:
                self.x = None
                self.w = None
                self.h = None


    def qrCodeThread(self):
        self.qr_code.start()
        while True:
            print("Qr code thread")
            start = time.perf_counter()
            scene = self.qr_code.camera.getImage()
            self.qr_code.detect_qr_codes(scene)
            end = time.perf_counter()
            time_elapsed = end - start
            # print("Time elapsed: ", time_elapsed)
    
    
    def aprilTagThread(self):
        # self.april_tag_detector.start()
        # print("April tag thread started")
        time.sleep(1)  # wait for the thread to start

        while True:
            # print("April tag thread")
            # start = time.perf_counter()
            scene = self.qr_code.camera.getImage()
            self.april_tag_detector.detect(scene)
            # end = time.perf_counter()
            # time_elapsed = end - start
            # print("Time elapsed: ", time_elapsed)


    
    def start_logging_thread(self):
        self.log_thread = True
        encoder_thread = threading.Thread(target=self.loggingThread)
        encoder_thread.start()
    
    def stop_logging_thread(self):
        self.log_thread = False
    
    def pause_logging_thread(self):
        self.push_back_encode = True
    
    def resume_logging_thread(self):
        self.push_back_encode = False


    #See if there is an easy way to get which direction you driving
    #Potentially change up the kinematics and odometry calculation
    #Also maybe change the loggingThread
    def searchGoal(self):
        # self.qrCodeThread()
        # self.aprilTagThread()
        april_tag_thread = threading.Thread(target=self.aprilTagThread)
        april_tag_thread.start()
        # self.vision_thread = True
        # vision_thread = threading.Thread(target=self.x_coord_object)
        # vision_thread.start()
        # print("Got here")
        # time.sleep(100)
        # self.log_thread = True
        # encoder_thread = threading.Thread(target=self.loggingThread)
        # encoder_thread.start()
        self.start_logging_thread()
        
        start = time.perf_counter()
        while True:

            end = time.perf_counter()
            time_elapsed = end - start
            print("Time elapsed: ", time_elapsed)
            if time_elapsed > 5:
                time.sleep(2)
                self.push_back_encode = True
                self.stop_logging_thread()
                time.sleep(1)

                predict = self.updateDirVec() + self.ico_out
                print("Init dir vec: ", predict)
                self.push_back_encode = False
                self.start_logging_thread()
                time.sleep(1)
                
                self.kinematic.control.turn(predict)
                self.visionTurn(self.x)
                self.push_back_encode = True
                self.stop_logging_thread()
                time.sleep(1)
                
                new_dir_vec = self.updateDirVec()
                print("New dir vec: ", new_dir_vec)
                reflex = new_dir_vec - predict
                self.ico_out = self.ICO(predict, reflex)
                self.kinematic.control.turn(new_dir_vec)
                
                self.push_back_encode = False
                self.start_logging_thread()
                time.sleep(1)
                
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
                # self.vision_thread = False
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