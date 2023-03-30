from kinematics import Kinematics
from yolo import YOLO
from camera import Camera
from MobileNet import MobileNetSSD
from qr_code import QR

import numpy as np
import cv2
import time
import threading
from typing import List
from concurrent.futures import Future
import concurrent.futures


class learningWalk:

    def __init__(self):
        self.kinematic = Kinematics()
        self.yolo = YOLO()
        self.picam = Camera()
        self.mobilenet = MobileNetSSD()
        self.qr = QR()

        self.center = self.picam.width/2
        self.w = 0
        self.h = 0
        
        self.time_thread = False

        self.encoder_values = []
        self.push_back_encode = False

        self.learn_rate = 0.1
        self.pred_weight = 0
        self.reflex_weight = 1
        self.reflex_old = 0

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



    def x_coord_yolo(self):
        scene = self.picam.getImage()

        box_center = self.qr.run_qr(scene)
        
        x = None

        if len(box_center) > 0:
            x = box_center[0][0]
            self.w = box_center[0][2]
            self.h = box_center[0][3]

        return x

    #Make the vision part run in a seperate thread
    def searchGoal(self):
        self.log_thread = True
        encoder_thread = threading.Thread(target=self.loggingThread)
        encoder_thread.start()

        start = time.perf_counter()
        while True:
            x_coord = self.x_coord_yolo()

            end = time.perf_counter()
            time_elapsed = end - start
            print("Time elapsed: ", time_elapsed)
            if time_elapsed > 10:
                print("Time for pirouette")
                self.push_back_encode = True
                self.log_thread = False

                predict = self.updateDirVec()
                print("Init predict: ", predict)
                
                # self.log_thread = True

                self.kinematic.control.turn(predict)
                self.visionTurn(x_coord) 
                #check if it is still logging here
                #Need information about how many tics it turns
                #Also include ICO learning in this
                start = time.perf_counter()
            else:
                print("I did not do pirouette")
                if x_coord == None:
                    self.kinematic.control.goStraight(500) # Replace with search algorithm
                else:
                    if self.w > 200 or self.h > 200:
                        self.kinematic.control.setLeftMotor(0, 1)
                        self.kinematic.control.setRightMotor(0, 1)
                        self.goal_found = True
                    else:
                        self.visionTarget(x_coord)
                
            if self.goal_found == True:
                self.push_back_encode = True
                self.log_thread = False
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

print("Encoder tics vec: ", walk.encoder_tics)