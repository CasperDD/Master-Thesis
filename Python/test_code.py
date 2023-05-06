from kinematics import Kinematics
from camera import Camera
from myapriltag import AprilTagDetector

import numpy as np
import cv2
import time
import threading
import multiprocessing
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
import concurrent.futures


class learningWalk:

    def __init__(self):
        self.kinematic = Kinematics()
        self.april_tag_detector = AprilTagDetector()
        

        self.center = self.qr_code.camera.width/2
        self.w = None
        self.h = None
        self.x = None

        self.paused = False  # Add a new flag to track pause state

        self.encoder_values = []
        self.log_process = None
        self.push_back_encode = multiprocessing.Value('b', False)
        self.pause_event = multiprocessing.Event()
        
        self.log_thread = False
        self.vision_thread = False
        self.x_coord_thread = None
        self.encoder_process = None

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

        prev_state_left = (self.encoder_values[tail][0], self.encoder_values[tail][1])
        prev_state_right = (self.encoder_values[tail][2], self.encoder_values[tail][3])
        for i in range(tail, head):
            curr_state_left = (self.encoder_values[i][0], self.encoder_values[i][1])
            curr_state_right = (self.encoder_values[i][2], self.encoder_values[i][3])
            
            if prev_state_left == (0, 0):
                if curr_state_left == (0, 1):
                    left_count += 1
                elif curr_state_left == (1, 0):
                    left_count -= 1
            elif prev_state_left == (0, 1):
                if curr_state_left == (0, 0):
                    left_count -= 1
                elif curr_state_left == (1, 1):
                    left_count += 1
            elif prev_state_left == (1, 0):
                if curr_state_left == (1, 1):
                    left_count -= 1
                elif curr_state_left == (0, 0):
                    left_count += 1
            elif prev_state_left == (1, 1):
                if curr_state_left == (1, 0):
                    left_count += 1
                elif curr_state_left == (0, 1):
                    left_count -= 1

            prev_state_left = curr_state_left
            prev_state_right = curr_state_right
       
        temp = []
        temp.append(left_count)
        temp.append(right_count)
        return temp


    def loggingProcess(self):
        head, tail = 1, 1
        start = time.perf_counter()
        time_elapsed_list = []
        with ThreadPoolExecutor() as executor:
            while True:
                start = time.perf_counter()
                if not self.log_process.is_alive():
                    # time.sleep(0.1)  # Add a small delay to reduce CPU usage
                    continue

                if not self.pause_event.is_set():  # Only log encoder values if not paused
                    # print("Logging process")
                    self.encoder_values.append(self.kinematic.control.get_encode_values())
                    # print(self.kinematic.control.get_encode_values())
                    end = time.perf_counter()
                    time_elapsed = end - start
                    time_elapsed_list.append(time_elapsed)
                    average_time_elapsed = sum(time_elapsed_list) / len(time_elapsed_list)  # Calculate average time elapsed
                    print("Average time elapsed: ", average_time_elapsed)
                    if self.push_back_encode.value:
                        tail = head
                        head = len(self.encoder_values)
                        print("Head: ", head)
                        # print("encoder values: ", self.encoder_values)
                        end = time.perf_counter()
                        time_elapsed = end - start
                        self.kinematic.time_point.append(time_elapsed)
                        async_result = executor.submit(self.ticCount, tail, head)
                        temp = async_result.result()
                        print("Left async vec: ", temp[0])
                        print("Right async vec: ", temp[1])
                        self.kinematic.left_encoder_tics += temp[0]  # Accumulate left encoder tics
                        self.kinematic.right_encoder_tics += temp[1]  # Accumulate right encoder tics
                        print("Left : ", self.kinematic.left_encoder_tics)
                        print("Right : ", self.kinematic.right_encoder_tics)
                        self.push_back_encode.value = False
                        start = time.perf_counter()


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


    def aprilTagThread(self):
        time.sleep(1)  # wait for the thread to start

        while True:
            scene = self.qr_code.camera.getImage()
            self.april_tag_detector.detect(scene)

    
    def pause_logging_process(self):
        self.push_back_encode.value = True  # Set push_back_encode to True when pausing
        self.pause_event.set()

    def resume_logging_process(self):
        self.push_back_encode.value = False  # Reset push_back_encode to False when resuming
        self.pause_event.clear()

    def start_logging_process(self):
        self.log_process = multiprocessing.Process(target=self.loggingProcess)
        self.log_process.start()

    def stop_logging_process(self):
        self.log_process.terminate()
        self.log_process.join()



    def searchGoal(self):
        self.aprilTagThread()
        april_tag_thread = threading.Thread(target=self.aprilTagThread)
        april_tag_thread.start()
        self.start_logging_process()
        start = time.perf_counter()
        
        while True:
            self.kinematic.control.goStraight(1000)
            self.pause_logging_process()

            time.sleep(5)
            self.resume_logging_process()
  
    
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