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
import multiprocessing
from pyzbar import pyzbar
from typing import List
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
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

        self.paused = False  # Add a new flag to track pause state

        self.encoder_values = []
        self.log_process = None
        self.push_back_encode = multiprocessing.Value('b', False)
        self.pause_event = multiprocessing.Event()
        # self.pause_lock = multiprocessing.Lock()
        # self.encoder_lock = multiprocessing.Lock()  # Use a Lock for thread-safe access to shared resources
        
        # self.log_thread = multiprocessing.Value('b', True)  # Use a boolean Value instead of a simple boolean variable for thread synchronization
        # self.push_back_encode = multiprocessing.Value('b', False)  # Use a boolean Value for push_back_encode

        self.log_thread = False
        # self.pause_event = threading.Event()
        self.vision_thread = False
        self.x_coord_thread = None
        self.encoder_process = None

        # self.encoder_values = []
        # self.push_back_encode = False
        # self.push_back_encode = multiprocessing.Value('b', False)
        # self.encoder_lock = multiprocessing.Lock()

        self.learn_rate = 0.1
        self.pred_weight = 0
        self.reflex_weight = 1
        self.reflex_old = 0
        self.ico_out = 0

        self.in_center = False
        self.goal_found = False

        self.encoder_tics = []
    
    
    def ticCount(self, tail, head):
        # print("In ticCount")
        left_count, right_count = 0, 0
        # with self.encoder_lock:
        # print("Tail, head: ", tail, head)
        prev_state_left = (self.encoder_values[tail][0], self.encoder_values[tail][1])
        prev_state_right = (self.encoder_values[tail][2], self.encoder_values[tail][3])
        for i in range(tail, head):
            curr_state_left = (self.encoder_values[i][0], self.encoder_values[i][1])
            curr_state_right = (self.encoder_values[i][2], self.encoder_values[i][3])
            
            # print(curr_state_left)
            
            if prev_state_left == (0, 0):
                if curr_state_left == (0, 1):
                    left_count += 1
                elif curr_state_left == (1, 0):
                    left_count -= 1
                # elif curr_state_left == (1, 1):
                #     left_count += 1
            elif prev_state_left == (0, 1):
                if curr_state_left == (0, 0):
                    left_count -= 1
                elif curr_state_left == (1, 1):
                    left_count += 1
                # elif curr_state_left == (1, 0):
                #     left_count += 1
            elif prev_state_left == (1, 0):
                if curr_state_left == (1, 1):
                    left_count -= 1
                elif curr_state_left == (0, 0):
                    left_count += 1
                # elif curr_state_left == (0, 1):
                #     left_count += 1
            elif prev_state_left == (1, 1):
                if curr_state_left == (1, 0):
                    left_count += 1
                elif curr_state_left == (0, 1):
                    left_count -= 1
                # elif curr_state_left == (0, 0):
                #     left_count += 1

            prev_state_left = curr_state_left
            prev_state_right = curr_state_right

                # if self.encoder_values[i - 1][0] == 0 and self.encoder_values[i][0] == 1:
                #     if self.encoder_values[i][1] == 0:
                #         left_count += 1
                #     else:
                #         left_count -= 1
                # elif self.encoder_values[i - 1][1] == 0 and self.encoder_values[i][1] == 1:
                #     if self.encoder_values[i][0] == 1:
                #         left_count += 1
                #     else:
                #         left_count -= 1

                # if self.encoder_values[i - 1][2] == 0 and self.encoder_values[i][2] == 1:
                #     if self.encoder_values[i][3] == 0:
                #         right_count -= 1
                #     else:
                #         right_count += 1
                # elif self.encoder_values[i - 1][3] == 0 and self.encoder_values[i][3] == 1:
                #     if self.encoder_values[i][2] == 1:
                #         right_count -= 1
                #     else:
                #         right_count += 1
            
            # for i in range(tail, head):
            #     # print("i, tail", i, tail)
            #     if (self.encoder_values[i][0] != self.encoder_values[i - 1][0] or self.encoder_values[i][1] != self.encoder_values[i - 1][1]):
            #         left_count += 1
                
            #     if (self.encoder_values[i][2] != self.encoder_values[i - 1][2] or self.encoder_values[i][3] != self.encoder_values[i - 1][3]):
                
            #         right_count += 1
       
        temp = []
        temp.append(left_count)
        temp.append(right_count)
        # print("End of ticCount")
        return temp


    # def loggingThread(self):
    #     async_vec: List[Future[List[int]]] = []
    #     head, tail = 1, 1
    #     start = time.perf_counter()
    #     while True:
    #         if not self.log_thread:
    #             break
    #         # print("Logging thread")
    #         self.encoder_values.append(self.kinematic.control.get_encode_values())
    #         if self.push_back_encode:
    #             tail = head
    #             head = len(self.encoder_values) 
    #             end = time.perf_counter()
    #             time_elapsed = end - start
    #             self.kinematic.time_point.append(time_elapsed)
    #             async_vec.append(concurrent.futures.ThreadPoolExecutor().submit(self.ticCount, tail, head))
    #             self.push_back_encode = False
        
    #     for i in range(len(async_vec)):
    #         temp = async_vec[i].result()
    #         print("Left async vec: ", temp[0])
    #         print("Right async vec: ", temp[1])
    #         self.kinematic.left_encoder_tics.append(temp[0])
    #         self.kinematic.right_encoder_tics.append(temp[1])
    
    #     print("Left encoder tics: ", self.kinematic.left_encoder_tics)
    #     print("Right encoder tics: ", self.kinematic.right_encoder_tics)

    # def loggingThread(self):
    #     head, tail = 1, 1
    #     start = time.perf_counter()
    #     while True:
    #         if not self.log_thread:
    #             time.sleep(0.1)  # Add a small delay to reduce CPU usage
    #             continue

    #         print("Logging thread: ", self.push_back_encode)
    #         self.encoder_values.append(self.kinematic.control.get_encode_values())
    #         if self.push_back_encode:
    #             print("in if statement")
    #             tail = head
    #             head = len(self.encoder_values) 
    #             end = time.perf_counter()
    #             time_elapsed = end - start
    #             self.kinematic.time_point.append(time_elapsed)
    #             print("Before ticCount")
    #             result = self.ticCount(tail, head)
    #             print("result: ", result)
    #             temp = result.get()
    #             print("Left async vec: ", temp[0])
    #             print("Right async vec: ", temp[1])
    #             self.kinematic.left_encoder_tics += temp[0]  # Accumulate left encoder tics
    #             self.kinematic.right_encoder_tics += temp[1]  # Accumulate right encoder tics
    #             print("Left : ", self.kinematic.left_encoder_tics)
    #             print("Right : ", self.kinematic.right_encoder_tics)
    #             self.push_back_encode = False
    #             start = time.perf_counter()


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
                # else:
                #     # self.push_back_encode = True  # Set push_back_encode to True when paused
                # time.sleep(0.001)



    # def loggingProcess(self):
    #     head, tail = 1, 1
    #     start = time.perf_counter()
    #     with ThreadPoolExecutor() as executor:
    #         while True:
    #             if not self.log_process.is_alive():
    #                 time.sleep(0.1)  # Add a small delay to reduce CPU usage
    #                 continue

    #             if not self.push_back_encode.value:  # Only log encoder values if not paused
    #                 self.encoder_values.append(self.kinematic.control.get_encode_values())

    #             # print("Logging process: ", self.push_back_encode.value)
    #             if self.push_back_encode.value:  # Check if push_back_encode flag is set
    #                 # print("Entered if statement")
    #                 tail = head
    #                 head = len(self.encoder_values)
    #                 end = time.perf_counter()
    #                 time_elapsed = end - start
    #                 self.kinematic.time_point.append(time_elapsed)
    #                 async_results = [executor.submit(self.ticCount, tail, head)]
    #                 for async_result in as_completed(async_results):
    #                     temp = async_result.result()
    #                     print("Left async vec: ", temp[0])
    #                     print("Right async vec: ", temp[1])
    #                     self.kinematic.left_encoder_tics += temp[0]  # Accumulate left encoder tics
    #                     self.kinematic.right_encoder_tics += temp[1]  # Accumulate right encoder tics
    #                     print("Left : ", self.kinematic.left_encoder_tics)
    #                     print("Right : ", self.kinematic.right_encoder_tics)
    #                     self.push_back_encode.value = False
    #                     start = time.perf_counter()

    #             time.sleep(0.1)



    # def loggingThread(self):
    #         head, tail = 1, 1
    #         start = time.perf_counter()
    #         time_elapsed_list = [] 
    #         while True:
    #             # print("Start of while loop")
    #             # start = time.perf_counter()
    #             # print("log thread: ", self.log_thread)
    #             if not self.log_thread:
    #                 # time.sleep(0.01)  # Add a small delay to reduce CPU usage
    #                 continue
                
    #             with self.encoder_lock:
    #                 self.encoder_values.append(self.kinematic.control.get_encode_values())

    #             # self.encoder_values.append(self.kinematic.control.get_encode_values())
    #             # end = time.perf_counter()
    #             # time_elapsed = end - start
    #             # time_elapsed_list.append(time_elapsed)
    #             # average_time_elapsed = sum(time_elapsed_list) / len(time_elapsed_list)  # Calculate average time elapsed
    #             # print("Average time elapsed: ", average_time_elapsed)
                
    #             # print("Logging thread: ", self.push_back_encode.value)
    #             if self.push_back_encode.value:
    #                 tail = head
    #                 head = len(self.encoder_values) 
    #                 end = time.perf_counter()
    #                 time_elapsed = end - start
    #                 self.kinematic.time_point.append(time_elapsed)
    #                 print("tail, head: ", tail, head)
    #                 async_result = self.async_ticCount(tail, head)
    #                 print("Async set")
    #                 temp = async_result.get(timeout=10)
    #                 print("Temp set", temp)
    #                 print("Left async vec: ", temp[0])
    #                 print("Right async vec: ", temp[1])
    #                 self.kinematic.left_encoder_tics += temp[0]  # Accumulate left encoder tics
    #                 self.kinematic.right_encoder_tics += temp[1]  # Accumulate right encoder tics
    #                 print("Left encoder tics: ", self.kinematic.left_encoder_tics)
    #                 print("Right encoder tics: ", self.kinematic.right_encoder_tics)
    #                 self.push_back_encode.value = False
    #                 start = time.perf_counter()



    # def loggingThread(self):
    #     head, tail = 1, 1
    #     start = time.perf_counter()
    #     time_elapsed_list = [] 
    #     while True:
    #         if not self.log_thread:
    #             continue

    #         with self.encoder_lock:
    #             self.encoder_values.append(self.kinematic.control.get_encode_values())

    #         if self.push_back_encode.value:
    #             tail = head
    #             head = len(self.encoder_values)
    #             end = time.perf_counter()
    #             time_elapsed = end - start
    #             self.kinematic.time_point.append(time_elapsed)
    #             print("tail, head: ", tail, head)
    #             self.async_ticcount_event.clear()
    #             process = multiprocessing.Process(target=self.async_ticCount, args=(tail, head, self.async_ticcount_event, self.result_queue, self.async_ticcount_lock))  # Pass the lock to the async_ticCount function
    #             process.start()
    #             print("Process started")
    #             self.async_ticcount_event.wait()
    #             print("Event set")
    #             process.join()
    #             print("Process completed")
    #             temp = self.result_queue.get()
    #             print("Temp set", temp)
    #             print("Left async vec: ", temp[0])
    #             print("Right async vec: ", temp[1])
    #             self.kinematic.left_encoder_tics += temp[0]
    #             self.kinematic.right_encoder_tics += temp[1]
    #             print("Left encoder tics: ", self.kinematic.left_encoder_tics)
    #             print("Right encoder tics: ", self.kinematic.right_encoder_tics)
    #             self.push_back_encode.value = False
    #             start = time.perf_counter()




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
            print("Time elapsed: ", time_elapsed)
    
    
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


    
    #     # Modify the start_logging_thread() method to create and start the logging thread
    # def start_logging_thread(self):
    #     self.log_thread = True
    #     self.push_back_encode = False  # Make sure push_back_encode is initially set to False
    #     self.encoder_thread = threading.Thread(target=self.loggingThread)
    #     self.encoder_thread.start()

    # # Modify the stop_logging_thread() method to stop the logging thread
    # def stop_logging_thread(self):
    #     self.log_thread = False
    #     self.encoder_thread.join()  # Wait for the logging thread to finish before returning

    # # Modify the pause_logging_thread() method to set the push_back_encode flag
    # def pause_logging_thread(self):
    #     self.push_back_encode = True

    # # Modify the resume_logging_thread() method to reset the push_back_encode flag
    # def resume_logging_thread(self):
    #     self.push_back_encode = False

    def start_logging_thread(self):
        self.log_thread = True
        self.push_back_encode = False
        self.encoder_thread = multiprocessing.Process(target=self.loggingThread)
        self.encoder_thread.start()

    def stop_logging_thread(self):
        self.log_thread = False
        self.encoder_thread.join()

    def pause_logging_thread(self):
        self.push_back_encode = True

    def resume_logging_thread(self):
        self.push_back_encode = False

    
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



    #Make the compilation faster and if that is not possible talk with Danish what do
    #Potentially make entire code in c++ (despair)
    def searchGoal(self):
        # self.qrCodeThread()

        # self.aprilTagThread()
        # april_tag_thread = threading.Thread(target=self.aprilTagThread)
        # april_tag_thread.start()
        # self.vision_thread = True
        # vision_thread = threading.Thread(target=self.x_coord_object)
        # vision_thread.start()
        # print("Got here")
        # time.sleep(100)
        # self.log_thread = True
        # encoder_thread = threading.Thread(target=self.loggingThread)
        # encoder_thread.start()
        self.start_logging_process()
        start = time.perf_counter()
        
        while True:
            print(self.push_back_encode.value)
            self.kinematic.control.goStraight(1000)
            self.pause_logging_process()
            self.stop_logging_thread()
            print(self.push_back_encode.value)
            time.sleep(5)
            self.resume_logging_process()
            predict = self.updateDirVec() + self.ico_out
            print("Predict theta: ", predict)
            
            self.resume_logging_thread()
            # self.start_logging_thread()
            self.kinematic.control.turn(predict* 1.0) 
            time.sleep(5)
            print("Negative Predict theta: ", -predict)
            self.kinematic.control.turn(-predict * 1.0)
            time.sleep(5)


            end = time.perf_counter()
            time_elapsed = end - start
            print("Time elapsed: ", time_elapsed)
            if time_elapsed > 5:
                time.sleep(2)
                self.push_back_encode = True
                self.stop_logging_thread()
                time.sleep(5)

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