from kinematics import Kinematics
from yolo import YOLO
from camera import Camera
import cv2
import math


class learningWalk:

    def __init__(self):
        self.kinematic = Kinematics()
        self.yolo = YOLO()
        self.picam = Camera()

        self.center = self.picam.width/2
        self.x = self.center


    def ICO(self, predict, reflex):
        temp = 0

    
    def piroutte(self):
        temp = 0
    
    
    def visionTurn(self):
        scene = self.picam.getImage()
        box_center = self.yolo.run_yolo(scene)
        # print(box_center)
        # print(self.x)

        if len(box_center) > 0:
            self.x = box_center[0][0]
            # print("Here")
            # print(self.x)

        if self.x < self.center - 10:
            self.kinematic.control.setLeftMotor(self.kinematic.control.speed, 0)
            self.kinematic.control.setRightMotor(self.kinematic.control.speed, 1)
        elif self.x > self.center + 10:
            self.kinematic.control.setLeftMotor(self.kinematic.control.speed, 1)
            self.kinematic.control.setRightMotor(self.kinematic.control.speed, 0)
        else:
            self.kinematic.control.setLeftMotor(0, 1)
            self.kinematic.control.setRightMotor(0, 1)



from learningwalk import learningWalk
import time

walk = learningWalk()

num_frames = 0
prev_time = time.time()

while True:
    walk.visionTurn()

    num_frames += 1
    curr_time = time.time()
    time_diff = curr_time - prev_time
    if time_diff > 1:
        fps = num_frames / time_diff
        print("fps: ", fps)
        num_frames = 0
        prev_time = curr_time

    # check for key press
    key = cv2.waitKey(1)
    if key == 27: # 27 is the ASCII code for the escape key
        break