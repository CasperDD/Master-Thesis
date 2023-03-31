# import threading
# import cv2
# import apriltag

# class AprilTagDetector:

#     def __init__(self, resize_width=160, resize_height=128):
#         self.tag_detector = apriltag.Detector()
#         self.image = None
#         self.resize_width = resize_width
#         self.resize_height = resize_height
    
#     def start(self):
#         self.thread = threading.Thread(target=self._display_window)
#         self.thread.daemon = True
#         self.thread.start()

#     def _display_window(self):
#         # cv2.namedWindow('AprilTag', cv2.WINDOW_NORMAL)
#         # cv2.resizeWindow('AprilTag', 160, 120)

#         while True:
#             if self.image is not None:
#                 resized_image = cv2.resize(self.image, (self.resize_width, self.resize_height))
#                 gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
#                 tags = self.tag_detector.detect(gray)

#                 if tags:
#                     for tag in tags:
                        
#                         print("Detected AprilTag with ID", tag.tag_id)

#                 cv2.imshow('AprilTag', resized_image)

#             key = cv2.waitKey(1) & 0xFF
#             if key == ord("q"):
#                 break

#     def detect(self, image):
#         self.image = image

import cv2
import apriltag

class AprilTagDetector:
    def __init__(self, resize_width=160, resize_height=128):
        self.tag_detector = apriltag.Detector()
        self.image = None
        self.resize_width = resize_width
        self.resize_height = resize_height
    
    def display_window(self):
        while True:
            if self.image is not None:
                resized_image = cv2.resize(self.image, (self.resize_width, self.resize_height))
                gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
                tags = self.tag_detector.detect(gray)

                if tags:
                    for tag in tags:
                        print("Detected AprilTag with ID", tag.tag_id)

                cv2.imshow('AprilTag', resized_image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    def detect(self, image):
        self.image = image
        resized_image = cv2.resize(self.image, (self.resize_width, self.resize_height))
        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        tags = self.tag_detector.detect(gray)

        if tags:
            for tag in tags:
                print("Detected AprilTag with ID", tag.tag_id)

        cv2.imshow('AprilTag', resized_image)
        cv2.waitKey(1)
