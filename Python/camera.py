# # import the necessary packages
# from picamera.array import PiRGBArray
# from picamera import PiCamera
# import time
# import cv2


# class camera:
#     camera = PiCamera()
#     rawCapture = PiRGBArray(camera, size=(160, 120))


#     def setup(self):
#         # camera = PiCamera()
#         self.camera.resolution = (160, 120)
#         self.camera.framerate = 32
#         # allow the camera to warmup
#         time.sleep(0.1)
        
#     def getImage(self):
#         # capture frames from the camera
#         self.camera.capture(self.rawCapture, format="bgr")
#         image = self.rawCapture.array
#         image = self.imageFlip(image)
#         # display the image on screen and wait for a keypress
#         cv2.imshow("Image", image)
#         cv2.imwrite("image.png", image)
#         cv2.waitKey(0)
        
#         # return image and information
#         return image

#     def getVideo(self):
#         for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
#             # grab the raw NumPy array representing the image, then initialize the timestamp
#             # and occupied/unoccupied text
#             image = frame.array
#             # show the frame
#             image = self.imageFlip(image)

#             cv2.imshow("Frame", image)
#             key = cv2.waitKey(1) & 0xFF

#             # clear the stream in preparation for the next frame
#             self.rawCapture.truncate(0)
            
#             # if the `q` key was pressed, break from the loop
#             if key == ord("q"):
#                 break
    
#     def imageFlip(self, image):
#         image = cv2.flip(image, 0)
#         image = cv2.flip(image, 1)
#         return image




# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2


class Camera:
    width = 160
    height = 120

    def __init__(self):
        self.camera = PiCamera()
        self.raw_capture = PiRGBArray(self.camera, size=(self.width, self.height))
        self.setup()

    def setup(self):
        self.camera.resolution = (self.width, self.height)
        self.camera.framerate = 32
        # allow the camera to warm up
        time.sleep(0.1)

    def getImage(self):
        # capture frames from the camera
        self.camera.capture(self.raw_capture, format="bgr")
        image = self.raw_capture.array
        image = self.imageFlip(image)
        # clear the stream in preparation for the next frame
        self.raw_capture.truncate(0)
        # return the image and information
        return image

    def getVideo(self):
        for frame in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            # show the frame
            image = self.imageFlip(image)

            cv2.imshow("Frame", image)
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            self.raw_capture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

    def imageFlip(self, image):
        image = cv2.flip(image, 0)
        image = cv2.flip(image, 1)
        return image



# #Code snippet to setup camera, get an image and/or get a video stream
# #Paste into main to run
# from camera import camera

# video = camera()
# video.setup()
# # video.getImage()
# video.getVideo()