# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2


class camera:
    camera = PiCamera()
    rawCapture = PiRGBArray(camera, size=(640, 480))


    def setup(self):
        # camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        # allow the camera to warmup
        time.sleep(0.1)
        
    def getImage(self):
        # capture frames from the camera
        self.camera.capture(self.rawCapture, format="bgr")
        image = self.rawCapture.array
        image = self.imageFlip(image)
        # display the image on screen and wait for a keypress
        cv2.imshow("Image", image)
        cv2.imwrite("image.png", image)
        cv2.waitKey(0)

    def getVideo(self):
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            # show the frame
            image = self.imageFlip(image)
            cv2.imshow("Frame", image)
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
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