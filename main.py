# import time
# import pigpio

# pin = 13

# pi = pigpio.pi()
# pi1 = pigpio.pi()
# pi.set_mode(12, 1) # GPIO 12 as output
# pi.set_mode(16, 1)
# pi.set_mode(26, 1)
# pi.set_mode(5, 1)
# pi.set_mode(6, 1)
# pi1.set_mode(pin, 1) # GPIO 13 as output

# pi.set_PWM_range(12, 255)  # now  25 1/4,   50 1/2,   75 3/4 on
# pi.set_PWM_range(13, 255)  # now  25 1/4,   50 1/2,   75 3/4 on

# pi.set_PWM_frequency(12, 1000)
# pi.set_PWM_frequency(13, 1000)

# pi.write(16, 1)
# pi.write(26, 0)
# pi.write(6, 1)
# pi.write(5, 0)

# # pi.write(12, 1) # set local Pi's GPIO 4 high
# # pi1.write(pin, 1) # set local Pi's GPIO 4 high
# print('before')

# pi.set_PWM_dutycycle(12,   200) # PWM on highest
# pi.set_PWM_dutycycle(13,   200) # PWM on highest


# time.sleep(5)
# print('after')

# pi.set_PWM_dutycycle(12,   0) # PWM on highest
# pi.set_PWM_dutycycle(13,   0) # PWM on highest

# # print('before')
# # time.sleep(2)
# # print('after')

# # pi.write(12, 0) # set local Pi's GPIO 4 low
# # pi1.write(pin, 0) # set local Pi's GPIO 4 low

# pi.stop()
# pi1.stop()


#Need to connect the lidar with the USB
import time
from rplidar import RPLidar
print('before')
lidar = RPLidar('/dev/ttyUSB0')
print('after')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    if i > 10:
        break

lidar.stop()
lidar.stop_motor()

time.sleep(3)
lidar.disconnect()

# # import the necessary packages
# from picamera.array import PiRGBArray
# from picamera import PiCamera
# import time
# import cv2
# # initialize the camera and grab a reference to the raw camera capture
# camera = PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640, 480))
# # allow the camera to warmup
# time.sleep(0.1)
# # capture frames from the camera
# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
# 	# grab the raw NumPy array representing the image, then initialize the timestamp
# 	# and occupied/unoccupied text
# 	image = frame.array
# 	# show the frame
# 	cv2.imshow("Frame", image)
# 	key = cv2.waitKey(1) & 0xFF
# 	# clear the stream in preparation for the next frame
# 	rawCapture.truncate(0)
# 	# if the `q` key was pressed, break from the loop
# 	if key == ord("q"):
# 		break