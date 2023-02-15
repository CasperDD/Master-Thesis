import time
import pigpio

pin = 10

pi = pigpio.pi()
pi1 = pigpio.pi()
pi.set_mode(12, pigpio.OUTPUT) # GPIO 12 as output
pi1.set_mode(pin, pigpio.OUTPUT) # GPIO 13 as output

pi.set_PWM_range(12, 100)  # now  25 1/4,   50 1/2,   75 3/4 on

pi.write(12, 1) # set local Pi's GPIO 4 high
pi1.write(pin, 1) # set local Pi's GPIO 4 high

print('before')
time.sleep(2)
print('after')

pi.set_PWM_dutycycle(12,   100) # PWM on highest

print('before')
time.sleep(2)
print('after')

pi.write(12, 0) # set local Pi's GPIO 4 low
pi1.write(pin, 0) # set local Pi's GPIO 4 low

pi.stop()
pi1.stop()

# from rplidar import RPLidar
# lidar = RPLidar('/dev/ttyAMA0')

# info = lidar.get_info()
# print(info)

# health = lidar.get_health()
# print(health)

# for i, scan in enumerate(lidar.iter_scans()):
#     print('%d: Got %d measurments' % (i, len(scan)))
#     if i > 10:
#         break

# lidar.stop()
# lidar.stop_motor()
# lidar.disconnect()