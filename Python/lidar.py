# Need to connect the lidar with the USB

# import time
# from rplidar import RPLidar


# print('before')
# lidar = RPLidar('/dev/ttyUSB0')
# print('after')

# info = lidar.get_info()
# print(info)

# health = lidar.get_health()
# print(health)

# for i, scan in enumerate(lidar.iter_scans()):
#     print('%d: Got %d measurments' % (i, len(scan)))
#     if i > 10:
#         break

# measurements = lidar.iter_measures()
# print(measurements)

# lidar.stop()
# lidar.stop_motor()

# time.sleep(3)

#
# 
# 
# 
#  lidar.disconnect()

# import sys
# from rplidar import RPLidar


# PORT_NAME = '/dev/ttyUSB0'


# def run(path):
#     '''Main function'''
#     lidar = RPLidar(PORT_NAME)
#     outfile = open(path, 'w')
#     try:
#         print('Recording measurments... Press Crl+C to stop.')
#         for measurment in lidar.iter_measures():
#             line = '\t'.join(str(v) for v in measurment)
#             outfile.write(line + '\n')
#     except KeyboardInterrupt:
#         print('Stoping.')
#     lidar.stop()
#     lidar.disconnect()
#     outfile.close()

# if __name__ == '__main__':
#     run('measurements.txt')



# '''Animates distances and measurment quality'''
# from rplidar import RPLidar
# import matplotlib.pyplot as plt
# import numpy as np

# PORT_NAME = '/dev/ttyUSB0'
# DMAX = 6000
# IMIN = 0
# IMAX = 50


# def update(plot, scan):
#     '''Updates plot'''
#     offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
#     plot.set_offsets(offsets)
#     intens = np.array([meas[0] for meas in scan])
#     plot.set_array(intens)
#     plt.show()
#     plt.pause(0.001)


# def run():
#     '''Main function'''
#     plt.ion()
#     lidar = RPLidar(PORT_NAME)
#     subplot = plt.subplot(111, projection='polar')
#     plot = subplot.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
#                            cmap=plt.cm.Greys_r, lw=0)
#     subplot.set_rmax(DMAX)
#     subplot.grid(True)
#     plt.show()
#     for scan in lidar.iter_scans():
#         if not plt.get_fignums():
#             break
#         update(plot, scan)
#     lidar.stop()
#     lidar.disconnect()

# if __name__ == '__main__':
#     run()