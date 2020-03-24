#!/usr/bin/env python
# coding=utf8

"""
Node for get altitude from seek lidar.
"""

import serial
import rospy
import tf
import threading
import time

import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Quaternion

show_plot_flag = False              # get plot

if show_plot_flag:
    import matplotlib.pyplot as plt

# topics
alt_topic = "/drone/alt"
topic_imu = "/mavros/imu/data"

port = "/dev/ttyUSB0"

# setup params
_rate = 80.0        # rate 80 Hz
_rate_lidar = 80.

srez = 20
lidar_offset = 0.0
speed_change = 1.0

use_filter_flag = True

# init values
current_val_lidar = 0.
alt_msg = Float32()
alt_array = list()
offset_filter = 0.0
init_filter = False
rotate = Quaternion()
srez_lidar = list()
ser = None

"""
def filter(current_data, prev_data, current_speed, max_speed):
    """
    #Pub alt from lidar
    #:param alt:
    #:return:
    """
    global offset_filter
    if current_speed < -max_speed:
        print("jump UP")
        offset_filter = prev_data - current_data
        print(offset_filter)
    if current_speed > max_speed:
        print("jump DOWN")
        offset_filter = 0.0
        print(offset_filter)
"""

def getLidarData():
    """
    Get lidar data from lidar
    :type data: LaserScan
    :return:
    """

    global current_val_lidar, rotate, lidar_offset, srez_lidar, ser, use_filter_flag
    distance = 0.0
    _delay = 1./_rate_lidar
    try:
        while True:
            count = ser.in_waiting
            if count > 8:
                recv = ser.read(9)
                ser.reset_input_buffer()
                if recv[0] == 'Y' and recv[1] == 'Y':  # 0x59 is 'Y'
                    low = int(recv[2].encode('hex'), 16)
                    high = int(recv[3].encode('hex'), 16)
                    distance = (low + high * 256) * 0.01


            # # пересчитываеем проектцию по углам
            euler = tf.transformations.euler_from_quaternion((rotate.x, rotate.y, rotate.z, rotate.w))
            dist_norm = (np.cos(euler[0]) * distance) * np.cos(euler[1])

            current_val_lidar = dist_norm - lidar_offset

            if use_filter_flag:
                srez_lidar.append(current_val_lidar)
                if len(srez_lidar) > srez:
                    del srez_lidar[0]

            time.sleep(_delay)
    except:
        if ser != None:
            ser.close()
        exit()
