#!/usr/bin/env python
# coding=utf8

"""
Node for get altitude from seek lidar.
"""

import serial
import rospy
import tf
import time

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
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

# init values
current_val_lidar = 0.
alt_msg = Float32()
rotate = Quaternion()
ser = None


def getLidarData():
    """
    Get lidar data from lidar
    :type data: LaserScan
    :return:
    """

    global current_val_lidar, rotate, ser
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

            current_val_lidar = dist_norm #- lidar_offset

            time.sleep(_delay)
    except:
        if ser != None:
            ser.close()
        exit()

def callbackRot(data):
    """
    Callback IMU
    :type data: Imu
    :param data: data from tf
    :return:
    """
    global rotate
    rotate = data.orientation


if __name__ == '__main__':

    # Init node
    rospy.init_node('lidar_seek_node', anonymous=True)

    rate = rospy.Rate(_rate)        # set rate

    # init params
    port = rospy.get_param("~port", port)
    # Subscriber
    rospy.Subscriber(topic_imu, Imu, callbackRot)

    # Publisher
    alt_pub = rospy.Publisher(alt_topic, Float32, queue_size=10)

    # init serial
    ser = serial.Serial(port, 115200)
    if ser.is_open == False:
        ser.open()

    #call getLidarData and output data on screen
    while(True):
        getLidarData()
        print(current_val_lidar)