#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import math
import sys, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray
import csv



class latencyData:

    def __init__(self):

        rospy.init_node('latency_test', anonymous = True)

        # variables
        filenumber = 0
        self.ls_msg_avg = 0.0
        self.cm_msg_avg = 0.0
        self.lz_msg_avg = 0.0
        self.ph_msg_avg = 0.0
        self.counter_ls = 0
        self.counter_cm = 0
        self.counter_lz = 0
        self.counter_ph = 0
        self.ls_array = []
        self.cm_array = []
        self.lz_array = []
        self.ph_array = []

        # subscribers
        self.laserscan_return = rospy.Subscriber("/laser_from", LaserScan, self.scan_return_callback)
        self.costmap_return = rospy.Subscriber("/costmap_from", OccupancyGrid, self.cost_return_callback)
        self.localization_return = rospy.Subscriber("/localization_from", PoseArray, self.local_return_callback)
        self.path_return = rospy.Subscriber("/path_from", Path, self.path_return_callback)

        rospy.spin()

        #script to write data to csv file
        with open('laserscan_latency_' + str(filenumber) + ".csv", mode='w') as csv_1:
            logger = csv.writer(csv_1, delimiter=',')

            for i in range(len(self.ls_array)):
                logger.writerow([self.ls_array[i]])

        with open('costmap_latency' + str(filenumber)+ ".csv", mode='w') as csv_2:
            logger = csv.writer(csv_2, delimiter=',')

            for i in range(len(self.cm_array)):
                logger.writerow([self.cm_array[i]])

        with open('localization_latency' + str(filenumber)+ ".csv", mode='w') as csv_3:
            logger = csv.writer(csv_3, delimiter=',')

            for i in range(len(self.lz_array)):
                logger.writerow([self.lz_array[i]])

        with open('path_latency' + str(filenumber)+ ".csv", mode='w') as csv_4:
            logger = csv.writer(csv_4, delimiter=',')

            for i in range(len(self.ph_array)):
                logger.writerow([self.ph_array[i]])


    def scan_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        # convert to miliseconds
        ls_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)
        self.ls_array.append(ls_msg_dt)

        # perform moving average
        self.counter_ls = self.counter_ls + 1.0
        self.ls_msg_avg = (self.counter_ls-1.0)*(self.ls_msg_avg)/(self.counter_ls) + (1.0/self.counter_ls)*ls_msg_dt
        self.printout()



    def cost_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        cm_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)
        self.cm_array.append(cm_msg_dt)

        self.counter_cm = self.counter_cm + 1.0
        self.cm_msg_avg = (self.counter_cm-1.0)*(self.cm_msg_avg)/(self.counter_cm) + (1.0/self.counter_cm)*cm_msg_dt
        self.printout()


    def local_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        lz_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)
        self.lz_array.append(lz_msg_dt)

        self.counter_lz = self.counter_lz + 1.0
        self.lz_msg_avg = (self.counter_lz-1.0)*(self.lz_msg_avg)/(self.counter_lz) + (1.0/self.counter_lz)*lz_msg_dt
        self.printout()


    def path_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        ph_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)
        self.ph_array.append(ph_msg_dt)

        self.counter_ph = self.counter_ph + 1.0
        self.ph_msg_avg = (self.counter_ph-1.0)*(self.ph_msg_avg)/(self.counter_ph) + (1.0/self.counter_ph)*ph_msg_dt
        self.printout()

    def printout(self):
        print("LaserScan [ms]: ", round(self.ls_msg_avg, 3) , "\tCostmap [ms]: " , round(self.cm_msg_avg, 3) , "\tPath [ms] :" , round(self.ph_msg_avg, 3) , "\tLocalization [ms]: ", round(self.lz_msg_avg, 3))



if __name__ == '__main__':
    nav = latencyData()

    # nav.laserscan_callback()
