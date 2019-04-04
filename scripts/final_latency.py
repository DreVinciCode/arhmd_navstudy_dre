#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import math
import sys, time
import numpy as np
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray
import csv


class latencyData:

    def __init__(self):

        rospy.init_node('latency_test', anonymous = True)

        # variables
        self.ls_msg_avg = 0.0
        self.cm_msg_avg = 0.0
        self.lz_msg_avg = 0.0
        self.ph_msg_avg = 0.0
        self.counter_ls = 0
        self.counter_cm = 0
        self.counter_lz = 0
        self.counter_ph = 0

        # subscribers
        self.laserscan_return = rospy.Subscriber("/laser_from", LaserScan, self.scan_return_callback)
        self.costmap_return = rospy.Subscriber("/costmap_from", OccupancyGrid, self.cost_return_callback)
        self.localization_return = rospy.Subscriber("/localization_from", PoseArray, self.local_return_callback)
        self.path_return = rospy.Subscriber("/path_from", Path, self.path_return_callback)

        rospy.spin()

    def scan_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        # convert to miliseconds
        ls_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)

        # perform moving average
        self.counter_ls = self.counter_ls + 1
        self.ls_msg_avg = (self.counter_ls-1)*(self.ls_msg_avg)/(self.counter_ls) + (1/self.counter_ls)*ls_msg_dt
        self.printout()

    def cost_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        cm_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)

        self.counter_cm = self.counter_cm + 1
        self.cm_msg_avg = (self.counter_cm-1)*(self.cm_msg_avg)/(self.counter_cm) + (1/self.counter_cm)*cm_msg_dt
        self.printout()


    def local_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        lz_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)

        self.counter_lz = self.counter_lz + 1
        self.lz_msg_avg = (self.counter_lz-1)*(self.lz_msg_avg)/(self.counter_lz) + (1/self.counter_lz)*lz_msg_dt
        self.printout()


    def path_return_callback(self, data):

        msg_return = data

        later = rospy.get_rostime()

        diff_secs = later.secs - msg_return.header.stamp.secs
        diff_nsecs = later.nsecs - msg_return.header.stamp.nsecs

        ph_msg_dt = (diff_nsecs / 1000000) + (diff_secs * 1000)

        self.counter_ph = self.counter_ph + 1
        self.ph_msg_avg = (self.counter_ph-1)*(self.ph_msg_avg)/(self.counter_ph) + (1/self.counter_ph)*ph_msg_dt
        self.printout()
        
    def printout(self):
        print("LaserScan [ms]: ", round(self.ls_msg_avg) , "\tCostmap [ms]: " , round(self.cm_msg_avg) , "\tPath [ms] :" , round(self.ph_msg_avg) , "\tLocalization [ms]: ", round(self.lz_msg_avg))



if __name__ == '__main__':
    nav = latencyData()

    # nav.laserscan_callback()
