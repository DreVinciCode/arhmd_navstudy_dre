#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
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
        self.ls_msg_avg = 0
        self.cm_msg_avg = 0
        self.lz_msg_avg = 0
        self.ph_msg_avg = 0
        self.counter_ls = 0
        self.counter_cm = 0
        self.counter_lz = 0
        self.counter_ph = 0

        # subscribers
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback)
        self.localiztion_sub = rospy.Subscriber("/particlecloud", PoseArray, self.localalize_callback)
        self.path_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.path_callback)

        # publishers
        self.laser_pub = rospy.Publisher("/ARFUROS/LaserScan", LaserScan, queue_size = 1)
        self.costmap_pub = rospy.Publisher("/ARFUROS/Costmap", OccupancyGrid, queue_size = 1)
        self.localalize_pub = rospy.Publisher("/ARFUROS/Localization", PoseArray, queue_size = 1)
        self.path_pub = rospy.Publisher("/ARFUROS/Path", Path, queue_size = 1)

        # rospy.spin()


    def scan_callback(self, data):

        msg = data
        # Create a for-loop to calculate a moving average for the time latency
    	# Step 1: get original scan message from driver
    	# msg =  rospy.wait_for_message("/scan",LaserScan)
        first_msg_id = msg.header.seq
        time_stamp_1 = msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0
    	# print(msg)

    	# Step 2: make a copy of that message and publish it to AR device
    	# save time at sending
    	pub.publish(msg)
        msg_time_sent = rospy.Time.now()

    	# Step 3: wait for the message to get back from the AR device -- the AR device should set the
    	# header of the return message to be identical to the header of the message it received
    	# save time at receiving and compare
    	second_msg = rospy.wait_for_message("/ARFUROS/LatencyTest/LaserScan",LaserScan)
        msg_time_recieved = rospy.Time.now()
        time_stamp_2 = second_msg.header.stamp.secs + second_msg.header.stamp.nsecs / 1000000000.0
        second_msg_id = second_msg.header.seq
    	# print(second_msg)

        if first_msg_id == second_msg_id:
            dt = time_stamp_2 - time_stamp_1
            ls_msg_dt = msg_time_recieved - msg_time_sent
            self.counter_ls = self.counter_ls + 1
            # print("/scan latency is: ", ls_msg_dt, dt)
        else:
            print("LaserScan Message ID mismatch...")

        self.ls_msg_avg = (self.counter_ls-1)*(self.ls_msg_avg)/(self.counter_ls) + (1/self.counter_ls)*ls_msg_dt


    def costmap_callback(self, data):

        msg = data
        # Create a for-loop to calculate a moving average for the time latency
    	# Step 1: get original scan message from driver
    	# msg =  rospy.wait_for_message("/scan",LaserScan)
        first_msg_id = msg.header.seq
        time_stamp_1 = msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0
    	# print(msg)

    	# Step 2: make a copy of that message and publish it to AR device
    	# save time at sending
    	pub.publish(msg)
        msg_time_sent = rospy.Time.now()

    	# Step 3: wait for the message to get back from the AR device -- the AR device should set the
    	# header of the return message to be identical to the header of the message it received
    	# save time at receiving and compare
    	second_msg = rospy.wait_for_message("/ARFUROS/LatencyTest/Costmap",OccupancyGrid)
        msg_time_recieved = rospy.Time.now()
        time_stamp_2 = second_msg.header.stamp.secs + second_msg.header.stamp.nsecs / 1000000000.0
        second_msg_id = second_msg.header.seq
    	# print(second_msg)

        if first_msg_id == second_msg_id:
            dt = time_stamp_2 - time_stamp_1
            cm_msg_dt = msg_time_recieved - msg_time_sent
            self.counter_cm = self.counter_cm + 1

            # print("/scan latency is: ", ls_msg_dt, dt)
        else:
            print("Costmap Message ID mismatch...")

        self.cm_msg_avg = (self.counter_cm-1)*(self.cm_msg_avg)/(self.counter_cm) + (1/self.counter_cm)*cm_msg_dt


    def localalize_callback(self, data):

        msg = data
        # Create a for-loop to calculate a moving average for the time latency
    	# Step 1: get original scan message from driver
    	# msg =  rospy.wait_for_message("/scan",LaserScan)
        first_msg_id = msg.header.seq
        time_stamp_1 = msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0
    	# print(msg)

    	# Step 2: make a copy of that message and publish it to AR device
    	# save time at sending
    	pub.publish(msg)
        msg_time_sent = rospy.Time.now()

    	# Step 3: wait for the message to get back from the AR device -- the AR device should set the
    	# header of the return message to be identical to the header of the message it received
    	# save time at receiving and compare
    	second_msg = rospy.wait_for_message("/ARFUROS/LatencyTest/Localization",PoseArray)
        msg_time_recieved = rospy.Time.now()
        time_stamp_2 = second_msg.header.stamp.secs + second_msg.header.stamp.nsecs / 1000000000.0
        second_msg_id = second_msg.header.seq
    	# print(second_msg)

        if first_msg_id == second_msg_id:
            dt = time_stamp_2 - time_stamp_1
            lz_msg_dt = msg_time_recieved - msg_time_sent
            self.counter_lz = self.counter_lz + 1
            # print("/scan latency is: ", ls_msg_dt, dt)
        else:
            print("Localization Message ID mismatch...")

        self.lz_msg_avg = (self.counter_lz-1)*(self.lz_msg_avg)/(self.counter_lz) + (1/self.counter_lz)*lz_msg_dt


    def path_callback(self, data):

        msg = data
        # Create a for-loop to calculate a moving average for the time latency
    	# Step 1: get original scan message from driver
    	# msg =  rospy.wait_for_message("/scan",LaserScan)
        first_msg_id = msg.header.seq
        time_stamp_1 = msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0
    	# print(msg)

    	# Step 2: make a copy of that message and publish it to AR device
    	# save time at sending
    	pub.publish(msg)
        msg_time_sent = rospy.Time.now()

    	# Step 3: wait for the message to get back from the AR device -- the AR device should set the
    	# header of the return message to be identical to the header of the message it received
    	# save time at receiving and compare
    	second_msg = rospy.wait_for_message("/ARFUROS/LatencyTest/Path",Path)
        msg_time_recieved = rospy.Time.now()
        time_stamp_2 = second_msg.header.stamp.secs + second_msg.header.stamp.nsecs / 1000000000.0
        second_msg_id = second_msg.header.seq
    	# print(second_msg)

        if first_msg_id == second_msg_id:
            dt = time_stamp_2 - time_stamp_1
            ph_msg_dt = msg_time_recieved - msg_time_sent
            self.counter_ph = self.counter_ph + 1
            # print("/scan latency is: ", ls_msg_dt, dt)
        else:
            print("Path Message ID mismatch...")

        self.ph_msg_avg = (self.counter_ph-1)*(self.ph_msg_avg)/(self.counter_ph) + (1/self.counter_ph)*ph_msg_dt

    def printout(self):
        print("LaserScan: ", self.ls_msg_avg , "\tCostmap: " , self.cm_msg_avg , "\tPath :" , self.ph_msg_avg , "\tLocalization: ", self.lz_msg_avg)



if __name__ == '__main__':
    nav = latencyData()

    nav.laserscan_callback()
