#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import sys, time
import numpy as np
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import csv


class latencyData:

    def __init__(self):

        rospy.init_node('latency_test', anonymous = True)
        self.first_ls_msg = 0
        self.second_ls_msg = 0
        # self.currentpoint_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # rospy.spin()



    # def scan_callback(self, data):
    # 	print(data) 


    def laserscan_callback(self):

    	pub = rospy.Publisher("/ARFUROS/LaserScan", LaserScan, queue_size = 1)

    	# Step 1: get original scan message from driver
    	msg =  rospy.wait_for_message("/scan",LaserScan)

    	print(msg)


    	# Step 2: make a copy of that message and publish it to AR device
    		# save time at sending
    	# rospy.Publisher("/ARFUROS/LaserScan", LaserScan, queue_size = 1)

    	pub.publish(msg)


    	# Step 3: wait for the message to get back from the AR device -- the AR device should set the 
    	# header of the return message to be identical to the header of the message it received
    		# save time at receiving and compare
    	second_msg = rospy.wait_for_message("/ARFUROS/LatencyTest/LaserScan",LaserScan)

    	print(second_msg)

        # obtain first message from /scan topic
        """rospy.loginfo("Getting first message...")
        self.first_ls_msg = rospy.wait_for_message("/ARFUROS/LaserScan",LaserScan)
        first_msg_id = self.first_ls_msg.header.seq

        time_stamp_1 = self.first_ls_msg.header.stamp.secs + self.first_ls_msg.header.stamp.nsecs / 1000000000.0

        first_msg_time = rospy.Time.now()
        rospy.loginfo("Got first message!")
        # print(first_msg_id, time_stamp_1,first_msg_time)
        print(self.first_ls_msg)

        # obtain first message from /scan topic
        rospy.loginfo("Waiting for return message...")
        self.second_ls_msg = rospy.wait_for_message("/ARFUROS/LatencyTest/LaserScan",LaserScan)

        time_stamp_2 = self.second_ls_msg.header.stamp.secs + self.second_ls_msg.header.stamp.nsecs / 1000000000.0
        second_msg_id = self.second_ls_msg.header.seq

        second_msg_time = rospy.Time.now()
        rospy.loginfo("Got second message!")
        # print(second_msg_id, time_stamp_2 ,second_msg_time)
        print(self.second_ls_msg)"""




        # check if second message is equal or greater than the first message and then calculate dt in seconds
        """if first_msg_id <= second_msg_id:
        	# dt = time_stamp_2 - time_stamp_1
            ls_msg_dt = second_msg_time - first_msg_time
            seconds_dt = ls_msg_dt / 1e9
            print("/scan latency is: ", ls_msg_dt)
        else:
            print("Error in message id...")"""


if __name__ == '__main__':
    nav = latencyData()

    nav.laserscan_callback()
