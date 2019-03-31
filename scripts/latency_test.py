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
        # self.currentpoint_sub = rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)

        # rospy.spin()

    def laserscan_callback(self):

        # obtain first message from /scan topic
        rospy.loginfo("Getting first message...")
        self.first_ls_msg = rospy.wait_for_message("/scan",LaserScan)
        first_msg_id = self.first_ls_msg.header.seq
        first_msg_time = rospy.Time.now()
        rospy.loginfo("Got first message!")
        print(first_msg_id, first_msg_time)

        # obtain first message from /scan topic
        rospy.loginfo("Waiting for return message...")
        self.second_ls_msg = rospy.wait_for_message("/scan_return",LaserScan)
        second_msg_id = self.second_ls_msg.header.seq
        second_msg_time = rospy.Time.now()
        rospy.loginfo("Got second message!")
        print(second_msg_id, second_msg_time)

        # check if second message is equal or greater than the first message and then calculate dt in seconds
        if first_msg_id >= second_msg_id:
            ls_msg_dt = second_msg_time - first_msg_time
            print("/scan latency is: ", ls_msg_dt)
        else:
            print("Error in message id...")


if __name__ == '__main__':
    nav = latencyData()

    nav.laserscan_callback()
