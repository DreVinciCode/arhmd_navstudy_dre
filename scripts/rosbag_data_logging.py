#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import sys, time
import numpy as np
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class arStudyData:

    def __init__(self):

        rospy.init_node('poseData', anonymous = True)
        self.current_filename = ""
        self.recording = False
        self.counter = 1
        self.info_ready = False

        self.currentpoint_sub = rospy.Subscriber("/point_ab", String, self.callback_point)
        self.pose_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, self.callback_pose)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # self.dataPose = PoseWithCovarianceStamped()
        self.command_sub = rospy.Subscriber("/data_logging_commands",String,self.command_callback)
        rospy.spin()



    def callback_point(self,data):

        if data.data == "B":
            self.recording = True
            self.bag = rosbag.Bag(self.current_filename, 'w')
            print("Recording Now!")

        elif data.data == "C":
            self.recording = False
            self.bag.close()
            print("End of Recording..")

        else:
            pass

    def command_callback(self,data):

        self.current_filename = data.data
        print("Current file is " + self.current_filename)


    def callback_pose(self,data):

        if self.recording == True:
            self.bag.write("/amcl_pose",data)
        else:
            pass

    def odom_callback(self,data):

       if self.recording == True:
           self.bag.write('/odom',data)
       else:
           pass


if __name__ == '__main__':
    nav = arStudyData()
