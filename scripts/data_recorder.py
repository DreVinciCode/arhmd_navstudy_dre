#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import sys, time
import numpy as np
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import csv


class arStudyData:

    def __init__(self):

        rospy.init_node('poseData', anonymous = True)
        self.current_filename = ""
        self.recording = False
        self.counter = 1

        self.y_plan_array = []
        self.x_plan_array = []

        # subscription to current reached goal notified by labeled point.
        self.currentpoint_sub = rospy.Subscriber("/point_ab", String, self.callback_point)

        # subscription to participart infomation (name and modal)
        self.command_sub = rospy.Subscriber("/data_logging_commands",String,self.command_callback)

        # subscription to position topic
        self.pose_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, self.callback_pose)

        # # subscription to odometry topic
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # subscription to global path plan
        # self.global_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.global_plan_callback)

        # subscriptoin to local path plan
        self.local_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self.local_plan_callback)
        #
        # # subscription to navfnROS path plan
        # self.navfnRos_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.navfnRos_callback)
        #
        # subscription to command velocity topic
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel_mux/input/navi", Twist, self.cmd_vel_callback)

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

            with open('/home/drevinci/catkin_ws/src/ari_study/rosbag/' + self.current_filename + ".csv", mode='w') as csv_file:
                logger = csv.writer(csv_file, delimiter=',')

                for i in range(len(self.y_plan_array)):
                    logger.writerow([self.x_plan_array[i], self.y_plan_array[i]])

            self.y_plan_array = []
            self.x_plan_array = []
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

    # def odom_callback(self,data):
    #
    #    if self.recording == True:
    #        self.bag.write('/odom',data)
    #    else:
    #        pass

    def local_plan_callback(self, data):
        x_array = []
        y_array = []

        if self.recording == True:

            for i in range(len(data.poses)):
                 y_array.append(data.poses[i].pose.position.y)
                 x_array.append(data.poses[i].pose.position.x)
            xavg = np.mean(x_array)
            yavg = np.mean(y_array)
            self.x_plan_array.append(xavg)
            self.y_plan_array.append(yavg)

        else:
            pass

    def cmd_vel_callback(self,data):

        if self.recording == True:
            #/mobile_base/commands/velocity
            self.bag.write('/cmd_vel_mux/input/navi', data)
        else:
            pass



if __name__ == '__main__':
    nav = arStudyData()
