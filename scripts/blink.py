#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import sys, time
import numpy as np
from std_msgs.msg import Float64, String, Int8
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import csv


class arStudyData:

    def __init__(self):

        rospy.init_node('blinker', anonymous = True)
        self.turning = False
        self.LEFT = False
        self.RIGHT = False
        self.flag = True

        self.y_plan = []
        self.x_plan = []
        self.path_array = []

        # subscriptoin to global path plan
        self.global_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.global_plan_callback)
        self.turning_pub = rospy.Publisher("/turn_signal", String, queue_size=10)

        rospy.spin()

    def global_plan_callback(self,data):

        current_x_array = []
        current_y_array = []

        future_x_array = []
        future_y_array = []

    
        threshold = 0.2

        if (len(self.path_array) >= 5):
            self.path_array.pop(1)
            self.path_array.append(data.poses)


            current_path = self.path_array[1]
            future_path = self.path_array[4]

            for i in range(len(current_path)):
                current_y_array.append(current_path[i].pose.position.y)
                current_x_array.append(current_path[i].pose.position.x)

            for i in range(len(future_path)):
                future_y_array.append(future_path[i].pose.position.y)
                future_x_array.append(future_path[i].pose.position.x)

            current_xavg = np.mean(current_x_array)
            current_yavg = np.mean(current_y_array)
            future_xavg = np.mean(future_x_array)
            future_yavg = np.mean(future_y_array)


            if (current_xavg > future_xavg):
                if future_yavg > (current_yavg + threshold):
                    self.turning_pub.publish("R")
                    print("R")

                elif future_yavg < (current_yavg - threshold):
                    self.turning_pub.publish("L")
                    print("L")

                else:
                    self.turning_pub.publish("S")
                    print("S") 

            elif (current_xavg < future_xavg):

                if future_yavg < (current_yavg - threshold):
                    self.turning_pub.publish("R")
                    print("R")

                elif future_yavg > (current_yavg + threshold):
                    self.turning_pub.publish("L")
                    print("L")

                else:
                    self.turning_pub.publish("S")
                    print("S") 

            else:
                self.turning_pub.publish("S")
                print("S")  


        else: 
            self.path_array.append(data.poses)


if __name__ == '__main__':
    nav = arStudyData()


