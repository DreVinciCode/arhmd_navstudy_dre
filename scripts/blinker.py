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

        # subscription to current reached goal notified by labeled point.
        self.currentpoint_sub = rospy.Subscriber("/point_ab", String, self.callback_point)

        # subscriptoin to global path plan

        self.global_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.global_plan_callback)
        # self.full_plan_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.full_plan_callback)

        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel_mux/input/navi", Twist , self.cmd_vel_callback)

        self.turning_pub = rospy.Publisher("/turn_signal", Int8, queue_size=10)

        rospy.spin()

    def callback_point(self,data):
        x_array = []
        y_array = []

        if data.data == "B":
            self.turning = True
            # plan =  rospy.wait_for_message("/move_base/NavfnROS/plan",Path)
            plan =  rospy.wait_for_message("/move_base/DWAPlannerROS/global_plan",Path)

            for i in range(len(plan.poses)):
                 y_array.append(plan.poses[i].pose.position.y)
                 x_array.append(plan.poses[i].pose.position.x)

            xavg = np.mean(x_array)
            yavg = np.mean(y_array)

            self.y_plan = yavg
            # print("y value", self.y_plan)
        elif data.data == "C":
            self.turning = False

    def global_plan_callback(self,data):
        x_array = []
        y_array = []
        b_prime_array = []
        threshold = 0.2


        if self.flag == False:

            if self.turning == True:

                for i in range(len(data.poses)):
                    y = data.poses[i].pose.position.y
                    y_diff = y-self.y_plan
                    y_array.append(y_diff)
                    # print(y - self.y_plan)

                y_max = max(y_array)
                y_min = min(y_array)
                #
                if y_max > y_min and y_max > threshold:
                    print("Right")
                elif y_max < abs(y_min) and y_min <  -1*threshold:
                    print("Left")
                else:
                    print("straight")

        else:

            x1, x2 = [data.poses[0].pose.position.x, data.poses[3].pose.position.x]
            y1, y2 = [data.poses[0].pose.position.y, data.poses[3].pose.position.y]
            slope = (y2 -y1)/(x2-x1)
            # y = mx + b
            b = y1 - slope*(x1)

            for i in range(len(data.poses)):
                y = data.poses[i].pose.position.y
                x = data.poses[i].pose.position.x

                b_prime = y - slope*(x)

                b_diff = b - b_prime
                # b_prime_array.append(b_prime)

                if abs(b_diff) > threshold:
                    if np.sign(b_diff) == -1:
                        print("R")

                    elif np.sign(b_diff) == 1:
                        print("L")

                else:
                    print(" ")


    def full_plan_callback(self, data):


        # use global plan topic and determine the max/min y-value of the current
        # path plan. if the max/min y value exceeds a threshold, then that should
        # trigger either the left or right blinker.

        # Determine if a single variable should be returned or use if else statments
        # to send self.LEFT or self.RIGHT, or self.STRAIGHT for neither.



        x_array = []
        y_array = []

        threshold = 0.8
        # if len(data.poses) > 20:
        if self.turning == True:

            for i in range(0, 100):
                 y_array.append(data.poses[i].pose.position.y)

                 x_array.append(data.poses[i].pose.position.x)


            xavg = np.mean(x_array)
            yavg = np.mean(y_array)

            y_extreme = np.something(x_array)

            # print("yavg", yavg, "xavg", xavg)
            if yavg < (self.y_plan - threshold):
                 self.turning_pub.publish(-1)
                 # print("LEFT")
            elif yavg > (self.y_plan + threshold):
                 self.turning_pub.publish(1)
                 # print("RIGHT")
            else:
                 self.turning_pub.publish(0)
                     # print("STRAIGHT")
            # xavg = np.mean(x_array)
            # yavg = np.mean(y_array)
            # self.x_plan_array.append(xavg)
            # self.y_plan_array.append(yavg)
            #
            # # see about using the first and last posiiton of the list of local_plan to determine
            # # when to blink. In
            # x1, x2 = [data.poses[0].pose.position.x, data.poses[2].pose.position.x]
            # y1, y2 = [data.poses[0].pose.position.y, data.poses[2].pose.position.y]
            #
            # # print("dx", abs(x2-x1), "dy", abs(y2-y1))
            # slope = (y2 -y1)/(x2-x1)
            # # y = mx + b
            # b = y1 - slope*(x1)
            #
            # x_last = data.poses[-1].pose.position.x
            # y_last = data.poses[-1].pose.position.y
            #
            # y_expected = slope * x_last + b
            #
            # if y_last > y_expected:


        else:
            pass

    def cmd_vel_callback(self, data):

        #obtain the z angular velocity. Negative is clockwise, Positive is counter clockwise
        z = data.angular.z

        # threshold value to activate blinker (looks like 1 is good)
        threshold = 1
        if z > threshold:
            self.LEFT = True

        elif z < -1*threshold:
            self.RIGHT = True

        else:
            self.LEFT = False
            self.RIGHT = False


        # if self.LEFT:
        #     print("Left")
        # elif self.RIGHT:
        #     print("Right")
        # else:
        #     print("straight")




if __name__ == '__main__':
    nav = arStudyData()
