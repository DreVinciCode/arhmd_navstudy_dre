#!/usr/bin/env python
from __future__ import print_function
import rosbag
import rospy
import sys, time
import numpy as np
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


class arStudyData:

    def __init__(self):

        rospy.init_node('poseData', anonymous = True)
        self.current_filename = ""
        self.recording = False
        self.counter = 1
        self.info_ready = False
        self.counter_global = 0
        self.counter_local = 0
        self.counter_navfnROS = 0


        # subscription to current reached goal notified by labeled point.
        self.currentpoint_sub = rospy.Subscriber("/point_ab", String, self.callback_point)

        # subscription to participart infomation (name and modal)
        self.command_sub = rospy.Subscriber("/data_logging_commands",String,self.command_callback)

        # # subscription to position topic
        # self.pose_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, self.callback_pose)

        # # subscription to odometry topic
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        #

        # subscription to global path plan
        self.global_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.global_plan_callback)

        # subscriptoin to local path plan
        self.local_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self.local_plan_callback)
        #
        # # subscription to navfnROS path plan
        # self.navfnRos_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.navfnRos_callback)
        #
        # # subscription to command velocity topic
        # self.cmd_vel_sub = rospy.Subscriber("/cmd_vel_mux/input/navi", Twist, self.cmd_vel_callback)

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


    # def callback_pose(self,data):
    #
    #     if self.recording == True:
    #         self.bag.write("/amcl_pose",data)
    #     else:
    #         pass
    #
    # def odom_callback(self,data):
    #
    #    if self.recording == True:
    #        self.bag.write('/odom',data)
    #    else:
    #        pass

    def global_plan_callback(self, data):

       self.counter_global += 1
       # print("Global replan: " + str(self.counter_global))
       if self.recording == True:
           self.bag.write("/move_base/DWAPlannerROS/global_plan", data)
       else:
           pass

    def local_plan_callback(self, data):

       self.counter_local += 1
       #print("Local replan: " + str(self.counter_local))

       if self.recording == True:
           self.bag.write("/move_base/DWAPlannerROS/local_plan", data)

       else:
           pass

    # def navfnRos_callback(self, data):
    #
    #     self.counter_navfnROS += 1
    #
    #     #print("Full Path replan: " + str(self.counter_navfnROS))
    #     if self.recording == True:
    #         self.bag.write("/move_base/NavfnROS/plan", data)
    #     else:
    #         pass


    #
    # def cmd_vel_callback(self,data):
    #
    #     if self.recording == True:
    #         #/mobile_base/commands/velocity
    #         self.bag.write('/cmd_vel_mux/input/navi', data)
    #     else:
    #         pass



if __name__ == '__main__':
    nav = arStudyData()
