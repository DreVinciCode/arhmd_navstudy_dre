#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
from std_msgs.msg import Float64, String, Int8
from nav_msgs.msg import Path


class arStudyData:

    def __init__(self):

        rospy.init_node('blinker', anonymous = True)

        self.point = ""
        self.north = False
        
        # subscriptoin to global path plan
        # self.global_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.global_plan_callback)

        self.arfuros_path_sub = rospy.Subscriber("/ARFUROS/Path", Path, self.path_callback)

        self.position_callback = rospy.Subscriber("/point_ab", String, self.position_callback)


        # publisher to send either left, right or straight
        self.turning_pub = rospy.Publisher("/turn_signal", String, queue_size=10)

        rospy.spin()


    def position_callback(self, data):

    	self.point = data.data
    	print(self.point)

    	if self.point == "B":
    		self.north = True
    		print("Facing North")

    	elif self.point == "C":
 			self.north = False
			print("Facing South")   		
    	else:
		 	pass

    def path_callback(self, data):

        total_x = 0
        total_y = 0

        path = data.poses


        if len(path) > 10:
            for i in range(0,10):

                x = data.poses[i].pose.position.x
                y = data.poses[i].pose.position.y

                total_x = total_x + x
                total_y = total_y + y


            if (total_x > 0):

                 print("left" , total_x)

            else:
                print("right", total_x)


        # self.turning_pub.publish("R")
        # self.turning_pub.publish("L")
        # self.turning_pub.publish("S")










if __name__ == '__main__':
    nav = arStudyData()
