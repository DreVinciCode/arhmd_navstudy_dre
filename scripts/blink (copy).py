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
        self.global_plan_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.global_plan_callback)

        self.position_callback = rospy.Subscriber("/point_ab", String, self.position_callback)


        # publisher to send either left, right or straight
        self.turning_pub = rospy.Publisher("/turn_signal", String, queue_size=10)

        rospy.spin()


    def position_callback(self, data):

    	self.point = data

    	if self.point == "A":
    		self.north = True

		if self.point == "B":
			self.north = False

    def signal(self, signal, bool):

    	if bool:

    		print(signal)   

    def global_plan_callback(self,data):
        # deviation from y-intercept
        threshold = 0.1
        b_array = []
     

        # use linear formula y = mx +b to determine a straight line from the robots
        # orientation. Using first and 4th point.
        x1, x2 = [data.poses[0].pose.position.x, data.poses[2].pose.position.x]
        y1, y2 = [data.poses[0].pose.position.y, data.poses[2].pose.position.y]
        slope = (y2 -y1)/(x2-x1)
        b = y1 - slope*(x1)

        for i in range(len(data.poses)):
            y = data.poses[-1].pose.position.y
            x = data.poses[-1].pose.position.x
     

            # determine whether if a nav point falls either on the left or right
            # side of the robots line.
            b_prime = (y - slope*(x)) 

            b_diff = b - b_prime

          
     	print(b_diff)

     	if self.north == True:
	        if abs(b_diff) > threshold:

	        	if np.sign(b_diff) == -1:

	        		self.signal("R", True)

	        	elif np.sign(b_diff) == 1:

	        		self.signal("L", True)
        		else:
        			pass

	    if self.north == False:

        	if abs(b_diff) > threshold:

	        	if np.sign(b_diff) == -1:

	        		self.signal("L", True)

	        	elif np.sign(b_diff) == 1:

	        		self.signal("R", True)
	
	        	else:
	        		pass


        		# rospy.sleep(5.)
        		# self.signal("R", False)
  #       if b_max > abs(b_min):

  #       	if b_max > threshold:
  #       		print(b_max, "R")

  #       	else:
  #       		print("S")

	  
		# if abs(b_min) > b_max:

		# 	if abs(b_min) > threshold:
		# 		print(b_min,"L")

		# 	else:
		# 		print("S")

        # b_diff = b - b_prime

        # if abs(b_diff) > threshold:
        #     if np.sign(b_diff) == -1:


        #         print("R")
        #         self.turning_pub.publish("R")

        #     elif np.sign(b_diff) == 1:
        #         print("L")
        #         self.turning_pub.publish("L")

        # else:
        #     print("S")
        #     self.turning_pub.publish("S")


if __name__ == '__main__':
    nav = arStudyData()
