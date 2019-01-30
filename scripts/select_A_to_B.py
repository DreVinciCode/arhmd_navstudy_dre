#!/usr/bin/env python
'''
Andre Cleaver
12122018


Script to select points on the map on the RVIZ mapself. Points are published
to the topic /point_coordinate as a Point message.
'''

import rospy
from geometry_msgs.msg import Pose, Point, PointStamped

class point_select:

    def __init__(self):

        rospy.init_node('pointData', anonymous = True)
        self.second_point = True
        self.point_selection_sub = rospy.Subscriber("clicked_point", PointStamped, self.point_callback)
        self.point_selection_pub = rospy.Publisher("/point_coordinate", Point, queue_size=10)
        self.posA = 0
        rospy.spin()

    def point_callback(self,data):
        print("Point: " + str(round(data.point.x,2)) + ", " + str(round(data.point.y,2)))
        self.point_selection_pub.publish(x = data.point.x, y = data.point.y, z = 0.0)

        if self.second_point:
            print("Select End Point...")
            self.second_point = False

        else:
            self.second_point = True
            print("Select Starting Point...")


if __name__ == '__main__':

    print("Select Starting Point...")

    try:
        pt = point_select()
        (pt.posA)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
