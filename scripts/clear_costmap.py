#!/usr/bin/env python
import sys
import rospy
import std_srvs.srv

from std_srvs.srv import *


# rospy.init_node('turtlesim', anonymous = True)

rospy.wait_for_service('/move_base/clear_costmaps')
try:
        clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        resp1 = clear_costmap()

except rospy.ServiceException, e:
    print "Service call failed: %s"%e

#rospy.wait_for_service('/clear', timeout = 2.0)

#call = rospy.ServiceProxy('/clear', std_srvs.srv.Empty())
# rospy.wait_for_service('~clear_costmaps')

print("clear costmap")
