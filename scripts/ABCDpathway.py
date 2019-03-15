#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Int32
import numpy as np
import rosbag
import std_srvs.srv
from tf import transformations
import math

class GoToPose():
    def __init__(self):

        self.counter = 0
        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)

	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':

    # waypoints for the turtlebot, [x,y, theata]
    point_A = [-18.39, 3.97, 225]
    point_B = [-19.78, 1.63, 180]
    point_C = [-33.56, 1.63, 90]
    point_D = [-33.56, 3.74, 270]
    point_E = [-33.56, 1.63, 0]
    point_F = [-19.78, 1.63, 45]
    point_G = [-18.39, 3.97, 45]

    try:
        rospy.init_node('nav_test', anonymous=False)
        point_pub = rospy.Publisher("/point_ab", String, queue_size=10)
        command_sub = rospy.Publisher("/data_logging_commands", String, queue_size = 1)
        navigator = GoToPose()

        start_index = 0
        goal_index = start_index
        locations_names = ['A', 'B', 'C', 'D', 'E', 'F','G']
        num_location = 7

        location_coord = np.zeros([num_location,3])

        location_coord[0] = point_A
        location_coord[1] = point_B
        location_coord[2] = point_C
        location_coord[3] = point_D
        location_coord[4] = point_E
        location_coord[5] = point_F
        location_coord[6] = point_G


        while not rospy.is_shutdown():

            turtlebot_orientation_in_degrees = location_coord[goal_index][2]
            #convert euler to quaternion and save in new variable quat
            quat = transformations.quaternion_from_euler(0, 0, math.radians(turtlebot_orientation_in_degrees))
            # Customize the following values so they are appropriate for your location
            position = {'x': location_coord[goal_index][0], 'y' : location_coord[goal_index][1]}
            quaternion = {'r1' : quat[0], 'r2' : quat[1], 'r3' : quat[2], 'r4' : quat[3]}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, quaternion)

            if success:
                rospy.loginfo("Hooray, reached point " + locations_names[goal_index])
                point_pub.publish(locations_names[goal_index])
                rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty())
                rospy.wait_for_service('/move_base/clear_costmaps')

            else:
                rospy.loginfo("The base failed to reach point " + locations_names[goal_index])
                rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty())
                rospy.wait_for_service('/move_base/clear_costmaps')

            goal_index += 1

            # num_location
            if goal_index >= num_location:
                goal_index = 0

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
else:
    print("Failed")
