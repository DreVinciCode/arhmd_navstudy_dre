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

class GoToPose():
    def __init__(self):

        self.posA = [0, 0]
        self.posB = [0, 0]
        self.counter = 0
        self.goal_sent = False
        rospy.Subscriber("/point_coordinate", Point, self.coordinate_callback)


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

    def coordinate_callback(self, data):

        if self.counter == 0:
            self.posA = [data.x, data.y]
            print("Point A: " + str(self.posA))
            self.counter = 0

        elif self.counter == 1:
            self.posB = [data.x, data.y]
            print("Point B: " + str(self.posB))
            self.counter = 0
        else:
            pass
        self.counter += 1

if __name__ == '__main__':

    zero  = [0, 0]

    try:
        rospy.init_node('nav_test', anonymous=False)
        point_pub = rospy.Publisher("/point_ab", String, queue_size=10)
        command_sub = rospy.Publisher("/data_logging_commands", String, queue_size = 1)
        navigator = GoToPose()

        print("Run select_A_to_B.py script and select points.")

        print(navigator.posA, navigator.posB)
        '''
            This is to make sure that both points have been obtained.
        '''

        A = navigator.posA
        B = navigator.posB

        start_index = 0
        goal_index = start_index
        locations_names = ['A', "B"]
        num_location = 2

        location_coord = np.zeros([num_location,2])

        location_coord[0] = A
        location_coord[1] = B

        while not rospy.is_shutdown():

            # Customize the following values so they are appropriate for your location
            position = {'x': location_coord[goal_index][0], 'y' : location_coord[goal_index][1]}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, quaternion)

            if success:
                rospy.loginfo("Hooray, reached point " + locations_names[goal_index])
                point_pub.publish(locations_names[goal_index])

            else:
                rospy.loginfo("The base failed to reach point " + locations_names[goal_index])


            goal_index += 1

            if goal_index >= num_location:
                goal_index = 0

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
else:
    print("Failed")
