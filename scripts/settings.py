#!/usr/bin/env python
from __future__ import print_function
import rospy
from dynamic_reconfigure.msg import Config
import dynamic_reconfigure.client


class setParameters():
    def __init__(self):
        rospy.init_node('parameter_set', anonymous=True)

        control_sub = rospy.Subscriber("/navigation_velocity_smoother/parameter_updates", Config, self.controller_callback)
        control_pub = rospy.Publisher("/navigation_velocity_smoother/parameter_updates", Config, queue_size=1)


        client = dynamic_reconfigure.client.Client("/navigation_velocity_smoother")
        params = { 'speed_lim_w' : 0.3, 'accel_lim_w' : 0.2, 'speed_lim_v' : 0.5 }
        config = client.update_configuration(params)

    def controller_callback(self, data):

        print(data.doubles[1].value)

    def test(self,w):

        self.set.doubles[1].value = w
        print(self.set)

if __name__ == '__main__':
    controller = setParameters()
