#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

bool heard_new_msg = false;
bool heard_reply = false;

sensor_msgs::LaserScan laser_msg;
sensor_msgs::LaserScan laser_msg_reply;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  laser_msg = *msg;
  heard_new_msg = true;
}


void reply_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  //laser_msg = *msg;
  heard_reply = true;
  laser_msg_reply = *msg;
  
  ROS_INFO("Heard reply!");
}

void waitForReply(){
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		ros::spinOnce();

		if (heard_reply)
			return;

		loop_rate.sleep();
	}
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "latency_test_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  
  ros::Subscriber sub_sensor = n.subscribe("/scan", 1, laser_callback);
  ros::Subscriber sub_reply = n.subscribe("/laser_from", 1, reply_callback);
  
  
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("/laser_to", 1000);

  ros::Rate loop_rate(5);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
   
  double latency_avg = 0;
   
  int count = 0;
  while (ros::ok())
  {
    

    ros::spinOnce();
    
    if (heard_new_msg){
		//publish the message to the AR device
		ROS_INFO("\n\nSending message to AR device with seq id = %d",laser_msg.header.seq);
		ROS_INFO_STREAM(laser_msg.header);
		double secs_begin =ros::Time::now().toSec();
		pub.publish(laser_msg);
		
		//then wait for response
		heard_reply = false;
		waitForReply();
		double secs_end =ros::Time::now().toSec();
		
		ROS_INFO("Heard reply with seq = %d",laser_msg_reply.header.seq);
		ROS_INFO_STREAM(laser_msg_reply.header);
		
		heard_new_msg = false;
		
		double latency = secs_end - secs_begin;
		
		latency_avg = 0.9*latency_avg + 0.1*latency;
		ROS_INFO("Average latency = %f",latency_avg);
	}

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
