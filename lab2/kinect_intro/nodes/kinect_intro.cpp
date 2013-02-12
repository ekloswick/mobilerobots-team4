/************************************************************
 * Name: Jailbreak.cpp
 * Original Author: Chad Jenkins
 * C++ Port: Chas Jhin
 * Date: 1/20/2012
 *
 * Purpose: This program will run the robot in a straight line
 *          until it stops.
 ***********************************************************/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include "std_msgs/String.h"
#include <turtlebot_node/TurtlebotSensorState.h>
#include <string>
#include <time.h>

bool bump = false;

/**
* Callback for subscription to sensor_state (which controls bump sensor)
*/
void bumpCallBack(const turtlebot_node::TurtlebotSensorState& msg){
	bump = msg.bumps_wheeldrops > 0;
    //If the bump sensor has been hit, then msg.bumps_wheeldrops will be > 0
}

void blobsCallBack(const cmvision::Blobs& blobIn) {
	//blobsIn.blob_count = blobsIn.size();
	//blobsIn.blobs[] =

	 
}

int main(int argc, char **argv){
	
	srand(time(NULL));

	ROS_INFO("Starting kinect-intro.cpp");
	ros::init(argc, argv, "kinect-intro");// Allows ros to read arguments passed in.
	
	// Create handle that will be used for both subscribing and publishing. Also, use this to get variables from the comand line.
	ros::NodeHandle n;	
	
	// Set up the subscriber(s)
	ros::Subscriber sub1 = n.subscribe("/turtlebot_node/sensor_state", 1000, bumpCallBack);
	ros::Subscriber sub2 = n.subscribe("/blobs", set[2], blobsCallBack);

	// Set up the publisher(s)
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
  	// Set the loop frequency in Hz.
	ros::Rate loop_rate(10);
	
	// Set up our movement.
	geometry_msgs::Twist t;

	// State Counter
	int state = 0;
	float random;

	while (ros::ok()) //runtime loop
	{
		/*if (bump == true || state > 0) {
			if (bump == true) {
				state = 15;
				random = (rand()%2-0.5)*2;
			}

			if (state > 10) {
				ROS_INFO("hello create, you have bumped into something");
				t.linear.x = -1.0; t.linear.y = 0; t.linear.z = 0;
				t.angular.x = 0; t.angular.y = 0; t.angular.z = 0;
				state--;
			} else if (state > 0) {
				ROS_INFO("THIS IS THE TURN STATJIEOFJIDSOFJDSIOG");
				t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;
				t.angular.x = 0; t.angular.y = 0; t.angular.z = random;
				state--;
			}
			bump = false;
		}
		else {
			ROS_INFO("hello create, you can spin now");
			t.linear.x = 1; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = 0;
		}*/


		ROS_INFO("HELLO THERE!");
	 
		// Publish to our geometry message.
		twist_pub.publish(t);
		
		// Call our callback functions at this point if messages are available.
		ros::spinOnce();
	 
    // Sleep for remaining leftover time in a cycle.
		loop_rate.sleep();
	}

	ROS_INFO("Program Interrupted!  Shutting Down...\n");
	return 0;
}


