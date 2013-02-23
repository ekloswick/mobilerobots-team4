	/************************************************************
 * Name: Robot_Chocolate.cpp
 ***********************************************************/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include "std_msgs/String.h"
#include <turtlebot_node/TurtlebotSensorState.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <cmvision/Blobs.h>
#include <cmvision/Blob.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Setup information
bool bump = false;
int state = 0;
unsigned int red = 0, green = 0, blue = 0;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
* Callback for subscription to sensor_state (which controls bump sensor)
*/
void bumpCallBack(const turtlebot_node::TurtlebotSensorState& msg){
	bump = msg.bumps_wheeldrops > 0;
    //If the bump sensor has been hit, then msg.bumps_wheeldrops will be > 0
}

void blobsCallBack(const cmvision::Blobs& blobsIn) {
	//for each blob in blobs[]
	int range = 15, lowestX = blobsIn.image_width, highestX = 0;
	for (int i = 0; i < blobsIn.blob_count; ++i)
	{
		// check for thin "false postiive" blobs
		if (blobsIn.blobs[i].area < 50)
		{
			continue;
		}

		if (blobsIn.blobs[i].red == red && blobsIn.blobs[i].green == green && blobsIn.blobs[i].blue == blue)
		{
			if (state < 0)
				return;

			// check if the bot is too close
			if (blobsIn.blobs[i].left < lowestX)
			{
				lowestX = blobsIn.blobs[i].left;
			}
			if (blobsIn.blobs[i].right > highestX)
			{
				highestX = blobsIn.blobs[i].right;
			}

			// turn left towards blob
			if (blobsIn.blobs[i].x < (blobsIn.image_width/2) - range)
			{
				//ROS_INFO("Moving right towards blob");
				state = 1;
			}
			// turn right towards blob
			else if (blobsIn.blobs[i].x > (blobsIn.image_width/2) + range)
			{
				//ROS_INFO("Moving left towards blob");
				state = 2;
			}
			// move straight towards blob
			else if ((blobsIn.blobs[i].x > (blobsIn.image_width/2) - range) && (blobsIn.blobs[i].x < (blobsIn.image_width/2) + range))
			{
				//ROS_INFO("Moving straight towards blob");
				state = 3;
			}
			// searching for a matching blob
			else
			{
				//ROS_INFO("NOPE! Searching...");
				state = 0;
			}
			
		}
	}

	// if blobs take half of screen width, back up
	if (highestX - lowestX < 0) {}
	else if (highestX - lowestX > blobsIn.image_width * 0.4)
	{
		state = -1;
	}
}

void cloudCallBack(const pcl::PointCloud::ConstPtr& cloud)
{
	//X,Y,Z of the centroid
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;

	//Number of points observed
	unsigned int n = 0;

	BOOST_FOREACH(const pcl::PoinstSYZ& pt, cloud->points)
	{
		if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
		{
			//Test to ensure the point is within the aceptable box.
        		if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        		{
          		//Add the point to the totals
				x += pt.x;
				y += pt.y;
				z += pt.z;
				n++;
        		}
		}
	}

	//If there are points, find the centroid and calculate the command goal.
    	//If there are no points, simply publish a stop goal.
    	if (n)
    	{ 
		x /= n; 
	 	y /= n; 
	 	z /= n;  
	 
	 	ROS_DEBUG("Centriod at %f %f %f with %d points", x, y, z, n);
	 
	 	geometry_msgs::Twist cmd;
	 	cmd.linear.x = (z - goal_z_) * z_scale_;
	 	cmd.angular.z = -x * x_scale_;
	 	cmdpub_.publish(cmd);
    	}
    	else
    	{
	 	ROS_DEBUG("No points detected, stopping the robot");
		state = -1;
		//cmdpub_.publish(geometry_msgs::Twist());
    	}
}

int main(int argc, char **argv){
	
	//srand(time(NULL));

	ROS_INFO("Starting kinect_intro.cpp");
	ros::init(argc, argv, "kinect_intro");// Allows ros to read arguments passed in.
	
	// Create handle that will be used for both subscribing and publishing. Also, use this to get variables from the comand line.
	ros::NodeHandle n;	
	
	// Set up the subscriber(s)
	ros::Subscriber sub1 = n.subscribe("/turtlebot_node/sensor_state", 1000, bumpCallBack);
	ros::Subscriber sub3 = n.subscribe("/blobs", 5, blobsCallBack);
	ros::Subscriber sub2 = n.subscribe("/camera/depth/points", 5, cloudCallBack);

	// Set up the publisher(s)
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
  	// Set the loop frequency in Hz.
	ros::Rate loop_rate(2);
	
	// Set up our movement.
	geometry_msgs::Twist t;

	// get parameters for the program	
	std::string bacon; 			
	n.getParam("seek_visit_order", bacon);

	// parse the parameter
	int order [3];
	int current = 0;
	order[0] = atoi(bacon.substr(0,1).c_str());
	order[1] = atoi(bacon.substr(2,1).c_str());
	order[2] = atoi(bacon.substr(4,1).c_str());


	while (ros::ok()) //runtime loop
	{
		// state: 0 = search, 1 = turn left towards blob, 2 = turn right towards blob, 3 = straight towards blob
		// state: -1 = stopped, <-1 = going backwards	
		if (state < -1) // going backwards
		{
			t.linear.x = -0.6; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = 0;
			state--;

			if (state <= -5)
			{
				state = 0;
				if (current == 3)
					exit(1);
			}
		}	
		else if (state == -1) // stopped
		{
			current++;
			t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = 0;
			state = -2;
		}		
		if (state == 0) // turn until color is found
		{
			t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = 0.2;
		}
		else if (state == 1)// turn left towards the found color
		{
			t.linear.x = 0.1; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = 0.1;	
		}
		else if (state == 2)// turn right towards the found color
		{
			t.linear.x = 0.1; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = -0.1;	
		}
		else if (state == 3)// go towards the found color
		{
			t.linear.x = 0.2; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = 0;	
		}

		// orange
		if (order[current] == 1)
		{
			red = 255; green = 0; blue = 0;
		}
		// green
		else if (order[current] == 2)
		{
			red = 0; green = 255; blue = 0;
		}
		// pink
		else if (order[current] == 3)
		{
			red = 0; green = 0; blue = 255;
		}
	 
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


