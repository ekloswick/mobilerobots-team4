/************************************************************
 * Name: Robot_Chocolate.cpp
 * By:   Eli and Craig
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
unsigned int timer = 0;
float rando = 1.0;

//X,Y,Z of the centroid
double minDistanceZ = 100.0, maxDistanceZ = 0.0;
double x = 0.0, y = 0.0, z = 0.0;
double min_y_ = -0.5;
double max_y_ = 0.5;
double min_x_ = -0.5;
double max_x_ = 0.5;
double max_z_ = 0.5;
double goal_z_ = 0.3;
double z_scale_ = 1.0;
double x_scale_ = 5.0;

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
		if (blobsIn.blobs[i].area < 100)
		{
			continue;
		}

		if (blobsIn.blobs[i].red == red && blobsIn.blobs[i].green == green && blobsIn.blobs[i].blue == blue)
		{
			if (state < 0 || state > 3)
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

	// if blobs take half of screen width (blob is found), back up
	if (highestX - lowestX < 0) {}
	else if (highestX - lowestX > blobsIn.image_width * 0.35)
	{
		timer = 0;
		state = -1;
	}
}

void cloudCallBack(const PointCloud::ConstPtr& cloud)
{
	x = 0.0; y = 0.0; z = 0.0;
	double minDistanceZ = 100.0, maxDistanceZ = 0.0;

	// Number of points observed
	unsigned int n = 0;

	BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points)
	{
		if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
		{
			// Test to ensure the point is within the aceptable box.
        		if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        		{
				if (pt.z < minDistanceZ)
				{
					x = pt.x;
					y = pt.y;
					minDistanceZ = z;
				}
				else if (pt.z > maxDistanceZ)
					maxDistanceZ = z;

				//ROS_INFO("%d %d %d\n", pt.x, pt.y, pt.z);

          		// Add the point to the totals
				//x += pt.x;
				//y += pt.y;
				//z += pt.z;
				n++;
        		}
		}
	}

	// If there are points and the difference in the farthest/closest object is large enough
    	if (n > 5000 && (maxDistanceZ - minDistanceZ) > 0.4)
    	{
	 	//ROS_INFO("n = %d, x = %f, y = %f\n", n, x, y);

		// try to avoid the object by turning away from it
		if (state == 1 || state == 2 || state == 3)
		{
			if (x < 0.0)
				state = 2;
			else if (x > 0.0)
				state = 1;
		}

		// old code that may be useful in the future
		/*if (!(state == 1 || state == 2 || state == 3 || state == 5 || state == 6))
		{
			state = 5;
			rando = (rand()%2-0.5)*2.0;
		}*/
		
    	}
}

int main(int argc, char **argv){
	
	srand(time(NULL));

	ROS_INFO("Starting robot_chocolate.cpp");
	ros::init(argc, argv, "robot_chocolate");// Allows ros to read arguments passed in.
	
	// Create handle that will be used for both subscribing and publishing. Also, use this to get variables from the comand line.
	ros::NodeHandle n;

	// if any params passed in, set them, else keep at default values
	if (n.getParam("min_y_", min_y_)) {}
	if (n.getParam("max_y_", max_y_)) {}
	if (n.getParam("min_x_", min_x_)) {}
	if (n.getParam("max_x_", max_x_)) {}
	if (n.getParam("max_z_", max_z_)) {}
	if (n.getParam("goal_z_", goal_z_)) {}
	if (n.getParam("z_scale_", z_scale_)) {}
	if (n.getParam("x_scale_", x_scale_)) {}

	// Set up the subscriber(s)
	ros::Subscriber sub1 = n.subscribe("/turtlebot_node/sensor_state", 1000, bumpCallBack);
	ros::Subscriber sub3 = n.subscribe("/blobs", 5, blobsCallBack);
	ros::Subscriber sub2 = n.subscribe("/camera/depth/points", 5, cloudCallBack);
	
	// Set up the publisher(s)
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
  	// Set the loop frequency in Hz.
	int hertz = 5;
	ros::Rate loop_rate(hertz);
	
	// Set up our movement.
	geometry_msgs::Twist t;

	// get parameters for the program	
	std::string bacon; 			
	n.getParam("seek_visit_order", bacon);

	// current number of blob to go to
	int current = 0;

	// read in blob params
	// we know the format of bacon must always be "1 3 2 3 1" with an odd number of characaters
	// make baconsize an even number, then divide that number by two to find how many blobs to find
	int baconsize = bacon.length();
	if (baconsize % 2 == 1) 
	{
		baconsize = (baconsize + 1)/2;
	}

	std::vector<int> order;
	for(int i = 0; i < baconsize; i++)
	{
		order.push_back(atoi(bacon.substr(i*2,1).c_str()));
	}

	while (ros::ok()) //runtime loop
	{
		//ROS_INFO("State = %d, Current = %d, order.size() = %d, Rando = %f\n", state, order[current], order.size(), rando);
		// state: 0 = search, 1 = turn left towards blob, 2 = turn right towards blob, 3 = straight towards blob
		// state: 4 = backup then randturn, 5 = randoturn + forward, 6 = backup then search, 7 = ???
		// state: -1 = stopped
		if (order[current] >= 10) // try to prevent the weird 'current = 32562' error
			exit(1);
		else if (state == -1) // stopped
		{
			t.linear.x = 0;
			t.angular.z = 0;

			// if all waypoints have been found
			if (current >= order.size())
			{
				ROS_INFO("Blob %d found!\n", order[current]);
				ROS_INFO("Last blob found! ALL DONE!\n");
				exit(1);
			}
			else
			{
				ROS_INFO("Blob %d found!\n", order[current]);
				if (current < order.size())
					++current;
				state = 5;
				rando = (rand()%2-0.5)*2.0;
			}
		}		
		else if (state == 0) // turn until color is found (search)
		{
			++timer;
			t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = -rando*0.4;

			if (timer >= 15*hertz)
			{
				timer = 0;
				state = 5;
				rando = (rand()%2-0.5)*2.0;
			}
		}
		else if (state == 1) // turn left towards the found color
		{
			t.linear.x = 0.1;
			t.angular.z = 0.1;	

			if (bump == true)
			{
				timer = 0;
				state = 4;
				bump = false;
			}
		}
		else if (state == 2) // turn right towards the found color
		{
			t.linear.x = 0.1; t.linear.y = 0; t.linear.z = 0;
			t.angular.x = 0; t.angular.y = 0; t.angular.z = -0.1;	
		
			if (bump == true)
			{
				timer = 0;
				state = 4;
				bump = false;
			}			
		}
		else if (state == 3) // go towards the found color
		{
			t.linear.x = 0.2;
			t.angular.z = 0;

			if (bump == true)
			{
				timer = 0;
				state = 4;
				bump = false;
			}	
		}
		else if (state == 4) // backup mode
		{
			++timer;
			t.linear.x = -0.3;
			t.angular.z = 0;
			
			if (timer >= hertz)
			{
				timer = 0;
				state = 5;
				rando = (rand()%2-0.5)*2.0;
			}
		}
		else if (state == 5) // rando_turn + small_forward mode
		{		
			++timer;
	
			if (bump == true) // if bumped, go to backup_search
			{
				timer = 0;
				state = 6;
				bump = false;
			}
			else if (timer <= hertz) // rando turn
			{	
				t.linear.x = 0;
				t.angular.z = rando;
			}
			else if (timer < 4*hertz) // small forward
			{
				t.linear.x = 0.2;
				t.angular.z = 0;
			}
			else if (timer >= 4*hertz) // go to search
			{
				timer = 0;
				state = 0;
			}
		}
		else if (state == 6) // backup_search mode
		{
			timer += 2;
			t.linear.x = -0.3;
			t.angular.z = 0;
			
			if (timer >= hertz)
			{
				timer = 0;
				state = 0;
			}
		}
		else if (state == 7) // follower mode, not used
		{
		 	t.linear.x = (z - goal_z_) * z_scale_;
		 	t.angular.z = -x * x_scale_;
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


