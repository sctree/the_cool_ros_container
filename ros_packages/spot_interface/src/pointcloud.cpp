/** @package spot_interface
* 
* \file pointcloud.cpp
* \brief file to transform the point cloud in the correct frame
* 
* \author Zoe Betta
* \version 1.0
* \date 25/03/2024
* \details
* 
* Subscribes to: <BR>
* /points2
*
* Publishes to: <BR>
* /tf_points2
* 
* Description: <BR>
* this file receives the pointcloud in a static frame and tranforms
* each point into a new frame that is based on the position and oreintation of 
* the robot at the specific time instance
*/


#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include <string>
#include <iostream>
#include <sstream>


ros::Publisher tf_pub;
tf::TransformListener *tf_listener; 
int actual_floor;


/**
* \brief it tranforms the pointcloud
* \param msg: it is the received pointcloud
*
* \return None
*
* this function reads the tree frame and of reach pointcloud received transforms it with respect to the position 
* of the robot in space
*/

void pointcloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
       sensor_msgs::PointCloud2 pcl_out;
	char s1[20]= "velodyne_base_link";
 	//char s2[10]="number";
	//std::stringstream ss;
	//ss<<actual_floor;
	//ss>>s2;
	//std::strcat(s1,s2);
       tf_listener->waitForTransform(s1 ,(*msg).header.frame_id, (*msg).header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud(s1, *msg, pcl_out , *tf_listener);
       tf_pub.publish(pcl_out);
}


/**
* \brief it initialize the rosnode
* \param None
* \return None
*
* this function initializes the publisher, subscribers and the tf_listener
*/
int main(int argc, char** argv)
{
ros::init(argc, argv, "mapconversion");
ros::NodeHandle n;
ros::NodeHandle nh;
ros::NodeHandle n1;
//ros::param::get("~floor", actual_floor);

//std::cout << actual_floor << std::endl;

ros::Subscriber sub = n.subscribe("/points2", 1000, pointcloudcallback);
tf_pub = nh.advertise<sensor_msgs::PointCloud2> ("tf_points2", 1000);


tf_listener    = new tf::TransformListener();

ros::spin();
}



















