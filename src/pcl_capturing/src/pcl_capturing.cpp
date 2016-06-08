#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
//transform includes
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
// combine pointclouds
//#include "pcl_ros/io/concatenate_fields.h"
// passthrough filter
#include <iostream>
#include <pcl/filters/passthrough.h>
//move it
#include <pcl_capturing/position_request.h>
#include <moveit_msgs/MoveGroupActionResult.h>

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pointcloud_capturing");
  ros::NodeHandle n_;

  // start service to position arm
  ros::ServiceClient client = n_.serviceClient<pcl_capturing::position_request>("/request_position");
  pcl_capturing::position_request srv;

  // set finish status to false
  ros::param::set("pcl_capturing/finished", false);

  std::cerr << "start pcl_capturing" << std::endl;
  // init pointcloud publisher
  ros::Publisher pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("pointcloud_captured", 1, true);

  // positions for arm to move to
  double posX[] = {-0.1, 0.155, -0.1, 0.155}; //0.1 = error
  double posY[] = {0.992, 0.992, 0.992, 0.992};
  double posZ[] = {0.5, 0.5, 0.7, 0.7};
  double oriX[] = {0, 0, 0, 0};
  double oriY[] = {0, 0, 0, 0};
  double oriZ[] = {0, 0, 0, 0};
  double oriW[] = {1, 1, 1, 1};

   // make pointcloud pointer and wait for message on input topic.
  sensor_msgs::PointCloud2::ConstPtr input_cloud; 

  // init variables
  pcl::PointCloud<pcl::PointXYZ> cloud_pass,  cloud_merged, cloud_tf;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converted (new pcl::PointCloud<pcl::PointXYZ>);

  // init objects
  // init passthrough filter
  pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
  // time for transform
  ros::Time now = ros::Time::now();
  // transform object
  tf::TransformListener listener;
  
  // delay for starting up move node etc.
  sleep(10);

  // main for loop while ros node is ok  
  while (ros::ok())
  {
	// run four times to capture four pointclouds
  	for (int i = 0; i < 4; i++)
  		{
  		std::cerr << "Move to goal: " << i << std::endl;
		// move to goal
		srv.request.pose.position.x = posX[i];
		srv.request.pose.position.y = posY[i];
		srv.request.pose.position.z = posZ[i];
		srv.request.pose.orientation.x = oriX[i];
		srv.request.pose.orientation.y = oriY[i];
		srv.request.pose.orientation.z = oriZ[i];
		srv.request.pose.orientation.w = oriW[i];
		if(!client.call(srv))
			{
			ROS_INFO("There was some error on the service call");
			}
		// sleep /wait to reach position
		ros::topic::waitForMessage<moveit_msgs::MoveGroupActionResult>("/move_group/result", ros::Duration(50.0));
		std::cerr << "Goal Reached: " << i << std::endl;
  		sleep(7);
		
		// position reached, capture pointcloud now
		// wait for input cloud for 5 sec
		input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("input", ros::Duration(5.0));

    		// convert pointcloud2 to pointxyz
    		pcl::fromROSMsg(*input_cloud, *cloud_converted);

		// (rough) pass through filter
    		pass_through_filter.setInputCloud (cloud_converted);
    		pass_through_filter.setFilterFieldName ("x");
    		pass_through_filter.setFilterLimits (-1,1);
    		pass_through_filter.setFilterFieldName ("y");
    		pass_through_filter.setFilterLimits (-1,1);
    		pass_through_filter.setFilterFieldName ("z");
    		pass_through_filter.setFilterLimits (-1,1);
   		pass_through_filter.filter (cloud_pass);
			
		// wait max 2 sec for transform
 		//listener.waitForTransform("/world", "/camera_link", now, ros::Duration(2.0));
				
    		// transform pointcloud
		pcl_ros::transformPointCloud ("/world", cloud_pass, cloud_tf, listener);
						
    		// merge pointcloud
        	if (cloud_merged.points.size () == 0)
    			{
			cloud_merged = cloud_tf;
			std::cerr << "first cloud" << std::endl;
    			}
    		else
    			{
			cloud_merged += cloud_tf;
			std::cerr << "cloud added" << std::endl;
    			}

		//std::cerr << cloud_merged << std::endl;
		// publish pointcloud
    		pub_.publish (cloud_merged);
		}

	std::cerr << "PCL_capturing Finished" << std::endl;
	// set finish status to true
  	ros::param::set("pcl_capturing/finished", true);
        // wait for ros to shutdown this node
	ros::waitForShutdown();	
  }
}

