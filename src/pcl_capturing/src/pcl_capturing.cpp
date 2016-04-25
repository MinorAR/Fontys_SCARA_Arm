#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
//transform
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
// passthrough filter
#include <iostream>
#include <pcl/filters/passthrough.h>
//move it

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZ> transform_pointcloud (pcl::PointCloud<pcl::PointXYZ> cloud_in)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  pcl::PointCloud<pcl::PointXYZ> cloud_tf;
  // get transform position of camera
  ros::Time now = ros::Time::now();
  listener.waitForTransform("/base_link", "/camera_link", now, ros::Duration(3.0));
  listener.lookupTransform("/base_link", "/camera_link",ros::Time(0), transform);
  // transform pointcloud
  pcl_ros::transformPointCloud (cloud_in , cloud_tf, transform);
  return cloud_tf;
}

void cloud_state (const pcl::PCLPointCloud2ConstPtr& input)
{
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converted (new pcl::PointCloud<pcl::PointXYZ>);
  // convert pointcloud2 to pointxyz
  pcl::fromPCLPointCloud2(*input, *cloud_converted);

  // pass through filter
  pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
  pass_through_filter.setInputCloud (cloud_converted);
  float limits [6]; //xmin, xmax, ymin, ymax, zmin, zmax; from parameter server
  std::string axisName [6] = {"xmin", "xmax", "ymin", "ymax", "zmin", "zmax"};
  for (int t = 1; t < 7; t++)
  {
    nh.getParam( axisName [t] , limits [t]);
  }
  pass_through_filter.setFilterFieldName ("x");
  pass_through_filter.setFilterLimits (limits [1], limits [2]);
  pass_through_filter.setFilterFieldName ("y");
  pass_through_filter.setFilterLimits (limits [3], limits [4]);
  pass_through_filter.setFilterFieldName ("z");
  pass_through_filter.setFilterLimits (limits [5], limits [6]);

  // for loop
  pcl::PointCloud<pcl::PointXYZ> cloud_merged, cloud_pass;
  pcl::PointCloud<pcl::PointXYZ> cloud_tf [4];

  for (int i = 1; i < 5; i++)
  {
	std::cerr << "Move to goal: " << i << std::endl;
	// move to goal

	// apply passthrough filter
	pass_through_filter.filter (cloud_pass);
	// transform the pointcloud
	cloud_tf [i] = transform_pointcloud(cloud_pass);
	// merge pointcloud
	if (i == 1) {
	cloud_merged = cloud_tf [i];
	}
	else {
	cloud_merged += cloud_tf [i];
	}
  }
  std::cerr << "publish merged cloud" << std::endl;
  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("cloud_merged.pcd", cloud_merged, false);
  // write ply file
  pcl::PLYWriter plywriter;
  plywriter.write<pcl::PointXYZ> ("cloud_merged.ply", cloud_merged, false);
  // Publish the data.
  pub.publish (cloud_merged);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_capturing");
  ros::NodeHandle nh;
  std::cerr << "start pcl_capturing" << std::endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_state);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("pointcloud_merged", 1);
  // Spin
  ros::spin ();
}
