#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
// voxel filter
#include <pcl/filters/voxel_grid.h>
// passthrough filter
#include <iostream>
#include <pcl/filters/passthrough.h>
// statistical outlier removal
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converted (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stat (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel (new pcl::PointCloud<pcl::PointXYZ>);

  // convert pointcloud2 to pointxyz
  pcl::fromPCLPointCloud2(*input, *cloud_converted);

  if (cloud_converted == 0)
  {
        std::cerr << "input is empty" << std::endl;
	return;
  }
  else
  {
        std::cerr << "cloud filtering start: " << cloud_converted << std::endl;
  }

  std::cerr << "PointCloud before filtering: " << cloud_converted->width * cloud_converted->height << " data points." << std::endl;

  //pass through filter
  float xmin, xmax, ymin, ymax, zmin, zmax;
  nh.getParam("/pcl_filtering/xmin", xmin);
  nh.getParam("/pcl_filtering/xmax", xmax);
  nh.getParam("/pcl_filtering/ymin", ymin);
  nh.getParam("/pcl_filtering/ymax", ymax);
  nh.getParam("/pcl_filtering/zmin", zmin);
  nh.getParam("/pcl_filtering/zmax", zmax);

  pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
  pass_through_filter.setInputCloud (cloud_converted);

//  pass_through_filter.setFilterFieldName ("x");
//  pass_through_filter.setFilterLimits (-1, 1);
//  pass_through_filter.setFilterFieldName ("y");
//  pass_through_filter.setFilterLimits (-1, 1);
//  pass_through_filter.setFilterFieldName ("z");
//  pass_through_filter.setFilterLimits (0, 3);
//  pass_through_filter.filter (*cloud_pass);

  pass_through_filter.setFilterFieldName ("x");
  pass_through_filter.setFilterLimits (xmin, xmax);
  pass_through_filter.setFilterFieldName ("y");
  pass_through_filter.setFilterLimits (ymin, ymax);
  pass_through_filter.setFilterFieldName ("z");
  pass_through_filter.setFilterLimits (zmin, zmax);
  pass_through_filter.filter (*cloud_pass);

  //statistical ourlier removal filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat;
  stat.setInputCloud (cloud_pass);
  stat.setMeanK (50);
  stat.setStddevMulThresh (1.0);
  stat.filter (*cloud_stat);

  //voxel grid filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud_stat);
  voxel.setLeafSize(0.01f, 0.01f, 0.01f);
  voxel.filter(*cloud_voxel);

  // convert to default pointcloud template
//  std::cerr << "PointCloud after filtering: " << cloud_stat->width * cloud_stat->height << " data points." << std::endl;

  // Publish the data
  pub.publish(cloud_voxel);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_filtering");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "output_filtered", 1, true);

  while (ros::ok())
  {
  ros::spinOnce();
  }
  std::cerr << "pcl_filtering finished" << std::endl;

  ros::shutdown();

}
