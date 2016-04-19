#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
  pcl::PCLPointCloud2::Ptr cloud_stat (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_voxel(new pcl::PCLPointCloud2);

  std::cerr << "PointCloud before filtering: " << input->width * input->height << " data points." << std::endl;

  //pass through filter
  float xmin, xmax, ymin, ymax, zmin, zmax;
  nh.getParam("xmin", xmin);
  nh.getParam("xmax", xmax);
  nh.getParam("ymin", ymin);
  nh.getParam("ymax", ymax);
  nh.getParam("zmin", zmin);
  nh.getParam("zmax", zmax);

	 //std::cerr << "xmin: " << xmin << std::endl;

  pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;
  pass_through_filter.setInputCloud (input);
  pass_through_filter.setFilterFieldName ("x");
  pass_through_filter.setFilterLimits (xmin, xmax);
  pass_through_filter.setFilterFieldName ("y");
  pass_through_filter.setFilterLimits (ymin, ymax);
  pass_through_filter.setFilterFieldName ("z");
  pass_through_filter.setFilterLimits (zmin, zmax);
  pass_through_filter.filter (*cloud_pass);

  //voxel grid filter
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
  voxel.setInputCloud(cloud_pass);
  voxel.setLeafSize(0.01f, 0.01f, 0.01f);
  voxel.filter(*cloud_voxel);

  //statistical ourlier removal filter
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> stat;
  stat.setInputCloud (cloud_voxel);
  stat.setMeanK (50);
  stat.setStddevMulThresh (1.0);
  stat.filter (*cloud_stat);

  // convert to default pointcloud template
  std::cerr << "PointCloud after filtering: " << cloud_stat->width * cloud_stat->height << " data points." << std::endl;

  // Publish the data
  pub.publish(*cloud_stat);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_filtering");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ( "output_filtered", 1);

  // Spin
  ros::spin ();
}
