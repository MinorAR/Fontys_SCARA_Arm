#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/pcd_io.h>
//transform
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
// passthrough filter
#include <iostream>
#include <pcl/filters/passthrough.h>
//move it

//global for merged cloud
pcl::PointCloud<pcl::PointXYZ> cloud_merged;
int count = 0;
class PCL_capturing
{
public:
  PCL_capturing()
  {
    // Create a ROS subscriber for the input point cloud
    sub_ = n_.subscribe("input", 1, &PCL_capturing::callback, this);
    // Publish to pointcloud_captured topic
    pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("pointcloud_captured", 1, true);
    //ros::spin();
  }

  void callback(const pcl::PCLPointCloud2ConstPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converted (new pcl::PointCloud<pcl::PointXYZ>);

    // get transform position of camera
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/world", "/camera_base_link", now, ros::Duration(3.0));
    listener.lookupTransform("/world", "/camera_base_link",ros::Time(0), transform);

    // convert pointcloud2 to pointxyz
    pcl::fromPCLPointCloud2(*input, *cloud_converted);

    // (rough) pass through filter
    pass_through_filter.setInputCloud (cloud_converted);
    pass_through_filter.setFilterFieldName ("x");
    pass_through_filter.setFilterLimits (-2,2);
    pass_through_filter.setFilterFieldName ("y");
    pass_through_filter.setFilterLimits (-2,2);
    pass_through_filter.setFilterFieldName ("z");
    pass_through_filter.setFilterLimits (0,3);
    pass_through_filter.filter (cloud_pass);
    // transform pointcloud
    pcl_ros::transformPointCloud (cloud_pass , cloud_tf, transform);
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
    // publish data
    pub_.publish (cloud_merged);//merged);
    // Write pcd file
    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> ("cloud_captured.pcd", cloud_merged, false);
    // Write ply file
    //pcl::PLYWriter plywriter;
    //plywriter.write<pcl::PointXYZ> ("cloud_captured.ply", cloud_merged, false);
    count++;
    // shutdown thread

    if (count < 5)
    {
    	ros::param::set("/pcl_capturing/finished", false);
	ros::shutdown();
    }
    else
    {
     	std::cerr << "cloud_merged published: " << cloud_merged << std::endl;
  	std::cerr << "pcl_capturing finished" << std::endl;
	ros::param::set("/pcl_capturing/finished", true);
        ros::waitForShutdown();
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;


  // callback function
  pcl::PointCloud<pcl::PointXYZ> cloud_pass, cloud_tf;
  pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
  // transform function
  tf::TransformListener listener;
  tf::StampedTransform transform;

};//End of class

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pointcloud_capturing");
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  std::cerr << "start pcl_capturing" << std::endl;
  for (int i = 1; i < 5; i++)
  {
  	std::cerr << "Move to goal: " << i << std::endl;
  	// move to goal
  	sleep(1);
  	//Create an object of class PCL_capturing that will take care of everything
  	PCL_capturing Object;
  	spinner.spin();
  }
  while (ros::ok())
{
  ros::spinOnce();
}
 //ros::waitForShutdown();
}

