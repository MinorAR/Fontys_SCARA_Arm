#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
// triangulation
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

// kd search
//#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

// markers
#include <visualization_msgs/Marker.h>
#include <cmath>

// workspace dimensions
const double Xmin = -0.3;
const double Xmax = 0.3;
const double Ymin = 0.5;
const double Ymax = 0.5;
const double Zmin = 0.3;
const double Zmax = 1;
const double Zresolution = 0.05;
const double Xresolution = 0.01;

ros::Publisher pub;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converted (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract (new pcl::PointCloud<pcl::PointXYZ>);

  // Object for storing the normals.
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  // Object for storing both the points and the normals.
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);

  // convert pointcloud2 to pointxyz
  pcl::fromPCLPointCloud2(*input, *cloud_converted);

  // Create visual publisher
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>( "visualization_path_marker", 0 , true);

  bool start;
  nh.getParam("/pcl_filtering/finished" , start);
  bool finished;
  nh.getParam("/pcl_extraction/finished" , finished);

  if (start == false && finished == false)
  {
        std::cerr << "pcl_extraction: start signal NOT received" << std::endl;
        return;
  }
  else if (cloud_converted == 0)
  {
       	std::cerr << "input pcl_extraction is empty" << std::endl;
	return;
  }
  else
  {
        std::cerr << "pcl_extraction start" << std::endl;
  }

  // Normal estimation.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud_converted);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromPCLPointCloud2(*input, *cloud_search);
  
  // set y to zero to search only for x and z
  for (size_t i = 0; i < cloud_search->points.size (); ++i)
  {
    cloud_search->points[i].y = 0;
  }

 // K nearest neighbor search

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1;
  kdtree1.setInputCloud (cloud_search);
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // make marker
  visualization_msgs::Marker path, arrow;
	path.header.frame_id = arrow.header.frame_id = "world";
	path.header.stamp = arrow.header.stamp = ros::Time();
	path.ns = arrow.ns = "Path";
	path.id = 1;
	arrow.id = 2;
	path.type = visualization_msgs::Marker::LINE_STRIP;
	arrow.type = visualization_msgs::Marker::ARROW;
	path.action = arrow.action = visualization_msgs::Marker::ADD;

	path.pose.orientation.w = 1.0;
	
	//arrow.pose.orientation.w = 1.0; //for test (without normal direction)
	path.scale.x = 0.01;
	path.color.a = 0.9; // Don't forget to set the alpha!
	path.color.r = 0.0;
	path.color.g = 1.0;
	path.color.b = 0.0;

 	path.lifetime = arrow.lifetime = ros::Duration();
	path.scale.x = 0.01;
	path.color.a = 0.9; // Don't forget to set the alpha!
	path.color.r = 0.0;
	path.color.g = 0.0;
	path.color.b = 1.0;

			
  // search kdtree for closest point to path
  pcl::PointXYZ searchPoint;
  volatile double x;
  unsigned int state = 0;

  for (double z = Zmin; z <= Zmax; z = z + Zresolution )
  {
 	searchPoint.z = z;
	switch(state)
	{
	case 0:
		for (x = Xmin ; x <= Xmax; x = x + Xresolution )
        	{
                searchPoint.x = x;
                std::cout << "K nearest neighbor search at (" << searchPoint.x
                << " " << searchPoint.z
                << ") with K=" << K << std::endl;

                if ( kdtree1.nearestKSearchT (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
			{
                        std::cout << "    "  << cloud_converted->points[ pointIdxNKNSearch[i] ].x 
                        << " " << cloud_converted->points[ pointIdxNKNSearch[i] ].y
                        << " " << cloud_converted->points[ pointIdxNKNSearch[i] ].z
                        << " (squared distance: " << pointNKNSquaredDistance[i] 
			<< pointNKNSquaredDistance[i] << ")" << std::endl;
			
			path.pose.position.x = arrow.pose.position.x = cloud_converted->points[ pointIdxNKNSearch[i] ].x;
			path.pose.position.y = arrow.pose.position.y = cloud_converted->points[ pointIdxNKNSearch[i] ].y;
			path.pose.position.z = arrow.pose.position.z = cloud_converted->points[ pointIdxNKNSearch[i] ].z;
			arrow.pose.orientation.x = normals->points[ pointIdxNKNSearch[i] ].normal_x;
			arrow.pose.orientation.y = normals->points[ pointIdxNKNSearch[i] ].normal_y;
			arrow.pose.orientation.z = normals->points[ pointIdxNKNSearch[i] ].normal_z;
			//arrow.points.push_back(arrowpoint);			
			//path.points.push_back(pathpoint);
			path_pub.publish( path );
			path_pub.publish( arrow );	
			}                
		}
        	}
		state = 1;
		break;

	case 1:
        	for (x = Xmax ; x >= Xmin; x = x - Xresolution )
        	{
                searchPoint.x = x;
                std::cout << "K nearest neighbor search at (" << searchPoint.x
                << " " << searchPoint.z
                << ") with K=" << K << std::endl;

                if ( kdtree1.nearestKSearchT (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
			{
                        std::cout << "    "  << cloud_converted->points[ pointIdxNKNSearch[i] ].x 
                        << " " << cloud_converted->points[ pointIdxNKNSearch[i] ].y
                        << " " << cloud_converted->points[ pointIdxNKNSearch[i] ].z
                        << " (squared distance: " << pointNKNSquaredDistance[i] 
			<< pointNKNSquaredDistance[i] << ")" << std::endl;

			path.pose.position.x = arrow.pose.position.x = cloud_converted->points[ pointIdxNKNSearch[i] ].x;
			path.pose.position.y = arrow.pose.position.y = cloud_converted->points[ pointIdxNKNSearch[i] ].y;
			path.pose.position.z = arrow.pose.position.z = cloud_converted->points[ pointIdxNKNSearch[i] ].z;
			arrow.pose.orientation.x = normals->points[ pointIdxNKNSearch[i] ].normal_x;
			arrow.pose.orientation.y = normals->points[ pointIdxNKNSearch[i] ].normal_y;
			arrow.pose.orientation.z = normals->points[ pointIdxNKNSearch[i] ].normal_z;
			//arrow.points.push_back(arrowpoint);			
			//path.points.push_back(pathpoint);
			path_pub.publish( path );
			path_pub.publish( arrow );	
			}
                }
        	}
		state = 0;
		break;
		}

  }


  // Publish the data
  pub.publish(cloud_extract);
  ros::param::set("/pcl_extraction/finished", true);

  // wait for shutdown
  ros::waitForShutdown();
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_extraction");
  ros::NodeHandle nh;
  ros::Rate r(30);

  // marker for marking workspace 
  visualization_msgs::Marker marker;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "output_extraction", 1, true);

  // Create visual publisher
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 , true);
 
  while (ros::ok())
  {
 	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "Workspace_Line";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.orientation.w = 1.0;
 	marker.lifetime = ros::Duration();
	marker.scale.x = 0.01;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	geometry_msgs::Point p;
	p.x = -0.5;
	p.y = 0.5;
	p.z = 0.3;
	marker.points.push_back(p);
	vis_pub.publish( marker );
	p.x = -0.5;
	p.y = 0.5;
	p.z = 1;
	marker.points.push_back(p);
 	vis_pub.publish( marker );
	p.x = 0.5;
	p.y = 0.5;
	p.z = 1;
	marker.points.push_back(p);
	p.x = 0.5;
	p.y = 0.5;
	p.z = 0.3;
	marker.points.push_back(p);
	
	r.sleep();


 // ros::spinOnce();
  }
}
