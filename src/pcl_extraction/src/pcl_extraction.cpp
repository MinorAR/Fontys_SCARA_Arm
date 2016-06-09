#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
// normal estimation
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
//transform
#include <tf/transform_datatypes.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>

//moveit 
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


// workspace dimensions
const double Xmin = -0.3;
const double Xmax = 0.3;
const double Ymin = 0.5;
const double Ymax = 0.5;
const double Zmin = 0.43;
const double Zmax = 0.8;
const double Zresolution = 0.03;
const double Xresolution = 0.02;
const double WorkspaceDistance = 1;

ros::Publisher pub;
ros::Publisher vis_pub;
ros::Publisher path_pub;
ros::Publisher display_publisher;

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

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_converted, *cloud_converted, indices);

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

	// marker for marking workspace 
  	visualization_msgs::Marker marker;

      	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "Workspace_Line";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	//delete all markers then add 
	marker.action = visualization_msgs::Marker::DELETE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.orientation.w = 1.0;
 	marker.lifetime = ros::Duration();
	marker.scale.x = 0.01;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	geometry_msgs::Point p;
	p.x = Xmin;
	p.y = WorkspaceDistance;
	p.z = Zmin;
	marker.points.push_back(p);
	vis_pub.publish( marker );
	p.x = Xmin;
	p.y = WorkspaceDistance;
	p.z = Zmax;
	marker.points.push_back(p);
 	vis_pub.publish( marker );
	p.x = Xmax;
	p.y = WorkspaceDistance;
	p.z = Zmax;
	marker.points.push_back(p);
	vis_pub.publish( marker );
	p.x = Xmax;
	p.y = WorkspaceDistance;
	p.z = Zmin;
	marker.points.push_back(p);
	vis_pub.publish( marker );
	p.x = Xmin;
	p.y = WorkspaceDistance;
	p.z = Zmin;
	marker.points.push_back(p);
	vis_pub.publish( marker );

  // Normal estimation.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud_converted);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);

  // Make a search cloud for kdtree
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromPCLPointCloud2(*input, *cloud_search);
  // set y to zero to search only for x and z
  for (size_t i = 0; i < cloud_search->points.size (); ++i)
  {
    cloud_search->points[i].y = 0;
  }

  // arrow marker
  const int cloudsize = (((Xmax - Xmin)/Xresolution) * ((Zmax - Zmin)/Zresolution)); 
  std::cerr << "Cloudpoints: " << cloudsize << std::endl;
  geometry_msgs::Pose PathArrow[cloudsize];
  int size = 0;

 	// make marker
  	visualization_msgs::Marker path, arrow;
	path.header.frame_id = arrow.header.frame_id = "world";
	path.header.stamp = arrow.header.stamp = ros::Time();
	path.ns = "Path";
	arrow.ns = "arrow";
	path.id = 1;
	//arrow.id = 2;
	path.type = visualization_msgs::Marker::LINE_STRIP;
	arrow.type = visualization_msgs::Marker::ARROW;
	path.action = arrow.action = visualization_msgs::Marker::DELETE;
	path.action = arrow.action = visualization_msgs::Marker::ADD;
	
	// line orientation
	path.pose.orientation.w = 1.0;
	
	path.color.a = 0.9; // Don't forget to set the alpha!
	path.color.r = 0.0;
	path.color.g = 1.0;
	path.color.b = 0.0;

 	path.lifetime = ros::Duration();
	path.scale.x = 0.001;
	path.color.a = 0.9; // Don't forget to set the alpha!
	path.color.r = 0.0;
	path.color.g = 0.0;
	path.color.b = 1.0;


	arrow.color.a = 0.9; // Don't forget to set the alpha!
	arrow.color.r = 0.0;
	arrow.color.g = 1.0;
	arrow.color.b = 0.0;

 	arrow.lifetime = ros::Duration();
	arrow.scale.x = 0.01;
	arrow.scale.y = 0.005;
	arrow.scale.z = 0.005;
	arrow.color.a = 1.0; // Don't forget to set the alpha!
	arrow.color.r = 0.0;
	arrow.color.g = 1.0;
	arrow.color.b = 0.0;

  // K nearest neighbor search
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1;
  kdtree1.setInputCloud (cloud_search);
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
			
  // search kdtree for closest point to path
  pcl::PointXYZ searchPoint;
  volatile double x;
  volatile int state = 0;

//transform normal
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Quaternion msg;
  tf::Vector3 dir_vector(-1.0, 0.0, 0.0);

  //moveit
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::planning_interface::MoveGroup group("FullArm");
  group.setPlannerId("RRTkConfigDefault");
  group.setStartStateToCurrentState();
  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory;
  geometry_msgs::Pose target_pose;


//group.setworkspace todo
	

  for (volatile double z = Zmin; z <= Zmax; z = z + Zresolution )
  {
 	searchPoint.z = z;
	switch(state)
	{
	case 0:
		for (x = Xmin ; x <= Xmax; x = x + Xresolution )
        	{
                searchPoint.x = x;
 
                if ( kdtree1.nearestKSearchT (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                        for (volatile int i = 0; i < pointIdxNKNSearch.size (); ++i)
			{
                        geometry_msgs::Point PathPoints0;
 			geometry_msgs::Point PathPoints0_1;
			PathPoints0.x = PathArrow[size].position.x = x;//cloud_converted->points[ pointIdxNKNSearch[i] ].x;
			PathPoints0.y = PathArrow[size].position.y = cloud_converted->points[ pointIdxNKNSearch[i] ].y;
			PathPoints0.z = PathArrow[size].position.z = z;//cloud_converted->points[ pointIdxNKNSearch[i] ].z;
			path.points.push_back(PathPoints0);
					
			path_pub.publish( path );	
			
			//transform normal
			tf::Vector3 axis_vector(normals->points[ pointIdxNKNSearch[i] ].normal_x,
				normals->points[ pointIdxNKNSearch[i] ].normal_y, 
				normals->points[ pointIdxNKNSearch[i] ].normal_z);
    			
    			tf::Vector3 vector = axis_vector.cross(dir_vector);
   			vector.normalized();
    			tf::Quaternion q(vector, -1.0*acos(axis_vector.dot(dir_vector)));
    			q.normalize();
    			tf::quaternionTFToMsg(q, msg);

			PathArrow[size].orientation.x = msg.x;
			PathArrow[size].orientation.y = msg.y;
			PathArrow[size].orientation.z = msg.z;
			PathArrow[size].orientation.w = msg.w;

			size++;
			}                
		}
        	}
		state = 1;
		break;

	case 1:
        	for (x = Xmax ; x >= Xmin; x = x - Xresolution )
        	{
                searchPoint.x = x;
  
                if ( kdtree1.nearestKSearchT (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                        for (volatile int i = 0; i < pointIdxNKNSearch.size (); ++i)
			{
			// publish path line points
			geometry_msgs::Point PathPoints1;
			geometry_msgs::Point PathPoints1_1;
			PathPoints1.x = PathArrow[size].position.x = x; //cloud_converted->points[ pointIdxNKNSearch[i] ].x;
			PathPoints1.y = PathArrow[size].position.y = cloud_converted->points[ pointIdxNKNSearch[i] ].y;
			PathPoints1.z = PathArrow[size].position.z = z; //cloud_converted->points[ pointIdxNKNSearch[i] ].z;
			path.points.push_back(PathPoints1);
				
			path_pub.publish( path );
			PathArrow[size].orientation.w = 1.0;
			
			// transform normal
			tf::Vector3 axis_vector(normals->points[ pointIdxNKNSearch[i] ].normal_x,
				normals->points[ pointIdxNKNSearch[i] ].normal_y, 
				normals->points[ pointIdxNKNSearch[i] ].normal_z);
    			
    			tf::Vector3 vector = axis_vector.cross(dir_vector);
   			vector.normalized();
    			tf::Quaternion q(vector, -1.0*acos(axis_vector.dot(dir_vector)));
    			q.normalize();
    			tf::quaternionTFToMsg(q, msg);

			PathArrow[size].orientation.x = msg.x;
			PathArrow[size].orientation.y = msg.y;
			PathArrow[size].orientation.z = msg.z;
			PathArrow[size].orientation.w = msg.w;	


			size++;
			}
                }
        	}
		state = 0;
		break;
		}

  }



//publish arrows 
  for (volatile int i = 0; i <= cloudsize; i++)
  {
	arrow.id = i + 2; //add 2 because id 1 is already used.
	arrow.pose.position.x = target_pose.position.x = PathArrow[i].position.x;
	//arrow.pose.position.y = target_pose.position.y = PathArrow[i].position.y;
	arrow.pose.position.y = PathArrow[i].position.y;
		target_pose.position.y = WorkspaceDistance;

	arrow.pose.position.z = target_pose.position.z = PathArrow[i].position.z;
		

	// Orientation
/*
	arrow.pose.orientation.x = target_pose.orientation.x = PathArrow[i].orientation.x;
	arrow.pose.orientation.y = target_pose.orientation.y = PathArrow[i].orientation.y;
	arrow.pose.orientation.z = target_pose.orientation.z = PathArrow[i].orientation.z;
	arrow.pose.orientation.w = target_pose.orientation.w = PathArrow[i].orientation.w;
*/
// /*
	arrow.pose.orientation.x = PathArrow[i].orientation.x;
	arrow.pose.orientation.y = PathArrow[i].orientation.y;
	arrow.pose.orientation.z = PathArrow[i].orientation.z;
	arrow.pose.orientation.w = PathArrow[i].orientation.w;
	
	target_pose.orientation.w = 1;
	target_pose.orientation.x = 0;
	target_pose.orientation.y = 0;
	target_pose.orientation.z = 0;
// */
	std::cerr << "arrow: " << arrow << std::endl; 
	std::cerr << "target_pose: " << target_pose << std::endl; 	
	path_pub.publish( arrow );
	waypoints.push_back(target_pose);
  }


  double fraction = group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory, //trajectory
					     true); // collision avoiding true -> test

  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(15.0);

  //move the arm
  
  group.move();

  std::cerr << "pcl_extraction: finished" << std::endl;  

  // Publish the data
  //pub.publish(cloud_extract);
  ros::param::set("/pcl_extraction/finished", true);

  // wait for shutdown
  ros::waitForShutdown();
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_extraction");
  ros::NodeHandle nh;
  //ros::Rate r(30);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "output_extraction", 1, true);

  // Create visual publisher
  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 , true);
  // Create visual path publisher
  path_pub = nh.advertise<visualization_msgs::Marker>( "visualization_path_marker", 0 , true);
  // path publisher
  display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
 
 while (ros::ok())
        {
                ros::spin();
        }
}
