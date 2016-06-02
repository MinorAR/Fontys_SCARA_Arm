#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <pclcaptureing_positioning/position_request.h>

class ServiceWout
{
public:
	ServiceWout();
	~ServiceWout();
	bool position_request_cb(pclcaptureing_positioning::position_request::Request  &req,
         		pclcaptureing_positioning::position_request::Response &res);
	bool planAndMove();
private:
	ros::NodeHandle node_handle;  
	ros::ServiceServer service;
	ros::Publisher display_publisher;

	moveit::planning_interface::MoveGroup* group;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::DisplayTrajectory display_trajectory;

	geometry_msgs::Pose stored_pose;

	bool trigger;
}; 
 

 

ServiceWout::ServiceWout()
{
	ROS_INFO("Constructor");

	group = new moveit::planning_interface::MoveGroup("FullArm");

	service = node_handle.advertiseService("/request_position", &ServiceWout::position_request_cb, this); 
	display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

//EndEffectorLink

ServiceWout::~ServiceWout()
{
	delete group;
}

bool ServiceWout::planAndMove()
{
	if(!trigger){
		return false;
	}

	trigger = false;

	ROS_INFO("Setting target pose...");

	ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

	group->setPoseTarget(stored_pose);

	ROS_INFO("Preparing to plan...");

	moveit::planning_interface::MoveGroup::Plan my_plan;
	group->plan(my_plan);

	sleep(10.0);

	ROS_INFO("Preparing to move...");
  
	group->move();

	return true;
}

bool ServiceWout::position_request_cb(pclcaptureing_positioning::position_request::Request  &req,
         		pclcaptureing_positioning::position_request::Response &res)

{	
	
	stored_pose.position.x = req.pose.position.x;
	stored_pose.position.y = req.pose.position.y;
	stored_pose.position.z = req.pose.position.z;
	stored_pose.orientation.x = req.pose.orientation.x;
	stored_pose.orientation.y = req.pose.orientation.y;
	stored_pose.orientation.z = req.pose.orientation.z;
	stored_pose.orientation.w = req.pose.orientation.w;

	res.success = true;

	trigger = true;

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pclcaptureing_positioning");

	//sleep(1.0);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Rate rate(10);
	
	ServiceWout myService;
  
	ROS_INFO("Ready_to_move_to_position");
	
	while(1){
		myService.planAndMove();

		rate.sleep();
	}
	//ros::spin();
	return 0;
}
