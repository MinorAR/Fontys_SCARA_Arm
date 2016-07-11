/*
This node is meant to offer a service which moves the arm to a requested position and send back a acknowlage that that the requested position is reached.
*/

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
	ServiceWout(); // constructor
	~ServiceWout(); // deconstructor
	bool position_request_cb(pclcaptureing_positioning::position_request::Request  &req,
         		pclcaptureing_positioning::position_request::Response &res); //callback function position_request
	bool planAndMove(); // plan and move function
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
 

 

ServiceWout::ServiceWout() // constructor
{
	ROS_INFO("Constructor"); // prints information for debugging/or general use

	group = new moveit::planning_interface::MoveGroup("FullArm"); // setup the movegroup

	service = node_handle.advertiseService("/request_position", &ServiceWout::position_request_cb, this);  // offer the node
	display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);// visualize the planned path
}

ServiceWout::~ServiceWout() // deconstructor
{
	delete group;
}

bool ServiceWout::planAndMove() // Defenition planAndMove function
{
	if(!trigger){ // activates the planAndMove function
		return false;
	}

	trigger = false;

	ROS_INFO("Setting target pose..."); 

	ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

	group->setPoseTarget(stored_pose); // set the target position to stored_pose

	ROS_INFO("Preparing to plan...");

	moveit::planning_interface::MoveGroup::Plan my_plan; // create a new plan
	group->plan(my_plan); // plan the new position in MoveIt

	sleep(10.0); // wait 10 sec

	ROS_INFO("Preparing to move...");
  
	group->move(); // This command is responible for activating the planned path and moving the arm.

	return true;
}

bool ServiceWout::position_request_cb(pclcaptureing_positioning::position_request::Request  &req,
         		pclcaptureing_positioning::position_request::Response &res)

{	
	stored_pose.orientation.w = req.pose.orientation.w; // store the requested positions in stored_pose.position
	stored_pose.position.x = req.pose.position.x;
	stored_pose.position.y = req.pose.position.y;
	stored_pose.position.z = req.pose.position.z;

	res.success = true;

	trigger = true;

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pclcaptureing_positioning"); //initialize the node

	//sleep(1.0);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Rate rate(10); // looprate (determines how fast the loop loops)
	
	ServiceWout myService; // instantiate object
  
	ROS_INFO("Ready_to_move_to_position");
	
	while(1){  // as long as the PlanAndMove is busy, then wait.
		myService.planAndMove(); // execute planAndMove

		rate.sleep();
	}
	//ros::spin();
	return 0;
}
