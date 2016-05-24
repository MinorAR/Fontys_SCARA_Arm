#ifndef SCARA_SETUP_HWA_H
#define SCARA_SETUP_HWA_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <ros/console.h>

namespace scara_setup {
	class ScaraSetupHWA: public hardware_interface::RobotHW
	{
	
	public:
		ScaraSetupHWA();
		~ScaraSetupHWA();
		
		void linearCb(const std_msgs::Float32::ConstPtr& state);
		void shoulderCb(const std_msgs::Float64::ConstPtr& state);
		void elbowCb(const std_msgs::Float64::ConstPtr& state);
		void wristCb(const std_msgs::Float64::ConstPtr& state);
		void fingerjointCb(const std_msgs::Float64::ConstPtr& state);
		
		void resetPositionsCb(const std_msgs::Bool::ConstPtr& msg);
	
		void read();
		void write();
		
		bool getResetSwitch();
		void clearResetSwitch();
		
	private:
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		
		double jnt_cmd[5];
		double jnt_pos[5];
		double jnt_vel[5];
		double jnt_eff[5];
		
		double act_cmd[5];
		
		double trans[5];
		
		bool reset_switch;
		
		ros::NodeHandle n;
		ros::Publisher linear_cmd_pub;
		ros::Subscriber linear_state_sub;
		ros::Publisher shoulder_cmd_pub;
		ros::Subscriber shoulder_state_sub;
		ros::Publisher elbow_cmd_pub;
		ros::Subscriber elbow_state_sub;
		ros::Publisher wrist_cmd_pub;
		ros::Subscriber wrist_state_sub;
		ros::Publisher fingerjoint_cmd_pub;
		ros::Subscriber fingerjoint_state_sub;
		
		ros::Subscriber reset_positions_sub;
	};
};

#endif
