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

namespace scara_setup {
	class ScaraSetupHWA: public hardware_interface::RobotHW
	{
	
	public:
		ScaraSetupHWA();
		~ScaraSetupHWA();
		
		void linearCb(const std_msgs::Float32::ConstPtr& state);
		//void linearVelCb(const std_msgs::Float64::ConstPtr& state);
		void shoulderCb(const std_msgs::Float64::ConstPtr& state);
		void elbowCb(const std_msgs::Float64::ConstPtr& state);
		void wristCb(const std_msgs::Float64::ConstPtr& state);
		void fingerjointCb(const std_msgs::Float64::ConstPtr& state);
	
		void read();
		void write();
		
	private:
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		//hardware_interface::VelocityJointInterface jnt_vel_interface;
		
		transmission_interface::JointToActuatorPositionInterface jnt_to_act;
		transmission_interface::ActuatorToJointPositionInterface act_to_jnt;
		
		transmission_interface::ActuatorData wrist_actuator_data;
		transmission_interface::JointData wrist_joint_data;
		
		double jnt_cmd[5];
		double jnt_pos[5];
		double jnt_vel[5];
		double jnt_eff[5];
		
		double act_cmd[5];
		double act_pos[5];
		double act_vel[5];
		double act_eff[5];
		
		double trans[5];
		
		ros::NodeHandle n;
		ros::Publisher linear_cmd_pub;
		ros::Subscriber linear_state_sub;
		//ros::Subscriber linear_vel_sub;
		ros::Publisher shoulder_cmd_pub;
		ros::Subscriber shoulder_state_sub;
		ros::Publisher elbow_cmd_pub;
		ros::Subscriber elbow_state_sub;
		ros::Publisher wrist_cmd_pub;
		ros::Subscriber wrist_state_sub;
		ros::Publisher fingerjoint_cmd_pub;
		ros::Subscriber fingerjoint_state_sub;
	};
};

#endif
