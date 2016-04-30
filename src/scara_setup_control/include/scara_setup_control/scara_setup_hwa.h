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

namespace scara_setup {
	class ScaraSetupHWA: public hardware_interface::RobotHW
	{
	
	public:
		ScaraSetupHWA();
		~ScaraSetupHWA();
		
		void wristCb(const dynamixel_msgs::JointState::ConstPtr& state);
	
		void read();
		void write();
		
	private:
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		
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
		ros::Publisher wrist_cmd_pub;
		ros::Subscriber wrist_state_sub;
	};
};

#endif
