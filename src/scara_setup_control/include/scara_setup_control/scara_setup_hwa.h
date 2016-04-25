#ifndef SCARA_SETUP_HWA_H
#define SCARA_SETUP_HWA_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

namespace scara_setup {
	class ScaraSetupHWA: public hardware_interface::RobotHW
	{
	
	public:
		ScaraSetupHWA();
		~ScaraSetupHWA();
	
		void read();
		void write();
		
	private:
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		
		double cmd[5];
		double pos[5];
		double vel[5];
		double eff[5];
	};
};

#endif
