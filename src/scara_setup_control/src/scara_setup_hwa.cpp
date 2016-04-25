#include "scara_setup_control/scara_setup_hwa.h"

scara_setup::ScaraSetupHWA::ScaraSetupHWA()
{
	// connect and register the joint state interface
	hardware_interface::JointStateHandle state_handle_linear("linear", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_linear);

   hardware_interface::JointStateHandle state_handle_shoulder("shoulder", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_shoulder);
   
   hardware_interface::JointStateHandle state_handle_elbow("elbow", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_elbow);
   
   hardware_interface::JointStateHandle state_handle_wrist("wrist", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_wrist);
   
   hardware_interface::JointStateHandle state_handle_fingerjoint("fingerjoint", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_fingerjoint);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_linear(jnt_state_interface.getHandle("linear"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_linear);

   hardware_interface::JointHandle pos_handle_shoulder(jnt_state_interface.getHandle("shoulder"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_shoulder);
   
   hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("elbow"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_elbow);
   
   hardware_interface::JointHandle pos_handle_wrist(jnt_state_interface.getHandle("wrist"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_wrist);
   
   hardware_interface::JointHandle pos_handle_fingerjoint(jnt_state_interface.getHandle("fingerjoint"), &cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_fingerjoint);

   registerInterface(&jnt_pos_interface);
}

scara_setup::ScaraSetupHWA::~ScaraSetupHWA()
{
	//
}

void scara_setup::ScaraSetupHWA::read()
{
	//
}

void scara_setup::ScaraSetupHWA::write()
{
	pos[0] = cmd[0];
	pos[1] = cmd[1];
	pos[2] = cmd[2];
	pos[3] = cmd[3];
	pos[4] = cmd[4];
}

int main(int argc, char** argv)
{
	ROS_INFO("Starting HWA layer...");
	ros::init(argc, argv, "scara_setup_hwa_layer");
	
	scara_setup::ScaraSetupHWA robot;
	controller_manager::ControllerManager cm(&robot);
	
	ros::AsyncSpinner spinner(4);
   spinner.start();

	ros::Rate loop_rate(50);

	ros::Time lt = ros::Time::now();

	while (ros::ok())
	{
		ros::Time ct = ros::Time::now();
		ros::Duration et = ct - lt;
		lt = ct;
		
		robot.read();
		cm.update(ct, et);
		robot.write();
		
		loop_rate.sleep();
	}

	return 0;
}
