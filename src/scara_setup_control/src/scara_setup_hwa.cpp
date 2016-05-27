#include "scara_setup_control/scara_setup_hwa.h"

scara_setup::ScaraSetupHWA::ScaraSetupHWA()
{
	trans[0] = 1.0; //linear, no transmission, this is handled by the velocity control loop
   trans[1] = 2.0; //shoulder
   trans[2] = -2.0; //elbow
   trans[3] = -2.0; //<- third joint is the wrist
   trans[4] = -1.0; //fingerjoint	
   
   reset_switch = false;
	
	// connect and register the joint state interface
	hardware_interface::JointStateHandle state_handle_linear("linear", &jnt_pos[0], &jnt_vel[0], &jnt_eff[0]);
   jnt_state_interface.registerHandle(state_handle_linear);

   hardware_interface::JointStateHandle state_handle_shoulder("shoulder", &jnt_pos[1], &jnt_vel[1], &jnt_eff[1]);
   jnt_state_interface.registerHandle(state_handle_shoulder);
   
   hardware_interface::JointStateHandle state_handle_elbow("elbow", &jnt_pos[2], &jnt_vel[2], &jnt_eff[2]);
   jnt_state_interface.registerHandle(state_handle_elbow);
   
   hardware_interface::JointStateHandle state_handle_wrist("wrist", &jnt_pos[3], &jnt_vel[3], &jnt_eff[3]);
   jnt_state_interface.registerHandle(state_handle_wrist);
   
   hardware_interface::JointStateHandle state_handle_fingerjoint("fingerjoint", &jnt_pos[4], &jnt_vel[4], &jnt_eff[4]);
   jnt_state_interface.registerHandle(state_handle_fingerjoint);

   registerInterface(&jnt_state_interface);
   
   // connect and register the joint position interface  
   hardware_interface::JointHandle pos_handle_linear(jnt_state_interface.getHandle("linear"), &jnt_cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_linear);
   
   hardware_interface::JointHandle pos_handle_shoulder(jnt_state_interface.getHandle("shoulder"), &jnt_cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_shoulder);
   
   hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("elbow"), &jnt_cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_elbow);
   
   hardware_interface::JointHandle pos_handle_wrist(jnt_state_interface.getHandle("wrist"), &jnt_cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_wrist);
   
   hardware_interface::JointHandle pos_handle_fingerjoint(jnt_state_interface.getHandle("fingerjoint"), &jnt_cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_fingerjoint);

   registerInterface(&jnt_pos_interface);
   
   //set up the publishers & listeners
   linear_cmd_pub = n.advertise<std_msgs::Float64>("/full_hw_controller/linear/command", 1000);
   linear_state_sub = n.subscribe("/scara_setup/linear_encoder/value", 1000, &scara_setup::ScaraSetupHWA::linearCb, this);
   
   shoulder_cmd_pub = n.advertise<std_msgs::Float64>("/full_hw_controller/shoulder/command", 1000);
   shoulder_state_sub = n.subscribe("/full_hw_controller/shoulder/state", 1000, &scara_setup::ScaraSetupHWA::shoulderCb, this);
   
   elbow_cmd_pub = n.advertise<std_msgs::Float64>("/full_hw_controller/elbow/command", 1000);
   elbow_state_sub = n.subscribe("/full_hw_controller/elbow/state", 1000, &scara_setup::ScaraSetupHWA::elbowCb, this);
   
   wrist_cmd_pub = n.advertise<std_msgs::Float64>("/full_hw_controller/wrist/command", 1000);
   wrist_state_sub = n.subscribe("/full_hw_controller/wrist/state", 1000, &scara_setup::ScaraSetupHWA::wristCb, this);
   
   fingerjoint_cmd_pub = n.advertise<std_msgs::Float64>("/full_hw_controller/fingerjoint/command", 1000);
   fingerjoint_state_sub = n.subscribe("/full_hw_controller/fingerjoint/state", 1000, &scara_setup::ScaraSetupHWA::fingerjointCb, this);
   
   reset_positions_sub = n.subscribe("/scara_setup/reset_positions", 1000, &scara_setup::ScaraSetupHWA::resetPositionsCb, this);
}

scara_setup::ScaraSetupHWA::~ScaraSetupHWA()
{
	//
}

bool scara_setup::ScaraSetupHWA::getResetSwitch()
{
	return reset_switch;
}

void scara_setup::ScaraSetupHWA::clearResetSwitch()
{
	reset_switch = false;
}

void scara_setup::ScaraSetupHWA::resetPositionsCb(const std_msgs::Empty::ConstPtr& msg)
{
	reset_switch = true;
}

void scara_setup::ScaraSetupHWA::linearCb(const std_msgs::Float32::ConstPtr& state)
{
	jnt_pos[0] = state->data / trans[0];
}

void scara_setup::ScaraSetupHWA::shoulderCb(const std_msgs::Float64::ConstPtr& state)
{
	jnt_pos[1] = state->data / trans[1];
}

void scara_setup::ScaraSetupHWA::elbowCb(const std_msgs::Float64::ConstPtr& state)
{
	jnt_pos[2] = state->data / trans[2];
}

void scara_setup::ScaraSetupHWA::wristCb(const std_msgs::Float64::ConstPtr& state)
{
	jnt_pos[3] = state->data / trans[3];
}

void scara_setup::ScaraSetupHWA::fingerjointCb(const std_msgs::Float64::ConstPtr& state)
{
	jnt_pos[4] = state->data / trans[4];
}

void scara_setup::ScaraSetupHWA::read()
{
	//
}

void scara_setup::ScaraSetupHWA::write()
{
	//bypassing the feedback loop here
	//jnt_pos[0] = jnt_pos[0] + jnt_cmd[0] * 0.02; //approximate integration of the velocity controlled linear joint :)
	//jnt_pos[1] = jnt_cmd[1];
	//jnt_pos[2] = jnt_cmd[2];
	//jnt_pos[3] = jnt_cmd[3];
	//jnt_pos[4] = jnt_cmd[4];
	
	std_msgs::Float64 msg;
	
	act_cmd[0] = jnt_cmd[0] * trans[0];
	msg.data = act_cmd[0] - jnt_pos[0];
	linear_cmd_pub.publish(msg);
	
	act_cmd[1] = jnt_cmd[1] * trans[1];
	msg.data = act_cmd[1];
	shoulder_cmd_pub.publish(msg);
	
	act_cmd[2] = jnt_cmd[2] * trans[2];
	msg.data = act_cmd[2];
	elbow_cmd_pub.publish(msg);
	
	act_cmd[3] = jnt_cmd[3] * trans[3];
	msg.data = act_cmd[3];
	wrist_cmd_pub.publish(msg);
	
	act_cmd[4] = jnt_cmd[4] * trans[4];
	msg.data = act_cmd[4];
	fingerjoint_cmd_pub.publish(msg);
}

int main(int argc, char** argv)
{
	ROS_INFO("Starting HWA layer...");
	ros::init(argc, argv, "scara_setup_hwa_layer");
	
	scara_setup::ScaraSetupHWA robot;
	controller_manager::ControllerManager cm(&robot);
	
	ros::AsyncSpinner spinner(4);
   spinner.start();

	ros::Rate loop_rate(20);

	ros::Time lt = ros::Time::now();

	while (ros::ok())
	{
		ros::Time ct = ros::Time::now();
		ros::Duration et = ct - lt;
		lt = ct;
		
		//robot.read(); <- hw pos constantly updated by the callbacks
		cm.update(ct, et, robot.getResetSwitch());
		robot.clearResetSwitch();
		robot.write();
		
		loop_rate.sleep();
	}

	return 0;
}
