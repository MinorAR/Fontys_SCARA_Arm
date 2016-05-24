#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from dxl.dxlchain import DxlChain

chain = DxlChain("/dev/ttyUSB0", rate=250000)

motors = chain.get_motor_list()

excitation = False
lower_limit = False
upper_limit = False
calibrated = False

rospy.init_node('full_hw_controller')

pub_reset_enc = rospy.Publisher('/scara_setup/linear_encoder/reset', Bool, queue_size=100)

def callback_cog_linear(data):
	callback_linear_override(Float64(-0.4))
	while lower_limit == False:
		pass
	
	pub_reset_enc.publish(Bool(True))

	rospy.sleep(0.05)
	
	callback_linear_override(Float64(0.4))
	while calibrated == False:
		pass
	
	callback_linear_override(Float64(0.0))

def set_excitation(par):
	global excitation
	
	if par:
		val = 1
		#print "Excitation ON"
		excitation = True
	else :
		val = 0
		excitation = False
		#print "Excitation OFF"
	
	chain.set_reg(1, "torque_enable", val)
	chain.set_reg(2, "torque_enable", val)
	chain.set_reg(3, "torque_enable", val)
	chain.set_reg(4, "torque_enable", val)
	chain.set_reg(5, "torque_enable", val)

def callback_linear(data):
	command = abs(round(data.data * 25600))
	
	#if command > 5:
	#	command = command + 80
	
	if command > 1023: 
		command = 1023
	
	if data.data < 0:
		command = command + 1024
		
	if lower_limit and (data.data < 0.0):
		return
		
	if upper_limit and (data.data > 0.0):
		return

	if excitation:
		chain.set_reg(1, "moving_speed", command)
		
def callback_linear_override(data):
	if data.data == 0.0:
		chain.set_reg(1, "torque_enable", 0)
		return
	
	chain.set_reg(1, "torque_enable", 1)
	command = abs(round(data.data * 25600))
	
	#if command > 5:
	#	command = command + 80
	
	if command > 1023: 
		command = 1023
	
	if data.data < 0:
		command = command + 1024
	
	if lower_limit and (data.data < 0.0):
		return
		
	if upper_limit and (data.data > 0.0):
		return
	
	chain.set_reg(1, "moving_speed", command)
	
def callback_excitation(data):
	set_excitation(data.data)
	
def callback_calibrated(data):
	global calibrated
	calibrated = data.data
	
def callback_lower_limit(data):
	global lower_limit
	lower_limit = data.data
	
	if lower_limit and chain.get_reg(1, "present_speed") > 1024:
		chain.set_reg(1, "moving_speed", 0)
	
def callback_upper_limit(data):
	global upper_limit
	upper_limit = data.data
	
	if upper_limit and chain.get_reg(1, "present_speed") < 1024:
		chain.set_reg(1, "moving_speed", 0)
	
def callback_shoulder(data):
	command = 2048 + round(data.data / 0.001533203)
	#chain.goto(2, command, speed=0, blocking=False)
	if excitation:
		chain.set_reg(2, "goal_pos", command)
	
def callback_elbow(data):
	command = 2048 + round(data.data / 0.001533203)
	#chain.goto(3, command, speed=0, blocking=False)
	if excitation:
		chain.set_reg(3, "goal_pos", command)
	
def callback_wrist(data):
	command = 2048 + round(data.data / 0.001533203)
	#chain.goto(4, command, speed=0, blocking=False)
	if excitation:
		chain.set_reg(4, "goal_pos", command)
	
def callback_fingerjoint(data):
	command = 512 + round(data.data / 0.005117188)
	#chain.goto(5, command, speed=0, blocking=False)
	if excitation:
		chain.set_reg(5, "goal_pos", command)

def talker():	
	rospy.Subscriber("/full_hw_controller/linear/command", Float64, callback_linear)
	rospy.Subscriber("/full_hw_controller/linear/override", Float64, callback_linear_override)
	
	shoulder_pub = rospy.Publisher("/full_hw_controller/shoulder/state", Float64, queue_size=10)
	rospy.Subscriber("/full_hw_controller/shoulder/command", Float64, callback_shoulder)
	
	elbow_pub = rospy.Publisher("/full_hw_controller/elbow/state", Float64, queue_size=10)
	rospy.Subscriber("/full_hw_controller/elbow/command", Float64, callback_elbow)
	
	wrist_pub = rospy.Publisher("/full_hw_controller/wrist/state", Float64, queue_size=10)
	rospy.Subscriber("/full_hw_controller/wrist/command", Float64, callback_wrist)
	
	fingerjoint_pub = rospy.Publisher("/full_hw_controller/fingerjoint/state", Float64, queue_size=10)
	rospy.Subscriber("/full_hw_controller/fingerjoint/command", Float64, callback_fingerjoint)
	
	rospy.Subscriber("/full_hw_controller/set_excitation", Bool, callback_excitation)
	
	rospy.Subscriber("/scara_setup/linear_encoder/lower_limit", Bool, callback_lower_limit)
	
	rospy.Subscriber("/scara_setup/linear_encoder/upper_limit", Bool, callback_upper_limit)
	
	rospy.Subscriber("/scara_setup/linear_encoder/calibrated", Bool, callback_calibrated)
	
	rospy.Subscriber("/full_hw_controller/cog_linear", Bool, callback_cog_linear)
	
	rate = rospy.Rate(20) # 10hz
	
	set_excitation(False)
	
	while not rospy.is_shutdown():
		rawval = (chain.get_reg(2, "present_position") - 2048) * 0.001533203
		shoulder_pub.publish(Float64(rawval))
		
		rawval = (chain.get_reg(3, "present_position") - 2048) * 0.001533203
		elbow_pub.publish(Float64(rawval))
		
		rawval = (chain.get_reg(4, "present_position") - 2048) * 0.001533203
		wrist_pub.publish(Float64(rawval))
		
		rawval = (chain.get_reg(5, "present_position") - 512) * 0.005117188
		fingerjoint_pub.publish(Float64(rawval))
		
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		chain.disable()
		pass