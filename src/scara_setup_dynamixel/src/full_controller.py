#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from dxl.dxlchain import DxlChain

class HWCoupling:
	def __init__(self):
		rospy.init_node('full_hw_controller')
	
		self.excitation = False
		self.lower_limit = False
		self.upper_limit = False
		self.calibrated = False
		
		self.chain = DxlChain(rospy.get_param('~port'), rate=rospy.get_param('~rate'))
		
		self.motors = self.chain.get_motor_list()
		
		self.pub_reset_enc = rospy.Publisher('/scara_setup/linear_encoder/reset', Bool, queue_size=100)
		
		rospy.Subscriber("/full_hw_controller/linear/command", Float64, self.callback_linear)
		rospy.Subscriber("/full_hw_controller/linear/override", Float64, self.callback_linear_override)
	
		self.shoulder_pub = rospy.Publisher("/full_hw_controller/shoulder/state", Float64, queue_size=10)
		rospy.Subscriber("/full_hw_controller/shoulder/command", Float64, self.callback_shoulder)
	
		self.elbow_pub = rospy.Publisher("/full_hw_controller/elbow/state", Float64, queue_size=10)
		rospy.Subscriber("/full_hw_controller/elbow/command", Float64, self.callback_elbow)
	
		self.wrist_pub = rospy.Publisher("/full_hw_controller/wrist/state", Float64, queue_size=10)
		rospy.Subscriber("/full_hw_controller/wrist/command", Float64, self.callback_wrist)
	
		self.fingerjoint_pub = rospy.Publisher("/full_hw_controller/fingerjoint/state", Float64, queue_size=10)
		rospy.Subscriber("/full_hw_controller/fingerjoint/command", Float64, self.callback_fingerjoint)
	
		rospy.Subscriber("/full_hw_controller/set_excitation", Bool, self.callback_excitation)
	
		rospy.Subscriber("/scara_setup/linear_encoder/lower_limit", Bool, self.callback_lower_limit)
	
		rospy.Subscriber("/scara_setup/linear_encoder/upper_limit", Bool, self.callback_upper_limit)
	
		rospy.Subscriber("/scara_setup/linear_encoder/calibrated", Bool, self.callback_calibrated)
	
		rospy.Subscriber("/full_hw_controller/cog_linear", Bool, self.callback_cog_linear)
	
		self.rate = rospy.Rate(20) # 10hz
	
		self.set_excitation(False)
		
	def callback_cog_linear(self, data):
		self.callback_linear_override(Float64(-0.4))
		while self.lower_limit == False:
			pass
	
		self.pub_reset_enc.publish(Bool(True))

		rospy.sleep(0.05)
	
		self.callback_linear_override(Float64(0.4))
		while self.calibrated == False:
			pass
	
		self.callback_linear_override(Float64(0.0))
		
	def set_excitation(self, par):
		if par:
			val = 1
			#print "Excitation ON"
			self.excitation = True
		else :
			val = 0
			self.excitation = False
			#print "Excitation OFF"
	
		self.chain.set_reg(1, "torque_enable", val)
		self.chain.set_reg(2, "torque_enable", val)
		self.chain.set_reg(3, "torque_enable", val)
		self.chain.set_reg(4, "torque_enable", val)
		self.chain.set_reg(5, "torque_enable", val)
		
	def callback_linear(self, data):
		command = abs(round(data.data * 25600))
	
		#if command > 5:
		#	command = command + 80
	
		if command > 1023: 
			command = 1023
	
		if data.data < 0:
			command = command + 1024
		
		if self.lower_limit and (data.data < 0.0):
			return
		
		if self.upper_limit and (data.data > 0.0):
			return

		if self.excitation:
			self.chain.set_reg(1, "moving_speed", command)
			
	def callback_linear_override(self, data):
		if data.data == 0.0:
			self.chain.set_reg(1, "torque_enable", 0)
			return
	
		self.chain.set_reg(1, "torque_enable", 1)
		command = abs(round(data.data * 25600))
	
		#if command > 5:
		#	command = command + 80
	
		if command > 1023: 
			command = 1023
	
		if data.data < 0:
			command = command + 1024
	
		if self.lower_limit and (data.data < 0.0):
			return
		
		if self.upper_limit and (data.data > 0.0):
			return
	
		self.chain.set_reg(1, "moving_speed", command)
		
	def callback_excitation(self, data):
		self.set_excitation(data.data)
	
	def callback_calibrated(self, data):
		self.calibrated = data.data
	
	def callback_lower_limit(self, data):
		self.lower_limit = data.data
	
		if self.lower_limit and self.chain.get_reg(1, "present_speed") > 1024:
			self.chain.set_reg(1, "moving_speed", 0)
	
	def callback_upper_limit(self, data):
		self.upper_limit = data.data
	
		if self.upper_limit and self.chain.get_reg(1, "present_speed") < 1024:
			self.chain.set_reg(1, "moving_speed", 0)
	
	def callback_shoulder(self, data):
		command = 2048 + round(data.data / 0.001533203)
		#chain.goto(2, command, speed=0, blocking=False)
		if self.excitation:
			self.chain.set_reg(2, "goal_pos", command)
	
	def callback_elbow(self, data):
		command = 2048 + round(data.data / 0.001533203)
		#chain.goto(3, command, speed=0, blocking=False)
		if self.excitation:
			self.chain.set_reg(3, "goal_pos", command)
	
	def callback_wrist(self, data):
		command = 2048 + round(data.data / 0.001533203)
		#chain.goto(4, command, speed=0, blocking=False)
		if self.excitation:
			self.chain.set_reg(4, "goal_pos", command)
	
	def callback_fingerjoint(self, data):
		command = 512 + round(data.data / 0.005117188)
		#chain.goto(5, command, speed=0, blocking=False)
		if self.excitation:
			self.chain.set_reg(5, "goal_pos", command)
			
	def loop(self):
		while not rospy.is_shutdown():
			rawval = (self.chain.get_reg(2, "present_position") - 2048) * 0.001533203
			self.shoulder_pub.publish(Float64(rawval))
		
			rawval = (self.chain.get_reg(3, "present_position") - 2048) * 0.001533203
			self.elbow_pub.publish(Float64(rawval))
		
			rawval = (self.chain.get_reg(4, "present_position") - 2048) * 0.001533203
			self.wrist_pub.publish(Float64(rawval))
		
			rawval = (self.chain.get_reg(5, "present_position") - 512) * 0.005117188
			self.fingerjoint_pub.publish(Float64(rawval))
		
			self.rate.sleep()
		
		self.set_excitation(False)
		self.chain.disable()
			
if __name__ == '__main__':
	try:
		node = HWCoupling()
		node.loop()
	except rospy.ROSInterruptException:
		pass
