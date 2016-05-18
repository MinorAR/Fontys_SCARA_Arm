#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from dxl.dxlchain import DxlChain

chain = DxlChain("/dev/ttyUSB0", rate=250000)

motors = chain.get_motor_list()

chain.set_reg(1, "torque_enable", 1)

def callback(data):
	command = abs(round(data.data * 10))
	
	if command > 5:
		command = command + 80
	
	if command > 1023: 
		command = 1023
	
	if data.data > 0:
		command = command + 1024
	
	#chain.goto(5, command, speed=0, blocking=False)
	chain.set_reg(1, "moving_speed", command)
	print command

def talker():
	rospy.init_node('linear_hw_controller')
	
	pub = rospy.Publisher("/linear_hw_controller/state", Float64, queue_size=10)
	
	rospy.Subscriber("/linear_hw_controller/command", Float64, callback)
	
	rate = rospy.Rate(50) # 10hz
	
	while not rospy.is_shutdown():
		rawval = chain.get_reg(1, "present_speed")
		
		if rawval > 1023:
			val = (rawval - 1023) / 10
		else:
			val = -rawval / 100
			
		pub.publish(Float64(rawval))
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		chain.disable()
		pass
