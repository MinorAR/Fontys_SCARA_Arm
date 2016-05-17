#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from dxl.dxlchain import DxlChain

chain = DxlChain("/dev/ttyUSB0", rate=250000)

motors = chain.get_motor_list()

chain.set_reg(5, "torque_enable", 1)

def callback(data):
	command = 512 + round(data.data / 0.005117188)
	chain.goto(5, command, speed=0, blocking=False)

def talker():
	rospy.init_node('fingerjoint_hw_controller')
	
	pub = rospy.Publisher("/fingerjoint_hw_controller/state", Float64, queue_size=10)
	
	rospy.Subscriber("/fingerjoint_hw_controller/command", Float64, callback)
	
	rate = rospy.Rate(50) # 10hz
	
	while not rospy.is_shutdown():
		rawval = (chain.get_reg(5, "present_position") - 512) * 0.005117188
		pub.publish(Float64(rawval))
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
