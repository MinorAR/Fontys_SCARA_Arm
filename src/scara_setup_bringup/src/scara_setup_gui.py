#!/usr/bin/env python
import rospy
from Tkinter import *
from std_msgs.msg import Bool
from std_msgs.msg import Float64

class RosPublisher:
	def __init__(self):
		#some 'globals'
		
		#Set up ROS stuff
		rospy.init_node('scara_setup_gui', anonymous=False)
		self.pub_excitation = rospy.Publisher('/full_hw_controller/set_excitation', Bool, queue_size=100)
		self.pub_reset_pos = rospy.Publisher('/scara_setup/reset_positions', Bool, queue_size=100)
		self.pub_z_override = rospy.Publisher('/full_hw_controller/linear/override', Float64, queue_size=100)
		self.pub_reset_enc = rospy.Publisher('/scara_setup/linear_encoder/reset', Bool, queue_size=100)
		
		#Set up visual stuff
		self.root = Tk()
		self.root.title("SCARA setup GUI")
		
		self.stateGroup = LabelFrame(self.root, text="Robot state", padx=10, pady=10)
		self.but_planned = Button(self.stateGroup, text='Planned', command = self.setPlanned)
		self.but_manual = Button(self.stateGroup, text='Manual', command = self.setManual)
		
		self.zGroup = LabelFrame(self.root, text="Manual Z control", padx=10, pady=10)
		self.but_up = Button(self.zGroup, text="Up")
		self.but_down = Button(self.zGroup, text="Down")
		
		self.but_up.bind('<ButtonPress-1>', self.zUpStart)
		self.but_up.bind('<ButtonRelease-1>', self.zUpStop)
		
		self.but_down.bind('<ButtonPress-1>', self.zDownStart)
		self.but_down.bind('<ButtonRelease-1>', self.zDownStop)
		
		self.resetGroup = LabelFrame(self.root, text="Resets", padx=10, pady=10)
		self.but_reset_enc = Button(self.resetGroup, text="Reset encoder", command = self.resetEncoder)
		
		self.stateGroup.pack(pady = 10, padx = 10, side = TOP, anchor = NW)
		self.but_planned.pack(pady = 10, padx = 10, side = LEFT)
		self.but_manual.pack(pady = 10, padx = 10, side = LEFT)
		
		self.zGroup.pack(pady = 10, padx = 10, side = TOP, anchor = NW)
		self.but_up.pack(pady = 10, padx = 10, side = LEFT)
		self.but_down.pack(pady = 10, padx = 10, side = LEFT)
		
		self.resetGroup.pack(pady = 10, padx = 10, side = TOP, anchor = NW)
		self.but_reset_enc.pack(pady = 10, padx = 10, side = LEFT)
		
		self.but_manual['state'] = 'disabled'
		
		self.root.after(100, self.periodic)
		self.root.mainloop()	
	
	#Button callbacks		
	def setPlanned(self):
		self.pub_reset_pos.publish(Bool(True))
		self.pub_excitation.publish(Bool(True))
		
		self.but_planned['state'] = 'disabled'
		self.but_up['state'] = 'disabled'
		self.but_down['state'] = 'disabled'
		self.but_reset_enc['state'] = 'disabled'
		self.but_manual['state'] = 'normal'
	
	def setManual(self):
		self.pub_excitation.publish(Bool(False))
		
		self.but_planned['state'] = 'normal'
		self.but_up['state'] = 'normal'
		self.but_down['state'] = 'normal'
		self.but_reset_enc['state'] = 'normal'
		self.but_manual['state'] = 'disabled'
		
	def zUpStart(self, event):
		if self.but_up['state'] == 'disabled':
			return
		
		self.pub_z_override.publish(Float64(0.4))
	
	def zUpStop(self, event):
		if self.but_up['state'] == 'disabled':
			return
		
		self.pub_z_override.publish(Float64(0.0))
		
	def zDownStart(self, event):
		if self.but_down['state'] == 'disabled':
			return
			
		self.pub_z_override.publish(Float64(-0.4))
		
	def zDownStop(self, event):
		if self.but_down['state'] == 'disabled':
			return
			
		self.pub_z_override.publish(Float64(0.0))
		
	def resetEncoder(self):
		if self.but_reset_enc['state'] == 'disabled':
				return
				
		self.pub_reset_enc.publish(Bool(True))
		
	def periodic(self):
		#do periodic stuff
		self.root.after(100, self.periodic)
		
pub = RosPublisher()


