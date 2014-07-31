#!/usr/bin/env python

PACKAGE = 'stewart_platform'
import roslib
roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from math import pi
import serial


class servo_controller:
	def __init__(self,params):
		# some dictionaries to support multiple options within the serial port
		parityDict = {'n':serial.PARITY_NONE,'e':serial.PARITY_EVEN,'o':serial.PARITY_ODD,'m':serial.PARITY_MARK,'s':serial.PARITY_SPACE}
		stopDict = {1:serial.STOPBITS_ONE,15:serial.STOPBITS_ONE_POINT_FIVE,2:serial.STOPBITS_TWO}
		dataDict = {5:serial.FIVEBITS,6:serial.SIXBITS,7:serial.SEVENBITS,8:serial.EIGHTBITS}
		# the actual controller for the Pololu Maestro
		self.controller = serial.Serial(port=params['port'],baudrate=params['baud'],parity=parityDict[params['parity']],stopbits=stopDict[params['stopBits']])
		# Stuff that I was told to pout in.....not sure why
		self.controller.write(chr(0xAA))
		self.controller.flush()
		# initial goal position is zeros
		self.goalPositions = np.zeros(6)
		self.curPositions = np.zeros(6)
		writeNextPos(self.goalPositions)		

	def writeNextPos(positions):
		for i in range(0,len(positions)):
			self.controller.write(get_command(i,positions[i]*10.5 + 1500))

	def get_command(channel, target):
		target = target*4
		serialBytes = chr(0x84)+chr(channel)+chr(target & 0x7F)+chr((target >> 7) & 0x7F)
		return serialBytes

	def writeGoal(msg):
		print msg
		degreeShift = -45
		rev_mask = np.array( [1 -1 -1 -1 1 1] )

		
if __name__ == '__main__':
	print "loading servo controller", rospy.get_param('servo_controller')
	platform = servo_controller( rospy.get_param('servo_controller')  )
	rospy.Subscriber('/stewart/JointState', JointState, sc.writeGoal)
	rospy.spin()
