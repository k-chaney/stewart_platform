#!/usr/bin/env python
PKG = 'stewart_platform'
import roslib; roslib.load_manifest(PKG)
import time
from math import pi
from threading import Thread
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class xbox_controller():
	def __init__(self):
		self.sleep_time = 1 / rospy.get_param('joy_hz')
		self.controllerMapping = rospy.get_param('x360')
		self.controllerMask = rospy.get_param('x360_dir_mask')
		print self.controllerMapping
		self.step_size = rospy.get_param('step_size')
		self.joy_data = None


		self.stewart_twist_pub = rospy.Publisher('/X360/Twist', Twist)
		self.stewart_pose = Twist()
		self.stewart_pose.linear.x = 0.0
		self.stewart_pose.linear.y = 0.0
		self.stewart_pose.linear.z = 0.25
		self.stewart_pose.angular.x = 0.0 # roll
		self.stewart_pose.angular.y = 0.0 # pitch
		self.stewart_pose.angular.z = 0.0 # yaw
		rospy.init_node('stewart_joy', anonymous=True)
		rospy.Subscriber('/X360/Joy', Joy, self.controllerCallback)

		rospy.Subscriber('/X360/Twist_In', Twist, self.IBVS_Twist)

		self.Twist_In = Twist()

	def controllerCallback(self, data):
		self.joy_data = data

	def IBVS_Twist(self,data):
		self.Twist_In = data

	def updateStewartPosition(self):
		while not rospy.is_shutdown():
			if self.joy_data:
				if (self.getJoyData('axes','rt') < -0.9):
					self.stewart_twist_pub.publish(self.Twist_In)
				else:
					self.stewart_pose.linear.x += self.getJoyData('axes','x') * self.step_size
					self.stewart_pose.linear.y += self.getJoyData('axes','y') * self.step_size
					self.stewart_pose.linear.z += self.getJoyData('axes','z') * self.step_size
					self.stewart_pose.angular.z += self.getJoyData('axes','yaw') * self.step_size
					self.stewart_twist_pub.publish(self.stewart_pose)
				if (self.getJoyData('axes','lt') < -0.9):
					self.stewart_pose.linear.x = 0.0
					self.stewart_pose.linear.y = 0.0
					self.stewart_pose.linear.z = 0.25
					self.stewart_pose.angular.x = 0.0 # roll
					self.stewart_pose.angular.y = 0.0 # pitch
					self.stewart_pose.angular.z = 0.0 # yaw
					self.stewart_twist_pub.publish(self.stewart_pose)


			time.sleep(self.sleep_time)
	def getJoyData(self,type,name):
		if type=='axis' or type=='axes':
			return self.joy_data.axes[self.controllerMapping[type][name]]*self.controllerMask[type][name]
		elif type=='button':
			return self.joy_data.button[self.controllerMapping[type][name]]*self.controllerMask[type][name]

if __name__ == '__main__':
	try:
		move_stewart = xbox_controller()
		t = Thread(target=move_stewart.updateStewartPosition)
		t.start()
		rospy.spin()
		move_stewart.alive = False
		t.join()
	except rospy.ROSInterruptException: pass