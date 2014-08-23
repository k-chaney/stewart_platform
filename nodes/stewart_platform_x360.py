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


		self.gantry_publisher = rospy.Publisher('/X360/Twist', Twist)
		self.gantry_pose = Twist()
		self.gantry_pose.linear.x = 0.0
		self.gantry_pose.linear.y = 0.0
		self.gantry_pose.linear.z = 0.25
		self.gantry_pose.angular.x = 0.0 # roll
		self.gantry_pose.angular.y = 0.0 # pitch
		self.gantry_pose.angular.z = 0.0 # yaw
		rospy.init_node('gantry_joy', anonymous=True)


		rospy.Subscriber('/X360/Joy', Joy, self.controllerCallback)
	def controllerCallback(self, data):
		self.joy_data = data
	def updateStewartPosition(self):
		while not rospy.is_shutdown():
			if self.joy_data:

				self.gantry_pose.linear.x += self.getJoyData('axes','x') * self.step_size
				self.gantry_pose.linear.y += self.getJoyData('axes','y') * self.step_size
				self.gantry_pose.linear.z += self.getJoyData('axes','z') * self.step_size
				self.gantry_pose.angular.z += self.getJoyData('axes','yaw') * self.step_size
				self.gantry_pose.angular.y += self.getJoyData('axes','v_dpad')*self.step_size
				self.gantry_pose.angular.x += self.getJoyData('axes','h_dpad')*self.step_size

				print "XYZ: ",self.gantry_pose.linear.x, self.gantry_pose.linear.y, self.gantry_pose.linear.z, 
				print "   RPY: ",self.gantry_pose.angular.x, self.gantry_pose.angular.y, self.gantry_pose.angular.z
				self.gantry_publisher.publish(self.gantry_pose)
			time.sleep(self.sleep_time)
	def getJoyData(self,type,name):
		if type=='axis' or type=='axes':
			return self.joy_data.axes[self.controllerMapping[type][name]]*self.controllerMask[type][name]
		elif type=='button':
			return self.joy_data.button[self.controllerMapping[type][name]]*self.controllerMask[type][name]

if __name__ == '__main__':
	try:
		move_gantry = xbox_controller()
		t = Thread(target=move_gantry.updateStewartPosition)
		t.start()
		rospy.spin()
		move_gantry.alive = False
		t.join()
	except rospy.ROSInterruptException: pass