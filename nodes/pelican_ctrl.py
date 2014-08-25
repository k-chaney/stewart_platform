#!/usr/bin/env python
PKG = 'stewart_platform'
import roslib; roslib.load_manifest(PKG)
import time
from math import pi
from threading import Thread
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from asctec_hl_comm.msg import mav_ctrl
from asctec_hl_comm.msg import mav_rcdata

class xbox_controller():
	def __init__(self):
		self.lmbda = 0.2 # tuning

		self.pelican_ctrl_pub = rospy.Publisher('/Pelican/ctrl', mav_ctrl)
		self.pelican_ctrl = mav_ctrl()

		# velocity with default parameters (should be safe)
		self.pelican_ctrl.type = 2 # 1--acceleration  2--velocity   3--position
		self.pelican_ctrl.x = 0
		self.pelican_ctrl.y = 0
		self.pelican_ctrl.z = 0
		self.pelican_ctrl.yaw = 0
		self.pelican_ctrl.v_max_xy = 1
		self.pelican_ctrl.v_max_z = 1

		self.typeCtrl = 7 # controller channel to listen to
		self.velocityMark = 2040 # the channel output when it's set to velocity


		rospy.init_node('pelican_joy', anonymous=True)


		self.joy_data = None
		rospy.Subscriber('/Pelican/Joy', mav_rcdata, self.controllerCallback)

		self.pelican_pose = PoseStamped()
		rospy.Subscriber('/Pelican/Pose_Feedback', PoseStamped, self.poseCallback)

		self.stewartDefaultTwist = Twist() # the default twist
		self.stewartDefaultTwist.linear.z=0.25

		rospy.Subscriber('/Pelican/Stewart_Twist', Twist, self.stewartTwistCallback)

	def controllerCallback(self, data): # needs to detect what 
		self.joy_data = data

	def poseCallback(self,data): # brings in the pelican pose for a positioning threshold
		self.pelican_pose = data

	def stewartTwistCallback(self,stewartCurrentTwist):
		if (self.joy_data.channel[self.typeCtrl] == self.velocityMark):
			self.pelican_ctrl.type = 2
			self.pelican_ctrl.x = (stewartCurrentTwist.linear.x-stewartDefaultTwist.linear.x) * self.lmbda
			self.pelican_ctrl.y = (stewartCurrentTwist.linear.y-stewartDefaultTwist.linear.y) * self.lmbda
			self.pelican_ctrl.z = (stewartCurrentTwist.linear.z-stewartDefaultTwist.linear.z) * self.lmbda
			self.pelican_ctrl.yaw = (stewartCurrentTwist.angular.z-stewartDefaultTwist.angular.z) * self.lmbda
			self.pelican_ctrl.v_max_xy = 1
			self.pelican_ctrl.v_max_z = 1

			self.pelican_ctrl_pub.publish(self.pelican_ctrl)

if __name__ == '__main__':
	try:
		move_stewart = xbox_controller()
		rospy.spin()
	except rospy.ROSInterruptException: pass