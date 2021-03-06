#!/usr/bin/env python
PKG = 'stewart_platform'
import roslib; roslib.load_manifest(PKG)
import time
from math import pi
from threading import Thread
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from asctec_hl_comm.msg import mav_ctrl
from asctec_hl_comm.msg import mav_rcdata

class pelican_controller():
	def __init__(self):
		self.sleep_time = 1 / 5
		# +z is down for the stewart platform -- the x360.yaml denotes this with a z mask of -1
		self.lambda_matrix = {'x':0.5,'y':-0.5,'z':-0.5,'yaw':0.0} # tuning for amount and direction

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

		self.pelican_safe = mav_ctrl()
		self.pelican_safe.type = 2 # 1--acceleration  2--velocity   3--position
		self.pelican_safe.x = 0
		self.pelican_safe.y = 0
		self.pelican_safe.z = 0
		self.pelican_safe.yaw = 0
		self.pelican_safe.v_max_xy = 1
		self.pelican_safe.v_max_z = 1

		self.safeIsWritten = False

		rospy.init_node('pelican_control', anonymous=True)

		self.joy_data = None
		rospy.Subscriber('/Pelican/Joy', Joy, self.controllerCallback)

		# don't have a proper node name for this yet but nothing is implemented yet
		self.pelican_pose = TransformStamped()
		rospy.Subscriber('/Pelican/PoseFeedback', TransformStamped, self.poseCallback) # rename everything to transforms
		self.pelican_safe_position = [[-1.5,1],[-1,1],[0,2]]

		self.stewartDefaultTwist = Twist() # the default twist
		self.stewartDefaultTwist.linear.z=0.21

		# even if the IK doesn't solve this twist it is still this far away from the target. It may actually turn out to help get the system unstuck
		rospy.Subscriber('/Pelican/StewartTwist', Twist, self.stewartTwistCallback)
		self.stewartCurrentTwist=None

	def controllerCallback(self, data): # needs to detect what 
		self.joy_data = data

	def poseCallback(self,data): # brings in the pelican pose for a positioning threshold
		self.pelican_pose = data

	def stewartTwistCallback(self,data):
		self.stewartCurrentTwist = data

	# maintains the control loop even in the event of IBVS stopping
	def updatePelican(self):
		while not rospy.is_shutdown():
			if (not self.joy_data is None and (not self.stewartCurrentTwist is None)):
				#print "SAFE?: ", self.isSafePosition()
				#print "pose: ", self.pelican_pose
				#print "curTwist: ", self.stewartCurrentTwist
				if (self.joy_data.axes[2] < -0.9 and self.isSafePosition()): # deadmans switch
					self.safeIsWritten = False
					self.pelican_ctrl.type = 2
					self.pelican_ctrl.x = (self.stewartCurrentTwist.linear.x-self.stewartDefaultTwist.linear.x) * self.lambda_matrix['x']
					self.pelican_ctrl.y = (self.stewartCurrentTwist.linear.y-self.stewartDefaultTwist.linear.y) * self.lambda_matrix['y']
					self.pelican_ctrl.z = (self.stewartCurrentTwist.linear.z-self.stewartDefaultTwist.linear.z) * self.lambda_matrix['z']
					self.pelican_ctrl.yaw = (self.stewartCurrentTwist.angular.z-self.stewartDefaultTwist.angular.z) * self.lambda_matrix['yaw']
					self.pelican_ctrl.v_max_xy = 1
					self.pelican_ctrl.v_max_z = 1
					self.pelican_ctrl_pub.publish(self.pelican_ctrl)
					print self.pelican_ctrl
				elif (self.safeIsWritten==False): # debounces the safe command write
					self.safeIsWritten = True
					self.pelican_ctrl_pub.publish(self.pelican_safe)
			time.sleep(self.sleep_time)

	def isSafePosition(self):
		return (self.pelican_safe_position[0][0]<self.pelican_pose.transform.translation.x<self.pelican_safe_position[0][1]) and (self.pelican_safe_position[1][0]<self.pelican_pose.transform.translation.y<self.pelican_safe_position[1][1]) and (self.pelican_safe_position[2][0]<self.pelican_pose.transform.translation.z<self.pelican_safe_position[2][1])

if __name__ == '__main__':
	try:
		move_pelican = pelican_controller()
		t = Thread(target=move_pelican.updatePelican)
		t.start()
		rospy.spin()
	except rospy.ROSInterruptException: pass