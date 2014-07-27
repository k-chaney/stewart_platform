#!/usr/bin/env python

PACKAGE = 'stewart_platform'
import roslib
roslib.load_manifest(PACKAGE)
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from stewart_platform import stewart_platform
from math import sin, cos, pi


class stewart_platform:
	def __init__(self,params): #expects the stewart_platform params from platformModel.yaml
		# load parameters for robot and startup the model
		self.alpha_b = params['alpha_b']
		self.alpha_b2 = params['alpha_b2']
		self.alpha_t = params['alpha_t']
		self.radius_b = params['radius_b']
		self.radius_t = params['radius_t']
		self.Theta_min = params['Theta_min']
		self.Theta_max = params['Theta_max']
		self.top_offset_angle = params['top_offset_angle']
		self.L_m = params['L_m']
		self.L_s = params['L_s']


		self.angle_base = []
		self.pos_base = []
		self.pos_top = []

		for i in range (1,4): # 1,2,3
			# base points
			angle_m_b = (2*pi/3)* (i-1) - self.alpha_b
			angle_p_b = (2*pi/3)* (i-1) + self.alpha_b
			self.angle_base.append( angle_m_b + self.alpha_b2 )
			self.angle_base.append( angle_p_b - self.alpha_b2 )
			self.pos_base.append( [j*self.radius_b for j in [cos(angle_m_b), sin(angle_m_b), 0.0] ])
			self.pos_base.append( [j*self.radius_b for j in [cos(angle_p_b), sin(angle_p_b), 0.0] ])

			# top points (with a 60 degree offset)
			angle_m_t = (2*pi/3)* (i-1) - self.alpha_t + self.top_offset_angle
			angle_p_t = (2*pi/3)* (i-1) + self.alpha_t + self.top_offset_angle
			self.pos_top.append( [j*self.radius_t for j in [cos(angle_m_t), sin(angle_m_t), 0.0] ])
			self.pos_top.append( [j*self.radius_t for j in [cos(angle_p_t), sin(angle_p_t), 0.0] ])

			#robot = {pos_top pos_base angle_base L_m L_s Theta_min Theta_max title_str};

		# some ros stuff
		self.curTwist = Twist()
		rospy.init_node("stewart_platform")
		self.joint_pub = rospy.Publisher('/stewart/JointState',JointState)


	def ikSolver(self,msg): #twist message expected
		self.curTwist = msg

if __name__ == '__main__':
	print "loading model", rospy.get_param('stewart_platform')
    platform = stewart_platform( rospy.get_param('stewart_platform') )
    rospy.Subscriber('/stewart/Twist', Twist, platform.ikSolver)
    rospy.spin()