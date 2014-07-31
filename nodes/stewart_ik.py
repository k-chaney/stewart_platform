#!/usr/bin/env python

PACKAGE = 'stewart_platform'
import roslib
roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
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


		self.angle_base = np.zeros(6)
		self.pos_base = np.zeros((6,3))
		self.pos_top = np.zeros((6,3))

		for i in range (1,4): # 1,2,3
			# base points
			angle_m_b = (2*pi/3)* (i-1) - self.alpha_b
			angle_p_b = (2*pi/3)* (i-1) + self.alpha_b
			self.angle_base[2*i-2] = angle_m_b + self.alpha_b2
			self.angle_base[2*i-1] = angle_p_b - self.alpha_b2
			self.pos_base[2*i-2] = self.radius_b * np.array([cos(angle_m_b), sin(angle_m_b), 0.0])
			self.pos_base[2*i-1] = self.radius_b * np.array([cos(angle_p_b), sin(angle_p_b), 0.0])

			# top points (with a 60 degree offset)
			angle_m_t = (2*pi/3)* (i-1) - self.alpha_t + self.top_offset_angle
			angle_p_t = (2*pi/3)* (i-1) + self.alpha_t + self.top_offset_angle
			self.pos_top[2*i-2] = self.radius_t * np.array([cos(angle_m_t), sin(angle_m_t), 0.0]) 
			self.pos_top[2*i-1] = self.radius_t * np.array([cos(angle_p_t), sin(angle_p_t), 0.0])

			#robot = {pos_top pos_base angle_base L_m L_s Theta_min Theta_max title_str};

		# some ros stuff
		self.angle_base = self.angle_base + pi/2
		self.curTwist = Twist()
		rospy.init_node("stewart_platform")
		self.joint_pub = rospy.Publisher('/stewart/JointState',JointState)


	def ikSolver(self,msg): #twist message expected
		twist = np.array( [ msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z ] )
		print twist
		
		# B is self.pos_base
		# P is self.pos_top
		# gamma is self.angle_base
		# R is self.L_m
		# D is self.L_s
		a = twist[3]
		platform_pose = self.pos_top * np.array( [[1,0,0],[0,cos(a),-sin(a)],[0,sin(a),cos(a)] ] ) # angle x
		a = twist[4]
		platform_pose = self.pos_top * np.array( [[cos(a),0,sin(a)],[0,1,0],[-sin(a),0,cos(a)] ] ) # angle y
		a = twist[5]
		platform_pose = self.pos_top * np.array( [[cos(a),-sin(a),0],[sin(a),cos(a),0],[0,0,1] ] ) # angle z
		platform_pose[:,0] = platform_pose[:,0] + twist[0] # translate x
		platform_pose[:,1] = platform_pose[:,1] + twist[1] # translate y
		platform_pose[:,2] = platform_pose[:,2] + twist[2] # translate z
		self.stweart_joints = JointState() # will be passed to ros
		self.joint_names = ["joint_%d" % i for i in range(0,6) ] # joint names
		self.joint_positions = np.zeros( (6) ) # positions will be changed
		joint_velocities = np.zeros( (6) ) # won't change
		joint_efforts = np.zeros( (6) ) # won't change
		success = 0 # success check
		
		for i in range(0,6):
			L = np.linalg.norm(platform_pose(i)-self.pos_base(i))
			# I don't understand this......
			a = 2*self.L_m*( platform_pose(i,3) - self.pos_base(i,3) )
			b = 2*self.L_m*( sin(self.angle_base(i)) * (platform_pose(i,1)-self.pos_base(i,1)) - cos(self.angle_base(i)) * ( platform_pose(i,2)-self.pos_base(i,2) ) )
			c = ( L^2 - self.L_s^2 + self.L_m^2 )

		

if __name__ == '__main__':
	print "loading model", rospy.get_param('stewart_platform')
	platform = stewart_platform( rospy.get_param('stewart_platform') )
	rospy.Subscriber('/stewart/Twist', Twist, platform.ikSolver)
	rospy.spin()