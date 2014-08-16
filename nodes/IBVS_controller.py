#!/usr/bin/env python

PACKAGE = 'stewart_platform'
import roslib
roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import sin, cos, pi, sqrt, asin, atan2


class stewart_platform:
	def __init__(self): #expects the stewart_platform params from platformModel.yaml
		# load parameters for robot and startup the model
		self.tgtEdge = rospy.get_param('tgtEdge')
		self.focalLength = rospy.get_param("camera/focal_length") / 1000
		self.imageWidth = rospy.get_param("camera/image_width")
		self.imageHeight = rospy.get_param("camera/image_height")
		self.detPitch = rospy.get_param("camera/detPitch")
		self.detPitchM = self.defPitch / 1000000
		self.sensorWidth = self.detPitchM*self.imageWidth
		self.sensorHeight = self.detPitchM*self.imageHeight
		self.processingScale = rospy.get_param("processingScale")
		self.tgtEdge = rospy.get_param("tgtEdge")
		self.circleRadius = rospy.get_param("circleRadius")
		self.lambda = rospy.get_param("lambda")
		self.zOffset = rospy.get_param("zOffset")
		
		self.hTc = rotationMatrix(0,pi/2,pi/2)
	
def rotationMatrix(x,y,z):
	M = np.dot( np.identity(3), np.array( [[1,0,0],[0,cos(x),-sin(x)],[0,sin(x),cos(x)] ] ) ) # angle x
	M = np.dot( M, np.array( [[cos(y),0,sin(y)],[0,1,0],[-sin(y),0,cos(y)] ] ) ) # angle y
	M = np.dot( M, np.array( [[cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1] ] ) ) # angle z
	return M
def translationMatrix(x,y,z):
	return np.array([[x,y,z],[x,y,z],[x,y,z]] )	
if __name__ == '__main__':
	print "loading model", rospy.get_param('stewart_platform')
	platform = stewart_platform( rospy.get_param('stewart_platform') )
	rospy.Subscriber('/stewart/Twist', Twist, platform.ikSolver)
	rospy.spin()
