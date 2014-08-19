#!/usr/bin/env python

PACKAGE = 'stewart_platform'
import roslib
roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
from numpy.linalg import pinv
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from math import sin, cos, pi, sqrt, asin, atan2
from transform import *

class IBVS:
	def __init__(self): #expects the stewart_platform params from platformModel.yaml
		# load parameters
		self.tgtEdge = rospy.get_param('tgtEdge')
		self.focalLength = rospy.get_param("camera/focal_length") / 1000
		self.imageWidth = rospy.get_param("camera/image_width")
		self.imageHeight = rospy.get_param("camera/image_height")
		self.detPitch = rospy.get_param("camera/detPitch")
		self.detPitchM = float(self.detPitch) / 1000000
		self.sensorWidth = self.detPitchM*self.imageWidth
		self.sensorHeight = self.detPitchM*self.imageHeight
		self.processingScale = rospy.get_param("processingScale")
		self.tgtEdge = rospy.get_param("tgtEdge")
		self.circleRadius = rospy.get_param("circleRadius")
		self.lmbda = rospy.get_param("lambda")
		self.zOffset = rospy.get_param("zOffset")
		
		self.hTc = rpy2tr(0, pi/2, pi/2) # just the identity for now
		self.cTh = np.linalg.inv(self.hTc) # inverse of what's above--just using the numpy inverse
		
		self.bTh = np.identity(4) # base to hand will start out at stow position--bTh will actually control the movement
		self.twist_pub = rospy.Publisher('/stewart/Twist',Twist)
		self.cur_twist = Twist()
		self.cur_twist.angular.x = 0
		self.cur_twist.angular.y = 0
		self.cur_twist.angular.z = 0
		self.cur_twist.linear.x = 0
		self.cur_twist.linear.y = 0
		self.cur_twist.linear.z = 0
		self.twist_pub.publish(self.cur_twist)

		self.u0 = self.imageWidth/2
		self.v0 = self.imageHeight/2
		pixelChange = 100
		self.uv_Pstar = np.array([[self.u0-pixelChange ,self.v0-pixelChange ],[self.u0-pixelChange ,self.v0+pixelChange ],[ self.u0+pixelChange ,self.v0+pixelChange ],[ self.u0+pixelChange ,self.v0-pixelChange  ]])
		#self.uv_Pstar = np.array( [[ 293.,  179.],[ 289.,  351.],[ 463.,  351.],[ 463.,  179.]])

	def circle_track(self,msg):
		#print msg
		x_c = 0
		y_c = 0
		for i in range(0,4):
			x_c += msg.data[3*i]
			y_c += msg.data[3*i+1]
		x_c /= 4
		y_c /= 4

		uv_pxy_ordered = np.zeros((4,2))
		uv_radii_ordered = np.zeros(4)
		for Pt in range(0,4):
			if ((msg.data[Pt*3] < x_c) and (msg.data[Pt*3+1] < y_c)):
				uv_pxy_ordered[0][0] = msg.data[Pt*3]
				uv_pxy_ordered[0][1] = msg.data[Pt*3+1]
				uv_radii_ordered[0] = msg.data[Pt*3+2]
			elif ((msg.data[Pt*3] < x_c) and (msg.data[Pt*3+1] > y_c)):
				uv_pxy_ordered[1][0] = msg.data[Pt*3]
				uv_pxy_ordered[1][1] = msg.data[Pt*3+1]
				uv_radii_ordered[1] = msg.data[Pt*3+2]
			elif ((msg.data[Pt*3] > x_c) and (msg.data[Pt*3+1] > y_c)):
				uv_pxy_ordered[2][0] = msg.data[Pt*3]
				uv_pxy_ordered[2][1] = msg.data[Pt*3+1]
				uv_radii_ordered[2] = msg.data[Pt*3+2]
			elif ((msg.data[Pt*3] > x_c) and (msg.data[Pt*3+1] < y_c)):
				uv_pxy_ordered[3][0] = msg.data[Pt*3]
				uv_pxy_ordered[3][1] = msg.data[Pt*3+1]
				uv_radii_ordered[3] = msg.data[Pt*3+2]
		#print uv_pxy_ordered
		#print uv_radii_ordered
		zEst = np.zeros(uv_radii_ordered.shape)

		for Pt in range(0,4):
			zEst[Pt] = (self.circleRadius*self.focalLength)/(uv_radii_ordered[Pt]*self.detPitchM) # this shindig helps preserve being able to calculate everything
		#print zEst
		#print self.uv_Pstar
		e = self.uv_Pstar - uv_pxy_ordered
		e = np.concatenate(e)
		#print e
		#print e
		J = visjac_p(self.focalLength,self.u0,self.v0,self.detPitchM,uv_pxy_ordered,zEst)

		v = self.lmbda * np.dot(pinv(J) , e )
		Tdelta = trnorm(diff2tr(v))
		print Tdelta

def visjac_p(f,u0,v0,rho, uv, Z):
	#print Z.shape
	#print uv.shape
	if uv.shape[0] > 2:
		L = visjac_p(f,u0,v0,rho, uv[0], Z[0])
		for i in range(1,uv.shape[0]):
			L = np.vstack( [L, visjac_p(f,u0,v0,rho, uv[i], Z[i]) ] );
		return L

	# convert to normalized image-plane coordinates
	x = (uv[0] - u0) * rho / f;
	y = (uv[1] - v0) * rho / f;
	#print x,y

	L = np.array([[1/Z, 0, -x/Z, -x*y, (1+x*x), -y],[0, 1/Z, -y/Z, -(1+y*y), x*y, x]])
	#print max([1/Z, 0, -x/Z, -x*y, (1+x*x), -y,0, 1/Z, -y/Z, -(1+y*y), x*y, x])*f
	L = -f * np.dot( np.diag(np.array([rho,rho])) , L) # again dealing with self.detPitchM being 0
	return L



def rotationMatrix(x,y,z):
	M = np.dot( np.identity(3), np.array( [[1,0,0],[0,cos(x),-sin(x)],[0,sin(x),cos(x)] ] ) ) # angle x
	M = np.dot( M, np.array( [[cos(y),0,sin(y)],[0,1,0],[-sin(y),0,cos(y)] ] ) ) # angle y
	M = np.dot( M, np.array( [[cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1] ] ) ) # angle z
	return M

def translationMatrix(x,y,z):
	return np.array([[x,y,z],[x,y,z],[x,y,z]] )

	
if __name__ == '__main__':
	rospy.init_node('IBVS_Controller')
	controller = IBVS()
	rospy.Subscriber('/tracker/circle_data',  Float64MultiArray , controller.circle_track)
	rospy.spin()
