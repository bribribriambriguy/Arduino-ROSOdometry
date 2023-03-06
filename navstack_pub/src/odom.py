#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import time
import tf
from math import sin, cos, atan2, pi, asin
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

lastLeft = 0
positionLeft = 0

lastRight = 0
positionRight = 0

ticksPerMeter = 661
wheelBase = 0.13335

x = 0
y = 0
th = 0


delta_left_wheel = 0
delta_right_wheel = 0


lastTime = 0

PI = 3.141592

def main():
	global odom_pub, odom_broadcaster
	rospy.init_node("odom_pub")
	rate = rospy.Rate(10)
	
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
	rospy.Subscriber("left_ticks", Int16, leftWheelTicks)
	rospy.Subscriber("right_ticks", Int16, rightWheelTicks)
	
	odom_broadcaster = tf.TransformBroadcaster()
	
	while not rospy.is_shutdown():
		pubOdom()
		rate.sleep()
	
def leftWheelTicks(leftCount):
		global positionLeft
		
		positionLeft = leftCount.data/ticksPerMeter
		
def rightWheelTicks(rightCount):
		global positionRight
		
		positionRight = rightCount.data/ticksPerMeter
		
def pubOdom():
	global x, y, th, lastTime, lastLeft, lastRight, delta_left_wheel, delta_right_wheel
	
	if positionLeft != 0 and positionRight != 0 and lastLeft != 0 and lastRight != 0:
	
		delta_left_wheel = (positionLeft - lastLeft)
  
		delta_right_wheel = (positionRight - lastRight)
  
	lastLeft = positionLeft
	lastRight = positionRight
	
	cycleDistance = (delta_left_wheel + delta_right_wheel)/2.0
	cycleAngle = atan2(delta_right_wheel - delta_left_wheel, wheelBase)	
	delta = rospy.Time.now().to_sec() - lastTime
	
	x += cos(th + cycleAngle / 2.0) * cycleDistance
	y += sin(th + cycleAngle / 2.0) * cycleDistance
	th += cycleAngle
	
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
	
	odom = Odometry()
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "map"
	odom.child_frame_id = "odom"
	
	odom.pose.pose = Pose(Point(x,y,0.0), Quaternion(*odom_quat))
	
	odom.twist.twist.linear.x = cycleDistance / delta
	odom.twist.twist.angular.z = cycleAngle / delta
	print(cycleDistance, cycleAngle)
	
	odom_pub.publish(odom)
	odom_broadcaster.sendTransform(
		(x,y,0.0), odom_quat, rospy.Time.now(), "odom","map")
	
	lastTime = rospy.Time.now().to_sec()			
		 				
		
		
main()		
