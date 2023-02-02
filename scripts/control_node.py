#!/usr/bin/env python3

# import ROS for developing the node
import rospy

# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from robotics_lab01.msg import Turtlecontrol

pos_msg = Pose()
param_data = Turtlecontrol()

def pose_callback(data):
	global pose_msg
	pos_msg.x = data.x

def params_callback(data):
	global param_data
	param_data.kp = data.kp
	param_data.xd = data.xd

if __name__ == '__main__':
	cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	pos_sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	control_params = rospy.Subscriber('/turtle1/control_params', Turtlecontrol, params_callback)
	
	rospy.init_node('control_node', anonymous = True)
	loop_rate = rospy.Rate(10)
	
	vel_cmd = Twist()	
	
	while not rospy.is_shutdown():
		vel_cmd.linear.x = param_data.kp * (param_data.xd - pos_msg.x) 
		
		cmd_pub.publish(vel_cmd)
		
		loop_rate.sleep()
