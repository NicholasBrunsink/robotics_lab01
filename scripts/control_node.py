#!/usr/bin/env python3

# import ROS for developing the node
import rospy

# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
# import turtlesim/Pose for current location data
from turtlesim.msg import Pose

# import robotics_lab01/Turtlecontrol for gain and destination parameters
from robotics_lab01.msg import Turtlecontrol

# initialize a Pose message to store turtle's Pose data
pos_msg = Pose()
# initialize a Turtlecontrol message to store parameter data 
param_data = Turtlecontrol()

# store data from Pose subscriber into pos_msg
def pose_callback(data):
	global pose_msg
	pos_msg.x = data.x

# store data from Turtlecontrol subscriber into param_data
def params_callback(data):
	global param_data
	param_data.kp = data.kp
	param_data.xd = data.xd

if __name__ == '__main__':
	# Initialize publisher to publish commands for turtle
	cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	# Initialize subscriber to listen to turtle's Pose msg to obtain turtle location info
	pos_sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	# Initialize subscriber to Turtlecontrol msg to obtain gain and desired location
	control_params = rospy.Subscriber('/turtle1/control_params', Turtlecontrol, params_callback)
	
	# Initialize node
	rospy.init_node('control_node', anonymous = True)
	# set up a loop frequency for the control loop (10Hz)
	loop_rate = rospy.Rate(10)
	# initialize a Twist message to store data from loop
	vel_cmd = Twist()	
	
	# set up a loop that runs at a 10Hz frequency
	while not rospy.is_shutdown():
		# set linear velocity to a value based on inputted parameters and current location (m/s)
		vel_cmd.linear.x = param_data.kp * (param_data.xd - pos_msg.x) 
		# publish the command to the sim
		cmd_pub.publish(vel_cmd)
		# wait for 0.1 of second until the next loop
		loop_rate.sleep()
