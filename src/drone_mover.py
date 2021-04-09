#! /usr/bin/env python3

import airsim
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

import numpy as np

# Initializing airsim client node and enabling API control
global mutex_lock
mutex_lock = False
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.simPause(False)

# Initializing ROS node
rospy.init_node("airsim_vehicle_mover")
r = rospy.Rate(10)

# Defining subscriber callback functions
def to_loc_by_vel(data):
	global mutex_lock
	if not mutex_lock:
	    mutex_lock = True
	    if len(data.data) == 4:
	        client.moveByVelocityAsync(data.data[0],data.data[1],data.data[2],data.data[3])
	    else:
	        rospy.logwarn("Control Move to location by velocity expected list of size 4 but received one of size {}".format(len(data.data)))
	    
	    mutex_lock = False

rospy.Subscriber('controls/to_loc_by_vel', Float64MultiArray, queue_size= 1, callback= to_loc_by_vel)


while not rospy.is_shutdown():
	try:
		pass
	except KeyboardInterrupt:
		client.enableApiControl(False)
		exit()
