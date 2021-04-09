#! /usr/bin/env python3

import airsim
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

import numpy as np
from pynput.keyboard import Key,Listener

client = airsim.MultirotorClient()
client.confirmConnection()
control_pub = rospy.Publisher('controls/to_loc_by_vel', Float64MultiArray, queue_size=1)
# Initializing ROS node
rospy.init_node("keyboard_controller")
global shift
shift = 2

def publish(vals):
    command = Float64MultiArray()
    layout = MultiArrayLayout()
    dimension = MultiArrayDimension()
    dimension.label = "keyboard_control"
    dimension.size = 4
    dimension.stride = 4
    layout.data_offset = 0
    layout.dim = [dimension]
    command.layout = layout
    command.data = vals
    control_pub.publish(command)
    
def move(direction):
    global shift
    try:
        position = client.getMultirotorState().kinematics_estimated.position
        #target_pos = np.array([position.x_val,position.y_val,position.z_val])
        target_pos = np.array([0,0,0])
        #print("{0}".format(direction))
        direction = direction.char
        if direction == 'e': # up
            target_pos = np.array([0,0,-1*shift])
        elif direction == 'q': # down
            target_pos = np.array([0,0,shift])
        elif direction == 'w': # forward
            target_pos = np.array([shift,0,0])
        elif direction == 'a': # left
            target_pos = np.array([0,-1*shift,0])
        elif direction == 's': # back
            target_pos = np.array([-1*shift,0,0])
        elif direction == 'd': # right
            target_pos = np.array([0,shift,0])
        
        publish(target_pos.tolist()+[5])
    except:
        pass
        
  
# Collect all event until released
with Listener(on_press = move) as listener:
    listener.join()

