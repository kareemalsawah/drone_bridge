#! /usr/bin/env python3

import airsim 
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import numpy as np


#ros
rospy.init_node('cheating_SLAM')

r = rospy.Rate(10)

h,w = None, None
rgb_img = None
depth_img = None
changed_rgb,changed_depth = False,False

def save_img(data):
    global rgb_img
    global h
    global w
    global changed_rgb
    if h is None:
        h,w = data.width,data.height
    rgb_img = np.frombuffer(data.data,dtype=np.uint8).reshape(h,w,3)
    changed_rgb = True
    
def save_depth(data):
    global depth_img
    global h
    global w
    global changed_depth
    if h is not None:
        depth_img = np.array(data.data).reshape(h,w)
        changed_depth = True

rospy.Subscriber('airsim/rgb/front', Image, queue_size= 1, callback=save_img)
rospy.Subscriber('airsim/depth/front', Float64MultiArray, queue_size= 1, callback=save_depth)

while not rospy.is_shutdown():
    if changed_rgb and changed_depth:
        changed_rgb,changed_depth = False,False
        print("Saving Stuff")
        np.save("rgb_img.npy",rgb_img)
        np.save("depth_img.npy",depth_img)
        break
    r.sleep()
    
