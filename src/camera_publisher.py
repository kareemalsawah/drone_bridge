#! /usr/bin/env python3

import rospy
import airsim
import numpy as np
import ros_numpy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32,Float64MultiArray,MultiArrayLayout,MultiArrayDimension

client = airsim.MultirotorClient()
client.confirmConnection()

#Initializing ros node 
rospy.init_node('camera_publisher')


# Publishers for rgb images
front_cam_pub = rospy.Publisher('airsim/rgb/front', Image, queue_size=1)
'''
right_cam_pub = rospy.Publisher('airsim/rgb/right', Image, queue_size=1) 
left_cam_pub = rospy.Publisher('airsim/rgb/left', Image, queue_size=1) 
back_cam_pub = rospy.Publisher('airsim/rgb/back', Image, queue_size=1) 
top_cam_pub = rospy.Publisher('airsim/rgb/top', Image, queue_size=1) 
bottom_cam_pub = rospy.Publisher('airsim/rgb/bottom', Image, queue_size=1) 
'''

# Publishers for depth images
front_depth_pub = rospy.Publisher('airsim/depth/front', Float64MultiArray, queue_size=1)
'''
right_depth_pub = rospy.Publisher('airsim/depth/right', Float64MultiArray, queue_size=1) 
left_depth_pub = rospy.Publisher('airsim/depth/left', Float64MultiArray, queue_size=1) 
back_depth_pub = rospy.Publisher('airsim/depth/back', Float64MultiArray, queue_size=1) 
top_depth_pub = rospy.Publisher('airsim/depth/top', Float64MultiArray, queue_size=1) 
bottom_depth_pub = rospy.Publisher('airsim/depth/bottom', Float64MultiArray, queue_size=1) 
'''

#cam_pubs = [front_cam_pub,right_cam_pub,left_cam_pub,back_cam_pub,top_cam_pub,bottom_cam_pub]
#depth_pubs = [front_depth_pub,right_depth_pub,left_depth_pub,back_depth_pub,top_depth_pub,bottom_depth_pub]
cam_pubs = [front_cam_pub]
depth_pubs = [front_depth_pub]

    
#frame_ids = ["front_camera_link","right_camera_link","left_camera_link","back_camera_link","top_camera_link","botton_camera_link"]
#cam_names = ["front","right","left","back","top","bottom"]
frame_ids = ["front_camera_link"]
cam_names = ["front"]

def create_depth_msg(depth_img):
    command = Float64MultiArray()
    layout = MultiArrayLayout()
    dimension = MultiArrayDimension()
    dimension.label = "depth_img"
    dimension.size = len(depth_img)
    dimension.stride = len(depth_img)
    layout.data_offset = 0
    layout.dim = [dimension]
    command.layout = layout
    command.data = depth_img
    return command

r = rospy.Rate(100) # FPS of the cameras
    
while not rospy.is_shutdown():
    
    requests_arr = []
    for cam in cam_names:
        requests_arr.append(airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)) # rgb image request
        requests_arr.append(airsim.ImageRequest(cam, airsim.ImageType.DepthPerspective, True, False)) # depth image request
    tic = time.time()
    responses = client.simGetImages(requests_arr)
    toc = time.time()
    #print((toc-tic)*1000)
    for idx in range(len(cam_names)):
        height,width = responses[2*idx].height,responses[2*idx].width
        
        # RGB Image publishing
        rgb_img = np.frombuffer(responses[2*idx].image_data_uint8, dtype=np.uint8).reshape(height,width,3)
        ros_msg_rgb = ros_numpy.msgify(Image, rgb_img, encoding="bgr8")
        ros_msg_rgb.header.stamp = rospy.Time.now()
        ros_msg_rgb.header.frame_id = frame_ids[idx]
        cam_pubs[idx].publish(ros_msg_rgb)
        #right_cam_pub.publish(ros_msg_rgb)
        
        # Depth Image publishing
        depth_img = np.array(responses[2*idx+1].image_data_float).reshape(-1)
        ros_msg_depth = create_depth_msg(depth_img.tolist())
        depth_pubs[idx].publish(ros_msg_depth)
    
    r.sleep()
