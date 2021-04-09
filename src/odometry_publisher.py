#! /usr/bin/env python3

import airsim 
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
import numpy as np

#airsim
client = airsim.MultirotorClient()
client.confirmConnection()

#ros
rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("airsim/GroundTruthPose", Odometry, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()

#inital values
current_time = rospy.Time.now()


r = rospy.Rate(60.0)

odom = Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    drone_state = client.getMultirotorState()

    x = drone_state.kinematics_estimated.position.x_val
    y = drone_state.kinematics_estimated.position.y_val
    z = -1* drone_state.kinematics_estimated.position.z_val
    #print("Position: {:.2f},{:.2f},{:.2f}".format(x,y,z))


    wo = drone_state.kinematics_estimated.orientation.w_val
    xo = drone_state.kinematics_estimated.orientation.x_val
    yo = drone_state.kinematics_estimated.orientation.y_val
    zo = -1 * drone_state.kinematics_estimated.orientation.z_val

    vx = drone_state.kinematics_estimated.linear_velocity.x_val
    vy = drone_state.kinematics_estimated.linear_velocity.y_val
    vz = drone_state.kinematics_estimated.linear_velocity.z_val
    v_ang_x = drone_state.kinematics_estimated.angular_velocity.x_val
    v_ang_y = drone_state.kinematics_estimated.angular_velocity.y_val
    v_ang_z = drone_state.kinematics_estimated.angular_velocity.z_val
    #print("Velocity: {:.2f},{:.2f},{:.2f}".format(vx,vy,vz))
    #print("Ang Vel: {:.2f},{:.2f},{:.2f}".format(v_ang_x,v_ang_y,v_ang_z))
    
    odom_quat = (xo, yo, zo, wo)
    odom_eul  = tf.transformations.euler_from_quaternion(odom_quat)
    #print("Orientation: {}".format(odom_eul))

    odom.header.stamp = current_time
    odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(v_ang_x, v_ang_y, v_ang_z))
    odom_pub.publish(odom)
    
    r.sleep()
