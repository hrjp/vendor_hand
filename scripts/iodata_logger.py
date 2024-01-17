#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
import rospy
import rosparam
from std_msgs.msg import Float32, Float32MultiArray, Empty
import numpy as np

start_flag = False
start_time = rospy.Time
def start_callback(msg):
    global start_flag, start_time
    start_flag = True
    start_time = rospy.Time.now()

now_angle_array = np.empty((0,2), float)
def now_angle_callback(msg):
    global start_flag, start_time
    if not start_flag:
        return
    now_time = (rospy.Time.now() - start_time).to_sec()
    now_angle = msg.data
    global now_angle_array
    now_angle_array = np.append(now_angle_array, np.array([[now_time, now_angle]]), axis=0)

now_linear_pos_array = np.empty((0,2), float)
def now_linear_pos_callback(msg):
    global start_flag, start_time
    if not start_flag:
        return
    now_time = (rospy.Time.now() - start_time).to_sec()
    now_linear_pos = msg.data
    global now_linear_pos_array
    now_linear_pos_array = np.append(now_linear_pos_array, np.array([[now_time, now_linear_pos]]), axis=0)

angles_array = np.empty((0,1), float)
def angles_callback(msg):
    global angles_array
    angles_array = np.array(msg.data).transpose()

orientations_array = np.empty((0,1), float)
def orientations_callback(msg):
    global orientations_array
    orientations_array = np.array(msg.data).transpose()


def main():
    rospy.init_node('iodata_logger')
    save_dir = rospy.get_param('~save_dir', '/home/catkin_ws/src/vendor_hand/log/')
    # subscriber
    rospy.Subscriber('now_angle', Float32, now_angle_callback)
    rospy.Subscriber('now_linear_pos', Float32, now_linear_pos_callback)
    rospy.Subscriber('angles', Float32MultiArray, angles_callback)
    rospy.Subscriber('orientations', Float32MultiArray, orientations_callback)
    rospy.Subscriber('start', Empty, start_callback)
    rospy.spin()

    # save
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
    global now_angle_array, now_linear_pos_array, angles_array, orientations_array
    np.savetxt(save_dir+"read_angle.csv",now_angle_array,delimiter=",")
    np.savetxt(save_dir+"read_linear_pos.csv",now_linear_pos_array,delimiter=",")
    np.savetxt(save_dir+"target_angles.csv",angles_array,delimiter=",")
    np.savetxt(save_dir+"read_orientations.csv",orientations_array,delimiter=",")

if __name__ == '__main__':
    main()
