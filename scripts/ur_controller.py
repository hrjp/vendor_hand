#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import rtde_receive
import rtde_control
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
class ur_controller_node :
    def __init__(self):
        rospy.init_node("ur_controller_node")
        
        ip = rospy.get_param('ip', '192.168.0.119')
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)

        self.target_sub = rospy.Subscriber("arm_target_value", Float32MultiArray, self.target_callback, queue_size=1)

        self.init_pose = self.rtde_r.getActualTCPPose()

    def __del__(self):
        self.rtde_c.stopScript()
    
    def target_callback(self, msg):
        pose = []
        for i in range(6):
            pose.append(msg.data[i]+self.init_pose[i])
        self.rtde_c.moveL(pose, msg.data[6], msg.data[7])


if __name__ == '__main__':
    ur_controller_node()
    rospy.spin()
