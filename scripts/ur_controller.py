#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import rtde_receive
import rtde_control
from geometry_msgs.msg import Pose

class ur_controller_node :
    def __init__(self):
        rospy.init_node("ur_controller_node")
        
        ip = rospy.get_param('ip', '192.168.0.119')
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)

        self.pose_sub = rospy.Subscriber("arm_pose", Pose, self.pose_callback)

        self.init_pose = self.rtde_r.getActualTCPPose()

    def __del__(self):
        self.rtde_c.stopScript()
    
    def pose_callback(self, msg):
        pose = [self.init_pose[0]+msg.position.x, self.init_pose[1]+msg.position.y, self.init_pose[2]+msg.position.z,
                self.init_pose[3], self.init_pose[4], self.init_pose[5]]
        self.rtde_c.moveL(pose, 0.01, 0.2)


if __name__ == '__main__':
    ur_controller_node()
    rospy.spin()
