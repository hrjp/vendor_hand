/**
 * @file arm_sim.cpp
 * @brief arm sim
 * @author Shunya Hara
 * @date 2023.6.13
 * @details
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    std_msgs::Float32 angle_msg;
    std_msgs::Float32 linear_vel_msg;
    angle_msg.data = msg->axes.at(3) *;
    linear_vel_msg.data = msg->axes.at(1) * 0.05;
    angle_pub.publish(angle_msg);
    linear_vel_pub.publish(linear_vel_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wp_loader_node");
    ros::NodeHandle n;
    // 制御周期10Hz
    ros::Rate loop_rate(10);

    // param setting
    ros::NodeHandle pn("~");

    // Publisher
    ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("angle", 1);
    ros::PUblisher linear_vel_pub = n.advertise<std_msgs::Float32>("linear_vel", 1);
    // subscriber
    ros::Subscriber joy_sub = n.subscribe("joy", 1, joyCallback);

    ros::spin();
    return 0;
}