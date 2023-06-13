/**
 * @file joy_to_value.cpp
 * @brief
 * @author Shunya Hara
 * @date 2023.6.13
 * @details
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Joy.h>

std_msgs::Float32 angle_msg;
std_msgs::Float32 linear_vel_msg;
void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    
    angle_msg.data = msg->axes.at(3) * 0.5;
    linear_vel_msg.data = msg->axes.at(1) * 0.05;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_value_node");
    ros::NodeHandle n;
    // param setting
    ros::NodeHandle pn("~");
    ros::Rate loop_rate(20);

    // Publisher
    ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("angle", 1);
    ros::Publisher linear_vel_pub = n.advertise<std_msgs::Float32>("linear_vel", 1);
    // subscriber
    ros::Subscriber joy_sub = n.subscribe("joy", 1, joyCallback);
    while (n.ok())
    {
        angle_pub.publish(angle_msg);
        linear_vel_pub.publish(linear_vel_msg);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}