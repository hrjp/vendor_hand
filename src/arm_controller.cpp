/**
 * @file arm_controller.cpp
 * @brief
 * @author Shunya Hara
 * @date 2023.6.15
 * @details
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// global variable
std::vector<geometry_msgs::Point> target_points;

void posesCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    target_points.clear();
    for(const auto &pose : msg->poses)
    {
        target_points.push_back(pose.position);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle n;
    // param setting
    ros::NodeHandle pn("~");
    ros::Rate loop_rate(10);

    // Publisher
    
    //subscriber
    ros::Subscriber poses_sub = n.subscribe("target_poses", 1, posesCallback);

    while (n.ok())
    {

        
        ros::spinOnce();    
        loop_rate.sleep();

    }
    return 0;
}