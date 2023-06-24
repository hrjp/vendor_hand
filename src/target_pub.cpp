/**
 * @file target_pub.cpp
 * @brief
 * @author Shunya Hara
 * @date 2023.6.15
 * @details
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_pub_node");
    ros::NodeHandle n;
    // param setting
    ros::NodeHandle pn("~");
    ros::Rate loop_rate(10);
    const auto arm_unit_length = pn.param<double>("arm_unit_length", 0.013);
    const auto vendor_num = pn.param<int>("vendor_num", 1);

    // Publisher
    ros::Publisher poses_pub = n.advertise<geometry_msgs::PoseArray>("target_poses", 1);
    ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("target_marker", 1);
    geometry_msgs::PoseArray poses_msg;
    auto pose_init = [](double x, double y, double z=0.0, double qw=1.0, double qx=0.0, double qy=0.0, double qz=0.0) {
        geometry_msgs::Pose pose;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
    };
    poses_msg.header.frame_id = "map";
    poses_msg.poses.emplace_back(pose_init(0.0, 0.0));

    // コ shape
    /*
    poses_msg.poses.emplace_back(pose_init(0.0, 0.0));
    poses_msg.poses.emplace_back(pose_init(0.2, 0.0));
    poses_msg.poses.emplace_back(pose_init(0.2, 0.3));
    poses_msg.poses.emplace_back(pose_init(0.0, 0.3));
    */
    // circle shape
    
    const auto radius = 0.1;
    for(int i=0; i<=180; i+=10){
        poses_msg.poses.emplace_back(pose_init(radius*sin(i*M_PI/180), radius-radius*cos(i*M_PI/180)));
    }
    
    // cos shape
    /*
    for(int i=0; i<540; i+=10){
        poses_msg.poses.emplace_back(pose_init(double(i)/750.0,0.05*(cos(i*M_PI/180)-1)));
    }
    */

    //offset
    for(int i=1; i<poses_msg.poses.size(); i++){
        poses_msg.poses.at(i).position.x += (vendor_num+1)*arm_unit_length;
    }

    while (n.ok())
    {

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_poses";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = poses_msg.poses.at(0);
        marker.scale.x = 0.005;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        for(const auto pose : poses_msg.poses){
            marker.points.push_back(pose.position);
        }
        markers_pub.publish(marker);

        poses_msg.header.stamp = ros::Time::now();
        poses_pub.publish(poses_msg);
        ros::spinOnce();    
        loop_rate.sleep();

    }
    return 0;
}