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
    const auto base_frame_id = pn.param<std::string>("base_frame_id", "map");

    // Publisher
    ros::Publisher poses_pub = n.advertise<geometry_msgs::PoseArray>("target_poses", 1);
    ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("target_marker", 1);
    ros::Publisher markers2_pub = n.advertise<visualization_msgs::Marker>("target_marker2", 1);
    
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
    poses_msg.header.frame_id = base_frame_id;
    poses_msg.poses.emplace_back(pose_init(0.0, 0.0));

    const int target_pose_shape_num = pn.param<int>("target_pose_shape_num", 0);
    switch (target_pose_shape_num) {
    case 1:
        // コ shape round
        {
            const double round=0.0;
            poses_msg.poses.emplace_back(pose_init(0.0, 0.0));
            for(int i=0; i<=90; i+=10){
                poses_msg.poses.emplace_back(pose_init(0.15-round+round*cos((i-90.0)*M_PI/180), round+round*sin((i-90.0)*M_PI/180)));
            }
            for(int i=0; i<=90; i+=10){
                poses_msg.poses.emplace_back(pose_init(0.15-round+round*cos((i-0.0)*M_PI/180), 0.18-round+round*sin((i-0.0)*M_PI/180)));
            }
            poses_msg.poses.emplace_back(pose_init(0.02, 0.18));
        }
        break;

    case 2:
        // コ shape
        {
            poses_msg.poses.emplace_back(pose_init(0.0, 0.0));
            poses_msg.poses.emplace_back(pose_init(0.1, 0.0));
            poses_msg.poses.emplace_back(pose_init(0.1, 0.15));
            poses_msg.poses.emplace_back(pose_init(0.0, 0.15));
        }
        break;

    case 3:
        // circle shape
        {
            const auto radius = 0.1;
            for(int i=0; i<=200; i+=10){
                poses_msg.poses.emplace_back(pose_init(radius*sin(i*M_PI/180), radius-radius*cos(i*M_PI/180)));
            }
        }
        break;
        
    case 4:
        // cos shape
        {
            for(int i=0; i<540; i+=10){
                poses_msg.poses.emplace_back(pose_init(double(i)/1000.0+0.02,0.05*(cos(i*M_PI/180)-1)));
            }
        }
        break;

    case 5:
        // triangle shape
        {
            poses_msg.poses.emplace_back(pose_init(0.2, 0.0));
            poses_msg.poses.emplace_back(pose_init(0.01, 0.2));
        }
        break;

    case 6:
        // small circle shape
        {
            const auto radius = 0.05;
            poses_msg.poses.emplace_back(pose_init(0.0, 0.0));
            for(int i=0; i<=180; i+=10){
                poses_msg.poses.emplace_back(pose_init(0.03+radius*sin(i*M_PI/180), radius-radius*cos(i*M_PI/180)));
            }
            for(int i=0; i<=170; i+=10){
                poses_msg.poses.emplace_back(pose_init(0.03-radius*sin(i*M_PI/180), 3.0*radius-radius*cos(i*M_PI/180)));
            }
        }
        break;
    
    case 7:
        // U shape
        {
            const auto radius = 0.08;
            for(int i=0; i<=180; i+=10){
                poses_msg.poses.emplace_back(pose_init(radius*sin(i*M_PI/180), radius-radius*cos(i*M_PI/180)));
            }
            poses_msg.poses.emplace_back(pose_init(-0.05, 2.0*radius));
        }
        break;
    
    case 8:
        //c  shape
        {
            const auto radius = 0.16;
            for(int i=0; i<=60; i+=5){
                poses_msg.poses.emplace_back(pose_init(0.03+radius*sin(i*M_PI/180), -(radius-radius*cos(i*M_PI/180))));
            }
            //poses_msg.poses.emplace_back(pose_init(-0.05, 2.0*radius));
        }
        break;

    case 9:
        //linear shape
        {
            poses_msg.poses.emplace_back(pose_init(0.01, 0.0));
            poses_msg.poses.emplace_back(pose_init(0.09, 0.0));
            poses_msg.poses.emplace_back(pose_init(0.1, 0.0));
            poses_msg.poses.emplace_back(pose_init(0.15, 0.0));
        }
        break;

    case 10:
        //hyotan  shape
        {
            poses_msg.poses.emplace_back(pose_init(0.05, 0.0));
            poses_msg.poses.emplace_back(pose_init(0.125, 0.05));
            poses_msg.poses.emplace_back(pose_init(0.2, 0.03));
            poses_msg.poses.emplace_back(pose_init(0.275, 0.05));
            //poses_msg.poses.emplace_back(pose_init(0.35, 0.0));
        }
        break;
    
    case 11:
        //hyotan  shape exp
        {
            for(int i=0; i<=280; i+=10){
                const double x=0.01*i;
                poses_msg.poses.emplace_back(pose_init(double(i)/2000.0+0.01,0.007*(cos(i*M_PI/180)-1)*(cos(i*M_PI/180)-1)));
            }
            for(int i=400; i<=620; i+=10){
                const double x=0.01*i;
                poses_msg.poses.emplace_back(pose_init(double(i-100)/2000.0+0.01,0.007*(cos(i*M_PI/180)-1)*(cos(i*M_PI/180)-1)));
            }
        }
        break;
    
    case 12:
        //hyotan  shape exp
        {
            // cos shape
        {
            for(int i=0; i<650; i+=10){
                poses_msg.poses.emplace_back(pose_init(double(i)/2500.0+0.01,-0.02*(cos(i*M_PI/180)-1)));
            }
        }
        }
        break;
    
    case 13:
        //c  shape
        {
            const auto radius = 0.11;
            for(int i=0; i<=125; i+=5){
                poses_msg.poses.emplace_back(pose_init(0.0+radius*sin(i*M_PI/180), -(radius-radius*cos(i*M_PI/180))));
            }
            //poses_msg.poses.emplace_back(pose_init(-0.05, 2.0*radius));
        }
        break;
    
    case 14:
        //mini hyotan
        {
            const auto radius = 0.05;
            for(int i=40; i<=140; i+=5){
                poses_msg.poses.emplace_back(pose_init(radius-radius*cos(i*M_PI/180),-0.03+radius*sin(i*M_PI/180)));
            }
            for(int i=40; i<=140; i+=5){
                poses_msg.poses.emplace_back(pose_init(0.1+radius-radius*cos(i*M_PI/180), -0.03+radius*sin(i*M_PI/180)));
            }
            //poses_msg.poses.emplace_back(pose_init(-0.05, 2.0*radius));
        }
        break;
    case 15:
        //mini c  shape
        {
            const auto radius = 0.11;
            for(int i=0; i<=60; i+=5){
                poses_msg.poses.emplace_back(pose_init(0.0+radius*sin(i*M_PI/180), -(radius-radius*cos(i*M_PI/180))));
            }
            //poses_msg.poses.emplace_back(pose_init(-0.05, 2.0*radius));
        }
        break;

    case 16:
        //c  shape
        {
            const auto radius = 0.11;
            for(int i=0; i<=90; i+=5){
                poses_msg.poses.emplace_back(pose_init(0.03+radius*sin(i*M_PI/180), -(radius-radius*cos(i*M_PI/180))));
            }
            //poses_msg.poses.emplace_back(pose_init(-0.05, 2.0*radius));
        }
        break;

    default:
        break;
    }

    
    //offset
    for(int i=1; i<poses_msg.poses.size(); i++){
        poses_msg.poses.at(i).position.x += (vendor_num+1)*arm_unit_length;
    }

    while (n.ok())
    {

        visualization_msgs::Marker marker;
        marker.header.frame_id = base_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_poses";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = poses_msg.poses.at(0);
        marker.lifetime = ros::Duration();
        marker.scale.x = 0.008;
        marker.scale.y = 0.008;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        for(const auto pose : poses_msg.poses){
            marker.points.push_back(pose.position);
        }
        markers_pub.publish(marker);
        
        marker.ns = "target_poses2";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.002;
        markers2_pub.publish(marker);
        poses_msg.header.stamp = ros::Time::now();
        poses_pub.publish(poses_msg);
        ros::spinOnce();    
        loop_rate.sleep();

    }
    return 0;
}