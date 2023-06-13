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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// global parameter
double arm_unit_length;
int arm_num;
int vendor_num;
// global variable
double angle = 0.0;

double linear_pos = 0.0;
std::vector<double> angles;

ros::Time prev_time_angle;
void angleCallback(const std_msgs::Float32::ConstPtr &msg)
{
    const auto dt = (ros::Time::now() - prev_time_angle).toSec();
    prev_time_angle = ros::Time::now();
    if (dt < 0.11)
    {
        angle += msg->data * dt;
    }
    const int linear_num = linear_pos / arm_unit_length;
    if (linear_num < 0 or linear_num > arm_num - 1)
    {
        ROS_ERROR("linear_num < 0 or linear_num > arm_num-1");
        return;
    }
    if (linear_num < vendor_num)
    {
        angles.at(arm_num - linear_num);
        return;
    }
    for (int i = 0; i < vendor_num; i++)
    {
        angles.at(arm_num - linear_num + i) = angle / double(vendor_num);
    }
}

ros::Time prev_time;
void linearVelCallback(const std_msgs::Float32::ConstPtr &msg)
{
    const auto dt = (ros::Time::now() - prev_time).toSec();
    prev_time = ros::Time::now();
    if (dt < 0.11)
    {
        linear_pos += msg->data * dt;
    }

    linear_pos = std::max(linear_pos, arm_unit_length * (vendor_num));
    linear_pos = std::min(linear_pos, arm_num * arm_unit_length);
    const int linear_num = linear_pos / arm_unit_length;
    for (int i = 0; i < arm_num - linear_num; i++)
    {
        angles.at(i) = 0.0;
    }
    if (linear_num < vendor_num)
    {
        angles.at(arm_num - linear_num);
        return;
    }
    for (int i = 0; i < vendor_num; i++)
    {
        angles.at(arm_num - linear_num + i) = angle / double(vendor_num);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_sim_node");
    ros::NodeHandle n;
    // 制御周期10Hz
    ros::Rate loop_rate(100);

    // param setting
    ros::NodeHandle pn("~");
    arm_num = pn.param<int>("arm_num", 50);
    arm_unit_length = pn.param<double>("arm_unit_length", 0.013);
    vendor_num = pn.param<int>("vendor_num", 2);
    const double arm_unit_radius = pn.param<double>("arm_unit_radius", 0.02);
    linear_pos = arm_unit_length * (vendor_num);
    prev_time = ros::Time::now();
    // Publisher
    ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("arm_sim_markers", 1);
    // subscriber
    ros::Subscriber angle_sub = n.subscribe("angle", 1, angleCallback);
    ros::Subscriber linear_sub = n.subscribe("linear_vel", 1, linearVelCallback);
    angles.resize(arm_num);

    visualization_msgs::MarkerArray markers;
    markers.markers.resize(arm_num);

    while (n.ok())
    {

        double angle_sum = 0.0;
        markers.markers[0].pose.position.x = linear_pos - (arm_num - vendor_num) * arm_unit_length;
        markers.markers[0].pose.position.y = 0.0;
        markers.markers[0].pose.position.z = 0.0;
        for (int i = 1; i < markers.markers.size(); i++)
        {
            markers.markers[i].header.frame_id = "map";
            markers.markers[i].header.stamp = ros::Time::now();
            markers.markers[i].ns = "arm";
            markers.markers[i].id = i;
            markers.markers[i].type = visualization_msgs::Marker::CYLINDER;
            markers.markers[i].action = visualization_msgs::Marker::ADD;
            angle_sum += angles.at(i);
            Eigen::Quaterniond quat(Eigen::AngleAxisd(angle_sum, Eigen::Vector3d::UnitZ()));
            Eigen::Vector3d pos(arm_unit_length, 0, 0);
            pos = quat * pos;
            markers.markers[i].pose.position.x = markers.markers[i - 1].pose.position.x + pos.x();
            markers.markers[i].pose.position.y = markers.markers[i - 1].pose.position.y + pos.y();
            markers.markers[i].pose.position.z = markers.markers[i - 1].pose.position.z + pos.z();
            markers.markers[i].pose.orientation.w = quat.w();
            markers.markers[i].pose.orientation.x = quat.x();
            markers.markers[i].pose.orientation.y = quat.y();
            markers.markers[i].pose.orientation.z = quat.z();
            markers.markers[i].scale.x = arm_unit_radius;
            markers.markers[i].scale.y = arm_unit_radius;
            markers.markers[i].scale.z = 0.02;
            markers.markers[i].color.r = 1.0;
            markers.markers[i].color.g = 0.0;
            markers.markers[i].color.b = 0.0;
            markers.markers[i].color.a = 1.0;
        }
        markers_pub.publish(markers);
        ros::spinOnce(); // subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
    }

    return 0;
}