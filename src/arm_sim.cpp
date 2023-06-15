/**
 * @file arm_sim.cpp
 * @brief arm sim
 * @author Shunya Hara
 * @date 2023.6.13
 * @details
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
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
std::vector<float> angles;

ros::Time prev_time_angle;
void angularCallback(const std_msgs::Float32::ConstPtr &msg)
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

void angleCallback(const std_msgs::Float32::ConstPtr &msg)
{

    angle = msg->data;
    
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

void anglesCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() != arm_num)
    {
        ROS_ERROR("msg->data.size() != arm_num");
        return;
    }
    angles = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_sim_node");
    ros::NodeHandle n;
    // 制御周期10Hz
    ros::Rate loop_rate(100);

    // param setting
    ros::NodeHandle pn("~");
    arm_num = pn.param<int>("arm_num", 100);
    arm_unit_length = pn.param<double>("arm_unit_length", 0.013);
    vendor_num = pn.param<int>("vendor_num", 2);
    const double arm_unit_radius = pn.param<double>("arm_unit_radius", 0.02);
    linear_pos = arm_unit_length * (vendor_num);
    prev_time = ros::Time::now();
    // Publisher
    ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("arm_sim_markers", 1);
    ros::Publisher debug_markers_pub = n.advertise<visualization_msgs::MarkerArray>("arm_sim_debug_markers", 1);
    ros::Publisher now_angle_pub = n.advertise<std_msgs::Float32>("now_angle", 1);
    ros::Publisher now_linear_pos_pub = n.advertise<std_msgs::Float32>("now_linear_pos", 1);
    // subscriber
    ros::Subscriber angle_sub = n.subscribe("angle", 1, angleCallback);
    ros::Subscriber angular_sub = n.subscribe("angular_vel", 1, angularCallback);
    ros::Subscriber linear_sub = n.subscribe("linear_vel", 1, linearVelCallback);
    ros::Subscriber angles_sub = n.subscribe("angles", 1, anglesCallback);
    angles.resize(arm_num);

    visualization_msgs::MarkerArray markers;
    markers.markers.resize(arm_num);
    visualization_msgs::MarkerArray debug_markers;
    debug_markers.markers.resize(1);


    while (n.ok())
    {

        double angle_sum = 0.0;
        markers.markers[0].header.frame_id = "map";
        markers.markers[0].header.stamp = ros::Time::now();
        markers.markers[0].ns = "arm";
        markers.markers[0].id = 0;
        markers.markers[0].type = visualization_msgs::Marker::CYLINDER;
        markers.markers[0].action = visualization_msgs::Marker::ADD;
        markers.markers[0].pose.position.x = linear_pos - (arm_num - vendor_num) * arm_unit_length;
        markers.markers[0].pose.position.y = 0.0;
        markers.markers[0].pose.position.z = 0.0;
        markers.markers[0].pose.orientation.w = 1.0;
        markers.markers[0].pose.orientation.x = 0.0;
        markers.markers[0].pose.orientation.y = 0.0;
        markers.markers[0].pose.orientation.z = 0.0;
        markers.markers[0].scale.x = arm_unit_radius;
        markers.markers[0].scale.y = arm_unit_radius;
        markers.markers[0].scale.z = 0.01;
        markers.markers[0].color.r = 1.0;
        markers.markers[0].color.g = 0.0;
        markers.markers[0].color.b = 0.0;
        markers.markers[0].color.a = 1.0;
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
            markers.markers[i].scale.z = 0.01;
            markers.markers[i].color.r = 1.0;
            markers.markers[i].color.g = 0.0;
            markers.markers[i].color.b = 0.0;
            markers.markers[i].color.a = 1.0;
        }

        {
        int i=0;
        debug_markers.markers[i].header.frame_id = "map";
        debug_markers.markers[i].header.stamp = ros::Time::now();
        debug_markers.markers[i].ns = "debug";
        debug_markers.markers[i].id = i;
        debug_markers.markers[i].type = visualization_msgs::Marker::ARROW;
        debug_markers.markers[i].action = visualization_msgs::Marker::ADD;
        angle_sum += angles.at(i);
        Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
        debug_markers.markers[i].pose.position.x = arm_unit_length;
        debug_markers.markers[i].pose.position.y = 0.0;
        debug_markers.markers[i].pose.position.z = arm_unit_radius;
        debug_markers.markers[i].pose.orientation.w = quat.w();
        debug_markers.markers[i].pose.orientation.x = quat.x();
        debug_markers.markers[i].pose.orientation.y = quat.y();
        debug_markers.markers[i].pose.orientation.z = quat.z();
        debug_markers.markers[i].scale.x = arm_unit_radius*3.0;
        debug_markers.markers[i].scale.y = arm_unit_radius*0.5;
        debug_markers.markers[i].scale.z = arm_unit_radius*0.5;
        debug_markers.markers[i].color.r = 0.0;
        debug_markers.markers[i].color.g = 1.0;
        debug_markers.markers[i].color.b = 0.0;
        debug_markers.markers[i].color.a = 1.0;
        }
        debug_markers_pub.publish(debug_markers);
        markers_pub.publish(markers);

        std_msgs::Float32 now_angle_msg;
        now_angle_msg.data = angle;
        now_angle_pub.publish(now_angle_msg);

        std_msgs::Float32 now_linear_pos_msg;
        now_linear_pos_msg.data = linear_pos;
        now_linear_pos_pub.publish(now_linear_pos_msg);
        
        ros::spinOnce(); // subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
    }

    return 0;
}