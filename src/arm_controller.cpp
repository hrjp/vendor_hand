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
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


// global parameter
double arm_unit_length;
int arm_num;
int vendor_num;

// global variable
std::vector<Eigen::Vector3d> target_points;
std::vector<float> target_angles;
ros::Publisher angles_pub;

void posesCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    target_points.clear();
    for(const auto &pose : msg->poses)
    {
        Eigen::Vector3d point(pose.position.x, pose.position.y, pose.position.z);
        target_points.push_back(point);
    }
    target_angles.clear();
    target_angles.resize(arm_num, 0.0);
    //reverse target_points
    std::reverse(target_points.begin(), target_points.end());
    int index = 1;
    auto now_point = target_points.at(0);
    const auto init_direction = target_points.at(index) - now_point;
    const auto offset_angle = std::atan2(init_direction.y(), init_direction.x());
    for(auto && angle : target_angles){
        const auto direction = target_points.at(index) - now_point;
        angle = std::atan2(direction.y(), direction.x())-offset_angle;
        now_point += arm_unit_length * direction.normalized();
        if((target_points.at(index)-now_point).norm() < arm_unit_length){
            index++;
        }
        if(index >= target_points.size()){
            break;
        }
    }
    std::reverse(target_angles.begin(), target_angles.end());
    std_msgs::Float32MultiArray pub_target_angles;
    pub_target_angles.data = target_angles;
    angles_pub.publish(pub_target_angles);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle n;
    // param setting
    ros::NodeHandle pn("~");
    arm_num = pn.param<int>("arm_num", 100);
    arm_unit_length = pn.param<double>("arm_unit_length", 0.013);
    vendor_num = pn.param<int>("vendor_num", 2);
    const double arm_unit_radius = pn.param<double>("arm_unit_radius", 0.02);
    ros::Rate loop_rate(10);

    // Publisher
    angles_pub = n.advertise<std_msgs::Float32MultiArray>("angles", 1);
    //subscriber
    ros::Subscriber poses_sub = n.subscribe("target_poses", 1, posesCallback);

    while (n.ok())
    {

        
        ros::spinOnce();    
        loop_rate.sleep();

    }
    return 0;
}