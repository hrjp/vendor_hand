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
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <optional>


// global parameter
double arm_unit_length;
int arm_num;
int vendor_num;
float max_unit_angle;

// global variable
std::vector<Eigen::Vector3d> target_points;
std::vector<float> target_angles;
ros::Publisher angles_pub;
ros::Publisher poses_pub;
int linear_num = 0;
double now_linear_pos = 0.0;

std::optional<Eigen::Vector3d> getIntarsectionPoint(const Eigen::Vector3d &p1, const double &r, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3)
{
    const double xd=p3.x()-p2.x();
    const double yd=p3.y()-p2.y();
    const double X=p2.x()-p1.x();
    const double Y=p2.y()-p1.y();
    const double a=xd*xd+yd*yd;
    const double b=xd*X+yd*Y;
    const double c=X*X+Y*Y-r*r;
    const double D=b*b-a*c;
    if(D<0){
        const Eigen::Vector3d sr=p1-p2;
        const Eigen::Vector3d se=p3-p2;
        const double p24=se.dot(sr)/se.norm();
        const Eigen::Vector3d p4=p2+se.normalized()*p24;
        return p1+(p4-p1).normalized()*r;
    }
    const double s1=(-b+std::sqrt(D))/a;
    const double s2=(-b-std::sqrt(D))/a;
    const double s=std::max(s1, s2);
    if(s>1.0){
        return std::nullopt;
    }
    return Eigen::Vector3d(p2.x()+xd*s, p2.y()+yd*s, p2.z());
}

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
    double angle_sum = -std::atan2(init_direction.y(), init_direction.x());
    linear_num = 0;
    geometry_msgs::PoseArray pub_arm_poses;
    pub_arm_poses.header.frame_id = "map";
    pub_arm_poses.header.stamp = ros::Time::now();
    for(auto && angle : target_angles){
        linear_num++;
        Eigen::Vector3d target_point;
        while(true){
            const auto next_point=getIntarsectionPoint(now_point,arm_unit_length,target_points.at(index-1),target_points.at(index));
            if(next_point){
                target_point=next_point.value();
                break;
            }
            index++;
            if(index >= target_points.size()){
                break;
            }
        }
        if(index >= target_points.size()){
            break;
        }
        const auto direction = target_point - now_point;
        angle = -std::atan2(direction.y(), direction.x())-angle_sum;
        if(angle > M_PI){
            angle -= 2*M_PI;
        }
        if(angle < -M_PI){
            angle += 2*M_PI;
        }
        angle=std::max(-max_unit_angle, std::min(max_unit_angle, angle));
        const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(-angle-angle_sum, Eigen::Vector3d::UnitZ()));
        angle_sum += angle;
        now_point += arm_unit_length * (quaternion * Eigen::Vector3d::UnitX());
        geometry_msgs::Pose pose;
        pose.position.x = now_point.x();
        pose.position.y = now_point.y();
        pose.position.z = now_point.z();
        pose.orientation.w = quaternion.w();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pub_arm_poses.poses.push_back(pose);
    }
    std_msgs::Float32MultiArray pub_target_angles;
    pub_target_angles.data = target_angles;
    //angles_pub.publish(pub_target_angles);
    poses_pub.publish(pub_arm_poses);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle n;
    // param setting
    ros::NodeHandle pn("~");
    arm_num = pn.param<int>("arm_num", 100);
    arm_unit_length = pn.param<double>("arm_unit_length", 0.013);
    vendor_num = pn.param<int>("vendor_num", 1);
    const double arm_unit_radius = pn.param<double>("arm_unit_radius", 0.02);
    const double linear_vel = pn.param<double>("linear_vel", 0.05);
    max_unit_angle = pn.param<float>("max_unit_angle_degree", 30)*M_PI/180.0;
    ros::Rate loop_rate(10);

    // Publisher
    angles_pub = n.advertise<std_msgs::Float32MultiArray>("angles", 1);
    ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("angle", 1);
    ros::Publisher linear_vel_pub = n.advertise<std_msgs::Float32>("linear_vel", 1);
    poses_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1);
    //subscriber
    ros::Subscriber poses_sub = n.subscribe("target_poses", 1, posesCallback);
    double now_angle = 0.0;
    ros::Subscriber angle_sub = n.subscribe<std_msgs::Float32>("now_angle", 1, 
        [&](const std_msgs::Float32::ConstPtr &msg){now_angle = msg->data;});
    ros::Subscriber linear_sub = n.subscribe<std_msgs::Float32>("now_linear_pos", 1, 
        [&](const std_msgs::Float32::ConstPtr &msg){now_linear_pos = msg->data;});
    bool is_start = false;
    ros::Subscriber start_sub = n.subscribe<std_msgs::Empty>("start", 1, 
        [&](const std_msgs::Empty::ConstPtr &msg){is_start=true;});

    const auto init_time = ros::Time::now();
    auto prev_time = ros::Time::now();
    while (n.ok())
    {
        const auto dt = (ros::Time::now() - prev_time).toSec();
        prev_time = ros::Time::now();
        //std::cout << "now_angle: " << now_angle << std::endl;
        //std::cout << "now_linear_pos: " << now_linear_pos << std::endl;
        //std::cout << "linear_num: " << linear_num << std::endl;
        if(is_start){
            std_msgs::Float32 angle_vel_msg;
            angle_vel_msg.data = target_angles.at(int(now_linear_pos/arm_unit_length))*double(vendor_num);
            angle_pub.publish(angle_vel_msg);

            std_msgs::Float32 linear_vel_msg;
            //std::cout << "now_linear_pos/arm_unit_length: " << now_linear_pos/arm_unit_length << std::endl;
            if(linear_num-vendor_num > int(now_linear_pos/arm_unit_length)){
                linear_vel_msg.data = linear_vel;
            }else{
                linear_vel_msg.data = 0.0;
            }
            linear_vel_pub.publish(linear_vel_msg);
        }
        
        ros::spinOnce();    
        loop_rate.sleep();

    }
    return 0;
}