/**
 * @file apriltag_transform.cpp
 * @brief
 * @author Shunya Hara
 * @date 2023.12.20
 * @details
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>

int base_tag_id = 0;
int first_tag_id = 0;
int end_tag_id = 0;
std::string base_frame_id = "base_link";
double arm_unit_radius = 0.02;
bool inverse_base_tag = false;

ros::Publisher markers_pub;
ros::Publisher diff_markers_pub;

geometry_msgs::PoseArray poses_msg;

void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (msg->detections.size() == 0)
    {
        return;
    }
    // base_tag_idの位置を探す
    int base_tag_index = -1;
    geometry_msgs::Pose base_tag_pose;
    for (int i = 0; i < msg->detections.size(); i++)
    {
        if (msg->detections[i].id[0] == base_tag_id)
        {
            base_tag_index = i;
            base_tag_pose = msg->detections[i].pose.pose.pose;
            break;
        }
    }
    if (base_tag_index == -1)
    {
        return;
    }
    //base_tagをx軸周りに180度回転
    if(inverse_base_tag){
        tf::Transform base_tag_tf;
        tf::poseMsgToTF(base_tag_pose, base_tag_tf);
        tf::Transform inverse_base_tag_tf = base_tag_tf * tf::Transform(tf::Quaternion(0.0, M_PI, 0.0));
        tf::poseTFToMsg(inverse_base_tag_tf, base_tag_pose);
    }

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray diff_marker_array;
    for(const auto & tag : msg->detections){
        if(tag.id[0] == base_tag_id){
            continue;
        }
        if(tag.id[0]<first_tag_id || end_tag_id<tag.id[0]){
            continue;
        }
        
        geometry_msgs::Pose transformed_pose;
        // base_tag_poseを基準にtagの位置を変換
        tf::Transform base_tag_tf;
        tf::poseMsgToTF(base_tag_pose, base_tag_tf);
        tf::Transform tag_tf;
        tf::poseMsgToTF(tag.pose.pose.pose, tag_tf);
        tf::Transform transformed_tf = base_tag_tf.inverse() * tag_tf;
        tf::poseTFToMsg(transformed_tf, transformed_pose);
        visualization_msgs::Marker marker;
        marker.header.frame_id = base_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = std::string("apriltag_")+base_frame_id;
        marker.id = tag.id[0];
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = transformed_pose.position.x;
        marker.pose.position.y = transformed_pose.position.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.scale.x = arm_unit_radius;
        marker.scale.y = arm_unit_radius;
        marker.scale.z = 0.01;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.emplace_back(marker);

        if(poses_msg.poses.size()<=tag.id[0]-first_tag_id){
            continue;
        }
        visualization_msgs::Marker diff_marker;
        diff_marker.header.frame_id = base_frame_id;
        diff_marker.header.stamp = ros::Time::now();
        diff_marker.ns = std::string("apriltag_diff_")+base_frame_id;
        diff_marker.id = tag.id[0];
        diff_marker.type = visualization_msgs::Marker::LINE_STRIP;
        diff_marker.action = visualization_msgs::Marker::ADD;
        diff_marker.points.emplace_back(marker.pose.position);
        diff_marker.points.emplace_back(poses_msg.poses.at(tag.id[0]-first_tag_id).position);
        diff_marker.scale.x = 0.001;
        diff_marker.color.r = 0.0;
        diff_marker.color.g = 0.0;
        diff_marker.color.b = 1.0;
        diff_marker.color.a = 1.0;
        diff_marker_array.markers.emplace_back(diff_marker);

    }
    markers_pub.publish(marker_array);
    diff_markers_pub.publish(diff_marker_array);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_transform_node");
    ros::NodeHandle n;
    // param setting
    ros::NodeHandle pn("~");
    base_tag_id = pn.param<int>("base_tag_id", 0);
    first_tag_id = pn.param<int>("first_tag_id", 0);
    end_tag_id = pn.param<int>("end_tag_id", 0);
    base_frame_id = pn.param<std::string>("base_frame_id", "base_link");
    arm_unit_radius = pn.param<double>("arm_unit_radius", 0.02);
    inverse_base_tag = pn.param<bool>("inverse_base_tag", false);
    ros::Rate loop_rate(10);

    // Publisher
    markers_pub = n.advertise<visualization_msgs::MarkerArray>("apriltag_markers", 1);
    diff_markers_pub = n.advertise<visualization_msgs::MarkerArray>("apriltag_diff_markers", 1);
    // subscriber
    ros::Subscriber joy_sub = n.subscribe("/tag_detections", 1, tagCallback);
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseArray>("poses", 1, [&](const geometry_msgs::PoseArray::ConstPtr &msg) { poses_msg = *msg; });
    while (n.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}