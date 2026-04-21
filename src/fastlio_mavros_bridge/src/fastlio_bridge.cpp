/*#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher vision_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped vision;

    vision.header.stamp = msg->header.stamp;
    vision.header.frame_id = "map";

    // ENU -> NED
    vision.pose.position.x = msg->pose.pose.position.y;
    vision.pose.position.y = msg->pose.pose.position.x;
    vision.pose.position.z = msg->pose.pose.position.z;

    // quaternion
    vision.pose.orientation = msg->pose.pose.orientation;

    vision_pub.publish(vision);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastlio_mavros_bridge");
    ros::NodeHandle nh;

    ros::Subscriber sub =
        nh.subscribe("/fastlio_odom", 50, odomCallback);

    vision_pub =
        nh.advertise<geometry_msgs::PoseStamped>(
            "/mavros/vision_pose/pose", 50);

    ros::spin();
}*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher vision_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped vision;

    vision.header.stamp = msg->header.stamp;
    vision.header.frame_id = "map";  // ENU frame from FAST-LIO2

    // Copy directly (ENU); MAVROS will convert to NED internally
    vision.pose.position.x = msg->pose.pose.position.x;
    vision.pose.position.y = msg->pose.pose.position.y;
    vision.pose.position.z = msg->pose.pose.position.z;

    // Orientation direct
    vision.pose.orientation = msg->pose.pose.orientation;

    vision_pub.publish(vision);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastlio_mavros_bridge");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/fastlio_odom", 50, odomCallback);
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 50);

    ros::spin();
    return 0;
}