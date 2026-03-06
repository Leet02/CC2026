#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher vision_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped vision;

    vision.header = msg->header;
    vision.pose = msg->pose.pose;

    vision_pub.publish(vision);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastlio_mavros_bridge");
    ros::NodeHandle nh;

    ros::Subscriber sub =
        nh.subscribe("/fastlio_odom", 10, odomCallback);

    vision_pub =
        nh.advertise<geometry_msgs::PoseStamped>(
            "/mavros/vision_pose/pose", 10);

    ros::spin();
}
