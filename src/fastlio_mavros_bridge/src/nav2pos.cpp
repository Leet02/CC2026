#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class Nav2Pos
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher vision_pub_;

public:
    Nav2Pos()
    {
        odom_sub_ = nh_.subscribe("/fastlio_odom", 50, &Nav2Pos::odomCallback, this);

        vision_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/vision_pose/pose", 50);

        ROS_INFO("nav2pos started. Publishing to /mavros/vision_pose/pose");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose;

        pose.header = msg->header;
        pose.header.frame_id = msg->header.frame_id;

        pose.pose = msg->pose.pose;

        vision_pub_.publish(pose);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav2pos");

    Nav2Pos node;

    ros::spin();

    return 0;
}