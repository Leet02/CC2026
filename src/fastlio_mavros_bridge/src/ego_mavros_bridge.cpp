#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h> 

class EgoMavrosBridge
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pos_cmd_sub_;
    ros::Publisher mavros_pub_;

    geometry_msgs::PoseStamped current_setpoint_;
    bool received_cmd_;

public:
    EgoMavrosBridge()
    {
        received_cmd_ = false;

        pos_cmd_sub_ = nh_.subscribe("/drone_0_planning/pos_cmd", 10, &EgoMavrosBridge::posCmdCallback, this);
        mavros_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        ROS_INFO("=== Ego MAVROS Bridge Start ===");
    }

    void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
    {
        current_setpoint_.header.stamp = ros::Time::now();
        current_setpoint_.header.frame_id = "map";   

        current_setpoint_.pose.position.x = msg->position.x;
        current_setpoint_.pose.position.y = msg->position.y;
        current_setpoint_.pose.position.z = msg->position.z;

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->yaw);
        current_setpoint_.pose.orientation.x = q.x();
        current_setpoint_.pose.orientation.y = q.y();
        current_setpoint_.pose.orientation.z = q.z();
        current_setpoint_.pose.orientation.w = q.w();

        received_cmd_ = true;

        ROS_INFO_THROTTLE(1.0, "[Bridge] get pos_cmd → x=%.2f y=%.2f z=%.2f yaw=%.2f", 
                          msg->position.x, msg->position.y, msg->position.z, msg->yaw);
    }

    void run()
    {
        ros::Rate rate(50);
        while (ros::ok())
        {
            if (received_cmd_)
            {
                mavros_pub_.publish(current_setpoint_);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_mavros_bridge");
    EgoMavrosBridge bridge;
    bridge.run();
    return 0;
}