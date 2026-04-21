#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Bool.h>

class DiffMavrosBridge
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber pos_cmd_sub_;
  ros::Publisher raw_pub_;
  ros::Subscriber override_sub;
  bool force_override = false;

public:
  DiffMavrosBridge()
  {
    pos_cmd_sub_ = nh_.subscribe("/drone_0_planning/pos_cmd", 50,
                                 &DiffMavrosBridge::cb, this);

    raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 50);

    override_sub = nh_.subscribe("/fsm/force_override", 10, &DiffMavrosBridge::overrideCb, this);
  }

  void overrideCb(const std_msgs::Bool::ConstPtr& msg) {
    force_override = msg->data;
  }

  void cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
  {
    if (force_override) return;

    mavros_msgs::PositionTarget sp;

    sp.header.stamp = ros::Time::now();
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 不做任何坐标转换（MAVROS内部处理）
    sp.position = msg->position;
    sp.velocity = msg->velocity;

    sp.acceleration_or_force.x = 0;
    sp.acceleration_or_force.y = 0;
    sp.acceleration_or_force.z = 0;

    sp.yaw = msg->yaw;

    // 禁止 yaw_rate
    sp.yaw_rate = 0;

    //只用 position + velocity + yaw
    sp.type_mask =
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    raw_pub_.publish(sp);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diff_bridge");
  DiffMavrosBridge b;
  ros::spin();
  return 0;
}