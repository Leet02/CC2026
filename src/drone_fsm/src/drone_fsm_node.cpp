#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/tf.h>
#include <cmath>

enum State { IDLE, ARM, TAKEOFF, HOVER, MISSION };

class DroneFSM {
private:
  ros::NodeHandle nh;
  ros::Subscriber state_sub, pose_sub, cmd_sub;
  ros::Publisher setpoint_pub;   // 仅用于起飞
  ros::Publisher cmd_pub;        // 持续转发给 px4ctrl 的 /cmd
  ros::ServiceClient arm_client, mode_client;

  mavros_msgs::State current_state;
  geometry_msgs::PoseStamped current_pose, target_pose;
  geometry_msgs::PoseStamped ego_goal;

  quadrotor_msgs::PositionCommand latest_cmd;  // 缓存最新规划指令
  bool has_ego_cmd = false;
  ros::Time last_cmd_time;

  State fsm_state = IDLE;
  ros::Rate rate{60};
  ros::Time last_req;

  void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
  }

  void cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    latest_cmd = *msg;
    latest_cmd.header.stamp = ros::Time::now();

    ego_goal.pose.position.x = msg->position.x;
    ego_goal.pose.position.y = msg->position.y;
    ego_goal.pose.position.z = msg->position.z;

    has_ego_cmd = true;
    last_cmd_time = ros::Time::now();

    ROS_INFO("Received ego cmd, goal: x=%.2f y=%.2f z=%.2f",
             msg->position.x, msg->position.y, msg->position.z);
  }

  bool switch_to_offboard() {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
      ROS_INFO("OFFBOARD enabled");
      return true;
    }
    return false;
  }

  bool arm_drone() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arm_client.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("Drone armed");
      return true;
    }
    return false;
  }

  double distance_to_goal() {
    double dx = current_pose.pose.position.x - ego_goal.pose.position.x;
    double dy = current_pose.pose.position.y - ego_goal.pose.position.y;
    double dz = current_pose.pose.position.z - ego_goal.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void publish_takeoff_setpoint() {
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "map";
    setpoint_pub.publish(target_pose);
  }

  void publish_cmd_to_px4ctrl() {
    if (!has_ego_cmd) return;

    latest_cmd.header.stamp = ros::Time::now();

    if (latest_cmd.header.frame_id.empty()) {
      latest_cmd.header.frame_id = "world";
    }

    cmd_pub.publish(latest_cmd);
  }

public:
  DroneFSM() {
    state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, &DroneFSM::state_cb, this);

    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10, &DroneFSM::pose_cb, this);

    cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "/drone_0_planning/pos_cmd", 10, &DroneFSM::cmd_cb, this);

    setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>(
        "/cmd", 20);   // px4ctrl 默认订阅这个话题

    arm_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");

    mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    last_req = ros::Time::now();
    last_cmd_time = ros::Time::now();

    target_pose.header.frame_id = "map";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 1.0;

    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();
  }

  void run() {
    while (ros::ok()) {
      // 只在 TAKEOFF 阶段持续发位置 setpoint
      if (fsm_state == TAKEOFF) {
        publish_takeoff_setpoint();
      }

      // 在 MISSION 阶段持续转发规划器指令到 /cmd
      if (fsm_state == MISSION) {
        publish_cmd_to_px4ctrl();
      }

      switch (fsm_state) {
        case IDLE:
          if (current_state.connected) {
            ROS_INFO("Connected, ready to send initial setpoints");

            for (int i = 0; i < 30; ++i) {
              target_pose.header.stamp = ros::Time::now();
              target_pose.pose.position.x = current_pose.pose.position.x;
              target_pose.pose.position.y = current_pose.pose.position.y;
              target_pose.pose.position.z = 1.0;

              tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
              target_pose.pose.orientation.x = q.x();
              target_pose.pose.orientation.y = q.y();
              target_pose.pose.orientation.z = q.z();
              target_pose.pose.orientation.w = q.w();

              setpoint_pub.publish(target_pose);
              ros::spinOnce();
              rate.sleep();
            }

            fsm_state = ARM;
          }
          break;

        case ARM:
          if ((ros::Time::now() - last_req) > ros::Duration(1.0)) {
            if (!current_state.armed) {
              if (arm_drone()) {
                fsm_state = TAKEOFF;
              }
            } else {
              fsm_state = TAKEOFF;
            }
            last_req = ros::Time::now();
          }
          break;

        case TAKEOFF: {
          if ((ros::Time::now() - last_req) > ros::Duration(1.0)) {
            if (current_state.mode != "OFFBOARD") {
              switch_to_offboard();
            }
            last_req = ros::Time::now();
          }

          double dz = std::fabs(current_pose.pose.position.z - 1.0);
          ROS_INFO_THROTTLE(1.0,
                            "TAKEOFF: x=%.2f y=%.2f z=%.2f",
                            current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z);

          if (dz < 0.1) {
            ROS_INFO("Reached 1.0m, entering HOVER");
            fsm_state = HOVER;
          }
          break;
        }

        case HOVER: {
          ROS_INFO_THROTTLE(5.0, "Hovering, waiting for ego command");

          if (current_pose.pose.position.z < 0.5) {
            target_pose.pose.position.z = 0.5;
            ROS_WARN_THROTTLE(1.0, "Altitude below 0.5m");
          }

          if (has_ego_cmd) {
            fsm_state = MISSION;
            ROS_INFO("Received ego command, entering MISSION");
          }
          break;
        }

        case MISSION: {
          double dist = distance_to_goal();

          ROS_INFO_THROTTLE(1.0,
                            "MISSION: dist_to_goal=%.3f, current=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f)",
                            dist,
                            current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z,
                            ego_goal.pose.position.x,
                            ego_goal.pose.position.y,
                            ego_goal.pose.position.z);

          // 如果很久没有收到新指令，或者到达目标，回到 HOVER
          if (dist < 0.15) {
            ROS_INFO("Goal reached, back to HOVER");
            has_ego_cmd = false;
            fsm_state = HOVER;
          } else if ((ros::Time::now() - last_cmd_time) > ros::Duration(1.0)) {
            ROS_WARN("Command timeout, back to HOVER");
            has_ego_cmd = false;
            fsm_state = HOVER;
          }
          break;
        }
      }

      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "drone_fsm_node");
  DroneFSM fsm;
  fsm.run();
  return 0;
}