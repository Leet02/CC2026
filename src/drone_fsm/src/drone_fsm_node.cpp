#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Dense>
#include <vector>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

class PX4MissionFSM
{
private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub, odom_sub, gate_sub, aruco_num_sub, balloon_sub;
    ros::Subscriber diff_raw_sub; // 监听DiffPlanner底层心跳用
    ros::Publisher goal_pub, px4_pos_pub;
    ros::ServiceClient set_mode_client, arming_client;

    ros::Publisher override_pub;

    enum FSMState {
        INIT_TAKEOFF,   // 解锁并直接起飞至1米
        GO_FIRST,       // 交给Planner前往第一个悬停点
        WAIT_GATE,      // 接收入口框目标
        GO_GATE_A,      // 交给Planner前往A点
        GO_STEP4,       // 交给Planner前往固定点
        WAIT_ARUCO,     // 接收Aruco目标
        RETURN_A,       // 交给Planner返回A点
        GO_STEP7,       // 交给Planner前往备战点
        WAIT_BALLOON,   // 找寻气球目标
        CRASH_BALLOON   // 底层直连防撞
    } state;

    mavros_msgs::State current_state;
    Eigen::Vector3d current_pos;
    double current_yaw;

    std::vector<Eigen::Vector3d> gate_buffer, aruco_buffer, balloon_buffer;
    Eigen::Vector3d point_A, aruco_target, balloon_target;

    bool goal_sent;
    bool aruco_received; 
    ros::Time goal_send_time;

    // 心跳保活
    ros::Time last_diff_time;      // 记录DiffPlanner上次向底层发点的时间
    Eigen::Vector3d backup_hover_pos; // 兜底用的悬停位置
    double backup_hover_yaw;          // 兜底用的偏航角

    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pos = Eigen::Vector3d(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
        
        // 提取Yaw角供接管悬停时平滑使用
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw);
    }

    // 听DiffPlanner对底层的控制(bridge节点通常发往 setpoint_raw)
    void diff_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
        last_diff_time = ros::Time::now();
    }

    void gate_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (state != WAIT_GATE) return;
        Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        if (p.x() >= 0.4 && p.x() <= 4.8 && p.y() >= -2.3 && p.y() <= 1.1 && p.z() <= 2.0) {
            gate_buffer.push_back(p);
        }
    }

    void aruco_num_cb(const std_msgs::Int32::ConstPtr& msg) {
        if (state != WAIT_ARUCO) return;
        
        // 只要处在 WAIT_ARUCO 状态，一收到话题就置为 true
        ROS_INFO("[MISSION] Received Aruco Num: %d", msg->data);
        aruco_received = true;
    }

    void balloon_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (state != WAIT_BALLOON) return;
        Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        if (p.x() >= -0.4 && p.x() <= 0.4 && p.y() >= -3.1 && p.y() <= 1.9) {
            balloon_buffer.push_back(p);
        }
    }

    void publishPlannerGoalWithRetry(Eigen::Vector3d pos, double yaw) {
        ros::Time now = ros::Time::now();
        if (!goal_sent || (now - goal_send_time).toSec() > 4.0) {
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = now;
            goal.header.frame_id = "world";
            goal.pose.position.x = pos.x(); goal.pose.position.y = pos.y(); goal.pose.position.z = pos.z();
            
            tf::Quaternion q; q.setRPY(0, 0, yaw);
            goal.pose.orientation.x = q.x(); goal.pose.orientation.y = q.y(); goal.pose.orientation.z = q.z(); goal.pose.orientation.w = q.w();
            
            goal_pub.publish(goal);
            ROS_INFO("[MISSION] => Planner Goal Sent: [%.2f, %.2f, %.2f] yaw: %.2f", pos.x(), pos.y(), pos.z(), yaw);
            
            goal_sent = true;
            goal_send_time = now;
        }
    }

    // 给底层的最原始定点驱动
    void publishDirectPX4(Eigen::Vector3d pos, double yaw) {
        geometry_msgs::PoseStamped pt;
        pt.header.stamp = ros::Time::now();
        pt.header.frame_id = "world";
        pt.pose.position.x = pos.x(); pt.pose.position.y = pos.y(); pt.pose.position.z = pos.z();
        
        tf::Quaternion q; q.setRPY(0, 0, yaw);
        pt.pose.orientation.x = q.x(); pt.pose.orientation.y = q.y(); pt.pose.orientation.z = q.z(); pt.pose.orientation.w = q.w();
        
        px4_pos_pub.publish(pt);
    }

public:
    PX4MissionFSM() : nh("~"), state(INIT_TAKEOFF), goal_sent(false), current_yaw(0.0) {
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &PX4MissionFSM::state_cb, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/fastlio_odom", 10, &PX4MissionFSM::odom_cb, this);

    // 听DiffPlanner通过桥接器发往飞控的raw指令，以便心跳监控
        diff_raw_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10, &PX4MissionFSM::diff_cb, this);
        
        gate_sub = nh.subscribe<geometry_msgs::PoseStamped>("/gate/pose_world", 10, &PX4MissionFSM::gate_cb, this);
        aruco_num_sub = nh.subscribe<std_msgs::Int32>("/aruco/num", 10, &PX4MissionFSM::aruco_num_cb, this);
        balloon_sub = nh.subscribe<geometry_msgs::PoseStamped>("/balloon/pose_world", 10, &PX4MissionFSM::balloon_cb, this);
        
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        px4_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        override_pub = nh.advertise<std_msgs::Bool>("/fsm/force_override", 10);

        last_diff_time = ros::Time(0);
        backup_hover_pos = Eigen::Vector3d(0, 0, 1.0);
        backup_hover_yaw = 0.0;
    }

    void run() {
            ros::Rate rate(20.0);
            
            ROS_INFO("[MISSION] Waiting for FCU connection...");
            while(ros::ok() && !current_state.connected){
                ros::spinOnce();
                rate.sleep();
            }

            ROS_INFO("[MISSION] Pre-filling 100 setpoints for OFFBOARD safety...");
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = 0; pose.pose.position.y = 0; pose.pose.position.z = 1.0;
            for(int i = 100; ros::ok() && i > 0; --i){
                px4_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }

            mavros_msgs::SetMode offb_set_mode; offb_set_mode.request.custom_mode = "OFFBOARD";
            mavros_msgs::CommandBool arm_cmd; arm_cmd.request.value = true;
            
            ros::Time last_request = ros::Time::now();

            while (ros::ok()) {
                ros::spinOnce();

                //  心跳接管
                // 判断距上次 Diffplanner 给飞控发指令有没有超过 0.15 秒
                bool planner_active = (ros::Time::now() - last_diff_time).toSec() < 0.15;

                if (!planner_active) {
                    // 如果DiffPlanner在休息，飞控马上就会因断联退出Offboard。
                    // 此时系统越级以20Hz接管强锁位置
                    if (state == CRASH_BALLOON) {
                        publishDirectPX4(balloon_target, M_PI); // 最终撞击最高优先级发点
                    } else if (state == INIT_TAKEOFF) {
                        publishDirectPX4(Eigen::Vector3d(0, 0, 1.0), 0.0); // 起飞前强锁1米高度
                    } else {
                        publishDirectPX4(backup_hover_pos, backup_hover_yaw); // 将飞机牢牢定死在当前点不再漂变
                    }
                } else {
                    // 如果Diffplanner现在正控制飞机平滑移向目标，闭嘴，停止占用信道避免与Diffplanner冲突
                    // 但是要不断记录飞机此刻的位置到 backup_hover_pos
                    // 如果规划器下一秒规划中断/或者到达终点停止发迹点了，能接力让飞机在此刻无缝原地悬停
                    backup_hover_pos = current_pos;
                    backup_hover_yaw = current_yaw;
                }

                
                switch(state) {
                    case INIT_TAKEOFF: {
                        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) ROS_INFO("Offboard enabled");
                            last_request = ros::Time::now();
                        } else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                            if( arming_client.call(arm_cmd) && arm_cmd.response.success) ROS_INFO("Vehicle armed!");
                            last_request = ros::Time::now();
                        }

                        if (current_state.mode == "OFFBOARD" && current_state.armed) {
                            if ((current_pos - Eigen::Vector3d(0, 0, 1.0)).norm() < 0.2) {
                                ROS_WARN("[MISSION] Takeoff Reached! Requesting Planner for STEP 1.");
                                state = GO_FIRST;
                                goal_sent = false; 
                            }
                        }
                        break;
                    }
                    
                    case GO_FIRST: {
                        Eigen::Vector3d p1(1.5, -0.65, 1.6);
                        publishPlannerGoalWithRetry(p1, 0.0); 
                        if ((current_pos - p1).norm() < 0.2) {
                            ROS_INFO("[MISSION] Arrived First Point. Waiting GATE.");
                            state = WAIT_GATE;
                            goal_sent = false; 
                            gate_buffer.clear();
                        }
                        break;
                    }

                    case WAIT_GATE: {
                        if (gate_buffer.size() >= 5) {
                            Eigen::Vector3d avg(0, 0, 0);
                            for (auto& p : gate_buffer) avg += p;
                            point_A = avg / gate_buffer.size();
                            ROS_INFO("[MISSION] Locked Point A. Approaching...");
                            state = GO_GATE_A;
                        }
                        break;
                    }

                    case GO_GATE_A: {
                        publishPlannerGoalWithRetry(point_A, 0.0); 
                        if ((current_pos - point_A).norm() < 0.2) {
                            ROS_INFO("[MISSION] Arrived Point A. Proceeding to Go_STEP4.");
                            state = GO_STEP4;
                            goal_sent = false;
                        }
                        break;
                    }

                    case GO_STEP4: {
                        Eigen::Vector3d p4(4.0, -0.1, 1.0);
                        publishPlannerGoalWithRetry(p4, 0.0);
                        if ((current_pos - p4).norm() < 0.2) {
                            ROS_INFO("[MISSION] Arrived Step 4. Waiting ARUCO.");
                            state = WAIT_ARUCO;
                            goal_sent = false;
                            aruco_received = false;
                        }
                        break;
                    }

                    case WAIT_ARUCO: {
                        // 这个状态下，心跳守卫层会自动接管维持悬停点 (p4)
                        if (aruco_received) {
                            ROS_INFO("[MISSION] ARUCO Triggered! Skipping GO_ARUCO, returning directly to Point A...");
                            state = RETURN_A;
                            goal_sent = false;
                        }
                        break;
                    }


                    case RETURN_A: {
                        publishPlannerGoalWithRetry(point_A, 0.0);
                        if ((current_pos - point_A).norm() < 0.2) {
                            ROS_INFO("[MISSION] Returned to Point A. Moving to Strike Prep...");
                            state = GO_STEP7;
                            goal_sent = false;
                        }
                        break;
                    }

                    case GO_STEP7: {
                        Eigen::Vector3d p7(1.5, -0.8, 0.8);
                        publishPlannerGoalWithRetry(p7, M_PI); 
                        if ((current_pos - p7).norm() < 0.2) {
                            ROS_INFO("[MISSION] Arrived Strike Prep. Waiting BALLOON.");
                            state = WAIT_BALLOON;
                            goal_sent = false;
                            balloon_buffer.clear();
                        }
                        break;
                    }

                    case WAIT_BALLOON: {
                        if (balloon_buffer.size() >= 5) {
                            Eigen::Vector3d avg(0, 0, 0);
                            for (auto& p : balloon_buffer) avg += p;
                            balloon_target = avg / balloon_buffer.size();
                            ROS_WARN("[MISSION] BALLOON LOCATED! => FULL SPEED CRASH!");
                            state = CRASH_BALLOON;
                        }
                        break;
                    }

                    case CRASH_BALLOON: {
                        // 主动关闭规划器输出隧道
                        std_msgs::Bool msg;
                        msg.data = true;
                        override_pub.publish(msg);

                        // 持续以 20Hz 直接通过 FSM 向飞控发点，无视避障
                        publishDirectPX4(balloon_target, M_PI);
                        break;
                    }
                }

                rate.sleep();
            }
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "px4_mission_node");
    PX4MissionFSM node;
    node.run();
    return 0;
}