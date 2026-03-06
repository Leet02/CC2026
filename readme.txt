


抱歉我说错了，关于tf树部分：如果启用mapping_mid360.launch文件中被注释掉的	<!-- 修改：统一TF树 
    <node pkg="tf" type="static_transform_publisher" name="map_to_lio" args="0 0 0 0 0 0 map lio_world 100"/>

    <node pkg="tf" type="static_transform_publisher" name="lio_to_base" args="0 0 0 0 0 0 lio_body base_link 100"/>

    <node pkg="tf" type="static_transform_publisher" name="base_to_frd" args="0 0 0 0 0 0 base_link base_link_frd 100"/>  MAVROS连到base_link -->以上部分，那么tf树会变为以下情况：
“--”代表链接
map有两个分支
map--map_ned(/mavros)
和map--lio_world(/map_to_lio)--lio_body--base_link--base_link_frd
odom--odom_ned(/mavros)

我现在把  <arg name="odom_topic" default="/fastlio_odom"/>  extrinsic_est_en:  true      imu_topic:  "/mavros/imu/data" #原本为/mavros/imu/data EKF2_AID_MASK=9，EKF2_EV_POS_Z=0，lidar_type: 2,EKF2_HGT_MODE改为使用气压计  这些你修改过的值改回了原值（即上述值）
如今再次测试的话，飞机可以起飞到一米高度（但是起飞过程中飞机会向xy随即方向飘，影响这个因素有哪些？仔细思考），不会出现qgc和gazebo真实高度不一致的情况。现在飞机用2D navi goal指点后会飞过去，但会越飞越低直到落到地上导致无法继续前进，而且gazebo里飞机已经到位了，而rviz中飞机位置却没变化，像是飞机不知道自己的当前位置一样。而如果开着offboard节点我怕他们发送的命令有冲突，EKF2_AID_MASK改成25后xy方向也不会乱飘了
lee@lee:~/craic2026$ rosrun fastlio_mavros_bridge ego_mavros_bridge 
和lee@lee:~/craic2026$ rosrun fastlio_mavros_bridge nav2pos
会不会冲突？
^Clee@lee:~$ rostopic echo /mavros/local_position/pose这个话题很对，ego也是接受这个话题的，但是rviz里不反映出飞机的移动，会不会是frame_id: "map"和ego不一样？但是rviz里飞机朝向显示的很准确
header: 
  seq: 1124
  stamp: 
    secs: 740
    nsecs: 252676051
  frame_id: "map"
pose: 
  position: 
    x: 0.05667458847165108
    y: -0.02735004760324955
    z: 0.7380059957504272
  orientation: 
    x: -0.005668823893883331
    y: -0.01077574106433566
    z: 0.17904213055094603
    w: -0.9837661548042423
---
header: 
  seq: 1125
  stamp: 
    secs: 740
    nsecs: 284679960
  frame_id: "map"
pose: 
  position: 
    x: 0.055389080196619034
    y: -0.027707170695066452
    z: 0.7372063398361206
  orientation: 
    x: -0.005290626063488159
    y: -0.010910663333858903
    z: 0.17833465355184208
    w: -0.9838952084543828
---
而当我^Clee@lee:~rostopic echo /mavros/vision_pose/pose（rviz里我选择的pose话题,此时飞机实际高度0.8米）
header: 
  seq: 853
  stamp: 
    secs: 717
    nsecs: 241000000
  frame_id: "lio_world"
pose: 
  position: 
    x: 0.012531489633995495
    y: 0.025149490109522766
    z: 0.023328888027034922
  orientation: 
    x: -0.008371581627127417
    y: -0.012218595944710484
    z: 0.18200083374829457
    w: 0.9831868179797313
---
header: 
  seq: 854
  stamp: 
    secs: 717
    nsecs: 287000000
  frame_id: "lio_world"
pose: 
  position: 
    x: 0.011971468531602265
    y: 0.026598613107263087
    z: 0.022896158167734482
  orientation: 
    x: -0.007987705084936361
    y: -0.011857588931303242
    z: 0.18159531773769513
    w: 0.9832694110608452
---
header: 
  seq: 855
  stamp: 
    secs: 717
    nsecs: 336000000
  frame_id: "lio_world"
pose: 
  position: 
    x: 0.01228768744649498
^Clee@lee:~rostopic echo /mavros/vision_pose/pose（飞机实际高度0.7米）
header: 
  seq: 853
  stamp: 
    secs: 717
    nsecs: 241000000
  frame_id: "lio_world"
pose: 
  position: 
    x: 0.012531489633995495
    y: 0.025149490109522766
    z: 0.023328888027034922
  orientation: 
    x: -0.008371581627127417
    y: -0.012218595944710484
    z: 0.18200083374829457
    w: 0.9831868179797313
---
header: 
  seq: 854
  stamp: 
    secs: 717
    nsecs: 287000000
  frame_id: "lio_world"
pose: 
  position: 
    x: 0.011971468531602265
    y: 0.026598613107263087
    z: 0.022896158167734482
  orientation: 
    x: -0.007987705084936361
    y: -0.011857588931303242
    z: 0.18159531773769513
    w: 0.9832694110608452
---
header: 
  seq: 855
  stamp: 
    secs: 717
    nsecs: 336000000
  frame_id: "lio_world"
pose: 
  position: 
    x: 0.01228768744649498
    y: 0.028137101759489257
    z: 0.023650394105517634
  orientation: 
    x: -0.008344851383165881
    y: -0.012314929769339667
    z: 0.1841197474828743
    w: 0.9827912415904995
---
header: 
  seq: 856
  stamp: 
    secs: 717
    nsecs: 386000000
  frame_id: "lio_world"
pose: 
  position: 
    x: 0.013343809345947578
    y: 0.0255869530507322
    z: 0.024438594844281517
  orientation: 
    x: -0.007895962358445674
    y: -0.012432276311962982
    z: 0.18528408215960893
    w: 0.9825746288106529

以下是现在部份文件情况：
<launch>
  <!-- 添加默认值，避免未定义arg错误 -->
  <arg name="map_size_x" default="20.0"/>
  <arg name="map_size_y" default="20.0"/>
  <arg name="map_size_z" default="5.0"/>
  <arg name="init_x" default="0.0"/>
  <arg name="init_y" default="0.0"/>
  <arg name="init_z" default="1.0"/>
  <arg name="target_x" default="5.0"/>
  <arg name="target_y" default="0.0"/>
  <arg name="target_z" default="1.0"/>

  <arg name="drone_id" default="0"/>
  <arg name="odom_topic" default="/fastlio_odom"/>

  <!-- number of moving objects -->
  <arg name="obj_num" default="0" />  <!-- 设0，禁用动态obj -->

  <!-- 全局remap，确保无前缀话题 -->
  <remap from="/drone_/fastlio_odom" to="/fastlio_odom" />
  <remap from="/drone_/mavros/local_position/pose" to="/mavros/local_position/pose" />
  <remap from="/drone_/depth/image_raw" to="/depth/image_raw" /> 

  <!-- main algorithm params -->
  <include file="$(find ego_planner)/launch/advanced_param.xml">

    <remap from="~traj_start_trigger" to="/traj_start_trigger" />  <!-- 让ego订阅全局trigger -->

    <arg name="drone_id" value="$(arg drone_id)"/>

    <arg name="map_size_x_" value="$(arg map_size_x)"/>
    <arg name="map_size_y_" value="$(arg map_size_y)"/>
    <arg name="map_size_z_" value="$(arg map_size_z)"/>
    <arg name="odometry_topic" value="$(arg odom_topic)"/>

    <arg name="obj_num_set" value="$(arg obj_num)" />

    <arg name="camera_pose_topic" value="/mavros/local_position/pose"/>
    <arg name="depth_topic" value="/depth/image_raw"/>

    <arg name="cloud_topic" value="/fastlio_pointcloud"/>

    <arg name="cx" value="321.04638671875"/>
    <arg name="cy" value="243.44969177246094"/>
    <arg name="fx" value="387.229248046875"/>
    <arg name="fy" value="387.229248046875"/>

    <arg name="max_vel" value="2.0" />
    <arg name="max_acc" value="3.0" />

    <arg name="planning_horizon" value="7.5" /> 

    <arg name="use_distinctive_trajs" value="true" />

    <arg name="flight_type" value="1" />
    
    <arg name="point_num" value="1" />

    <arg name="point0_x" value="0" />
    <arg name="point0_y" value="0" />
    <arg name="point0_z" value="1.0" />

    <arg name="point1_x" value="0.0" />
    <arg name="point1_y" value="15.0" />
    <arg name="point1_z" value="1.0" />

    <arg name="point2_x" value="15.0" />
    <arg name="point2_y" value="0.0" />
    <arg name="point2_z" value="1.0" />

    <arg name="point3_x" value="0.0" />
    <arg name="point3_y" value="-15.0" />
    <arg name="point3_z" value="1.0" />

    <arg name="point4_x" value="-15.0" />
    <arg name="point4_y" value="0.0" />
    <arg name="point4_z" value="1.0" />
    
  </include>

  <!-- trajectory server -->
  <node pkg="ego_planner" name="drone_$(arg drone_id)_traj_server" type="traj_server" output="screen">
    <remap from="position_cmd" to="drone_$(arg drone_id)_planning/pos_cmd"/>
    <remap from="~planning/bspline" to="drone_$(arg drone_id)_planning/bspline"/>

    <param name="traj_server/time_forward" value="1.0" type="double"/>
  </node>

  <!-- 启用waypoint_generator，支持2D Nav Goal -->
  <node pkg="waypoint_generator" name="waypoint_generator" type="waypoint_generator" output="screen">
    <remap from="~odom" to="$(arg odom_topic)"/>        
    <remap from="~goal" to="/move_base_simple/goal"/>
    <remap from="~traj_start_trigger" to="/traj_start_trigger" />
    <param name="waypoint_type" value="manual-lonely-waypoint"/>    
  </node>

  <!-- 移除simulator.xml include，避免模拟冲突（用真实PX4/Gazebo） -->
  <!-- <include file="$(find ego_planner)/launch/simulator.xml"> ... </include> -->

  <!-- 添加odom_visualization节点 -->
  <node pkg="odom_visualization" name="drone_$(arg drone_id)_odom_visualization" type="odom_visualization" output="screen">
    <remap from="~odom" to="$(arg odom_topic)" />  
    <param name="_frame_id" value="lio_world" />  
    <param name="color/r" value="1.0"/>
    <param name="color/g" value="0.0"/>
    <param name="color/b" value="0.0"/>
    <param name="color/a" value="1.0"/>
    <param name="covariance_scale" value="100.0"/>
    <param name="robot_scale" value="1.0"/>
    <param name="tf45" value="false"/>
  </node>

  <!-- 统一use_sim_time -->
  <param name="/use_sim_time" value="true" />

  <node name="ego_rviz" pkg="rviz" type="rviz" args="-d $(find ego_planner)/launch/mapping.rviz" required="true" />

  <!-- obj_generator（动态obj），注释掉（比赛静态） -->
  <!-- <node pkg="plan_env" name="obj_generator" type="obj_generator" output="screen">
    <param name="obj_generator/obj_num" value="$(arg obj_num)"/>    
    <param name="obj_generator/x_size" value="12.0"/>  
    <param name="obj_generator/y_size" value="12.0"/>  
    <param name="obj_generator/h_size" value="1.0"/>   
    <param name="obj_generator/vel" value="1.5"/>   
    <param name="obj_generator/yaw_dot" value="2.0"/>   
    <param name="obj_generator/acc_r1" value="1.0"/>   
    <param name="obj_generator/acc_r2" value="1.0"/>   
    <param name="obj_generator/acc_z" value="0.0"/>   
    <param name="obj_generator/scale1" value="0.5"/>   
    <param name="obj_generator/scale2" value="1.0"/>   
    <param name="obj_generator/interval" value="100.0"/>    
    <param name="obj_generator/input_type" value="1"/>   
  </node> -->

</launch>#include <ros/ros.h>
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

        ROS_INFO("=== Ego MAVROS Bridge 已启动（使用 lio_world） ===");
    }

    void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
    {
        current_setpoint_.header.stamp = ros::Time::now();
        current_setpoint_.header.frame_id = "lio_world";   // ← 关键修复（原来是 "map"）

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

        ROS_INFO_THROTTLE(1.0, "[Bridge] 收到 pos_cmd → x=%.2f y=%.2f z=%.2f yaw=%.2f", 
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
}#include <ros/ros.h>
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
        pose.header.frame_id = "map";

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
}#include <ros/ros.h>
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
<launch>
	<arg name="rviz" default="false" />
    <arg name="drone_id" default="1"/>
    <arg name="drone_type" default="x280"/>

	<group if="$(arg rviz)">
		<node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find fast_lio)/rviz_cfg/loam_livox.rviz" />
	</group>

	<!-- 读取 fast-lio 参数 ，原本用mid360.yaml-->
	<rosparam command="load" file="$(find global_interface)/config/fast_lio/mid360.yaml" />
	<param name="feature_extract_enable" type="bool" value="0"/>
	<param name="point_filter_num" type="int" value="3"/>
	<param name="max_iteration" type="int" value="3" />
	<param name="filter_size_surf" type="double" value="0.5" />
	<param name="filter_size_map" type="double" value="0.5" />
	<param name="cube_side_length" type="double" value="1000" />
	<param name="runtime_pos_log_enable" type="bool" value="0" />

	<!-- 启动 fast-lio 激光里程计 -->
	<node pkg="fast_lio" type="fastlio_mapping" name="fastlio" output="screen" >
		<remap from="/Odometry"   to="fastlio_odom"/>
		<remap from="/cloud_registered"   to="fastlio_pointcloud"/>
		<remap from="/fastlio_path"   to="fastlio_path"/>
		<remap from="/fastlio_color_pointcloud"   to="fastlio_color_pointcloud"/>
	</node>
	<!-- 修改：统一TF树 
    <node pkg="tf" type="static_transform_publisher" name="map_to_lio" args="0 0 0 0 0 0 map lio_world 100"/>

    <node pkg="tf" type="static_transform_publisher" name="lio_to_base" args="0 0 0 0 0 0 lio_body base_link 100"/>

    <node pkg="tf" type="static_transform_publisher" name="base_to_frd" args="0 0 0 0 0 0 base_link base_link_frd 100"/>  MAVROS连到base_link -->
</launch>common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data" #原本为/mavros/imu/data
    camera_topic: "hik_camera_1/image/"
    time_sync_en: false         # ONLY turn on when external time synchronization is really not possible
    time_offset_lidar_to_imu: 0.0 # Time offset between lidar and IMU calibrated by other algorithms, e.g. LI-Init (can be found in README).
                                  # This param will take effect no matter what time_sync_en is. So if the time offset is not known exactly, please set as 0.0

preprocess:
    lidar_type: 2                # 原为2.。。1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 
    scan_line: 4
    blind: 0.09                  # 这里是圆形区域过滤的半径的平方，例如 0.09 就代表过滤距离雷达 0.3 m 内的点云

color_mapping:
    K_camera: [1791.45312, 0.0, 748.30875,
               0.0, 1798.74170, 544.853954,
               0.0, 0.0, 1.0]
    D_camera: [-0.06037161, 0.04314707, 0.00133593, 0.0015451, 0.0]
    time_offset_lidar_to_camera: 0.0                # ms

    extrinsic_T: [  -0.107012, 0.021146,  -0.143666]
    extrinsic_R: [ 0.03464462, -0.12268504, -0.99184055,
                    0.99894527, -0.02567799,  0.03806937,
                    -0.030139,   -0.99211383,  0.12166599]

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    360
    det_range:     100.0
    extrinsic_est_en:  true      # 原本为true: enable the online estimation of IMU-LiDAR extrinsic
    extrinsic_T: [0.07, 0.0, 0.072]
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1]

# 用于矫正Fast-lio2 结果到IMU上，实机用，仿真不需要
#imu2lidar:
#    extrinsic_T: [ 0.046263, 0.005615, 0.113574]
 #   extrinsic_R: [-9.34155765e-01, 1.15741066e-16, 3.56865531e-01,
 #                   7.18268579e-03, -9.99797428e-01, 1.88018925e-02,
 #                   3.56793240e-01, 2.01271492e-02, 9.33966531e-01]


publish:
    path_en:  true
    scan_publish_en:  true       # false: close all the point cloud output
    dense_publish_en: true       # false: low down the points number in a global-frame point clouds scan.
    scan_bodyframe_pub_en: true  # true: output the point cloud scans in IMU-body-frame

pcd_save:
    pcd_save_en: false
    interval: 10                 # how many LiDAR frames saved in each pcd file; 
                                 # -1 : all frames will be saved in ONE pcd file, may lead to memory crash when having too much frames.
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math

current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def local_position_cb(msg):
    global current_pose
    current_pose = msg

def distance_to_target(target):
    dx = current_pose.pose.position.x - target[0]
    dy = current_pose.pose.position.y - target[1]
    dz = current_pose.pose.position.z - target[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=local_position_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    target = [0.0, 0.0, 1.0]
    pose.pose.position.x = target[0]
    pose.pose.position.y = target[1]
    pose.pose.position.z = target[2]

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    rospy.loginfo("正在起飞到 1m 高度...")

    while not rospy.is_shutdown():   # ← 永久循环，永不退出
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD 模式已启用")
            last_req = rospy.Time.now()

        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd).success:
                rospy.loginfo("飞行器已解锁")
            last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        if distance_to_target(target) < 0.12:
            rospy.loginfo("已到达1m！现在保持悬停，EGO Planner 可以接管了")

        rate.sleep()
        


