#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rtk_publisher");
    ros::NodeHandle nh("~");  // 使用私有命名空间参数
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/mavros/gps/fix", 10);
    // ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10);

    // 读取参数
    std::string port;
    int baud;
    std::string frame_id;

    nh.param<std::string>("port", port, "/dev/ttyTHS1");   // 默认值
    nh.param<int>("baud", baud, 115200);
    nh.param<std::string>("frame_id", frame_id, "gps");

    ROS_INFO("RTK Publisher started with:");
    ROS_INFO("  port: %s", port.c_str());
    ROS_INFO("  baud: %d", baud);
    ROS_INFO("  frame_id: %s", frame_id.c_str());

    // 打开串口
    serial::Serial ser;
    try {
        ser.setPort(port);
        ser.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port %s", port.c_str());
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO("Serial Port %s initialized", port.c_str());
    } else {
        return -1;
    }



    ros::Rate loop_rate(50);
    while (ros::ok()) {
        if (ser.available()) {
            std_msgs::String raw_msg;
            raw_msg.data = ser.readline();
            std::string line = raw_msg.data;

            // 解析 GNGGA (定位信息)
        if (line.find("$GNGGA") == 0) {
            sensor_msgs::NavSatFix fix;
            fix.header.stamp = ros::Time::now();
            fix.header.frame_id = frame_id; // launch 文件传入的 frame_id

            // 将 NMEA 字符串拆分
            std::vector<std::string> tokens;
            std::stringstream ss(line);
            std::string token;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            // NMEA GNGGA 字段：
            // 0:$GNGGA
            // 1:UTC time
            // 2:latitude (ddmm.mmmmm)
            // 3:N/S
            // 4:longitude (dddmm.mmmmm)
            // 5:E/W
            // 6:fix quality (0=无效,1=GPS,2=DGPS,4=RTK Fixed...)
            // 7:num satellites
            // 8:HDOP
            // 9:altitude (m)
            // 10: "M"
            // 11: geoid separation
            // 12: "M"
            // ...

            try {
                // 纬度解析
                if (!tokens[2].empty() && !tokens[3].empty()) {
                    double lat_deg = std::stod(tokens[2].substr(0,2));
                    double lat_min = std::stod(tokens[2].substr(2));
                    double latitude = lat_deg + lat_min / 60.0;
                    if (tokens[3] == "S") latitude = -latitude;
                    fix.latitude = latitude;
                }

                // 经度解析
                if (!tokens[4].empty() && !tokens[5].empty()) {
                    double lon_deg = std::stod(tokens[4].substr(0,3));
                    double lon_min = std::stod(tokens[4].substr(3));
                    double longitude = lon_deg + lon_min / 60.0;
                    if (tokens[5] == "W") longitude = -longitude;
                    fix.longitude = longitude;
                }

                // 高度解析
                if (!tokens[9].empty()) {
                    fix.altitude = std::stod(tokens[9]);
                }

                // 状态解析
                if (!tokens[6].empty()) {
                    int fix_quality = std::stoi(tokens[6]);
                    switch(fix_quality){
                        case 0: fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
                        case 1: fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; break;
                        case 2: fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX; break;
                        case 4: fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break; // RTK Fixed
                        default: fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
                    }
                    fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                }

            } catch (const std::exception& e) {
                ROS_WARN("GNGGA parse error: %s", e.what());
            }

            gps_pub.publish(fix);
        }

            // // 解析 GPHDT (航向角)
            // if (line.find("$GPHDT") == 0) {
            //     std::vector<std::string> tokens;
            //     std::stringstream ss(line);
            //     std::string item;
            //     while (std::getline(ss, item, ',')) {
            //         tokens.push_back(item);
            //     }

            //     if (tokens.size() >= 2) {
            //         try {
            //             double heading = std::stod(tokens[1]); // 航向角 (度)
            //             std_msgs::Float64 yaw_msg;
            //             yaw_msg.data = heading;
            //             yaw_pub.publish(yaw_msg);
            //         } catch (...) {
            //             // 忽略解析错误
            //         }
            //     }
            // }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
