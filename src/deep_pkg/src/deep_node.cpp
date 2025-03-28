#include <ros/ros.h>
#include <serial/serial.h>
#include <dmuuv_msgs/DepthSensor.h> // 包含新的消息类型
#include <sstream>
#include <iostream>
#include <iomanip> // for std::put_time
#include <regex>

serial::Serial ser;

// 用于发布带时间戳和深度信息的ROS话题
dmuuv_msgs::DepthSensor timestamped_depth_msg; // 修改命名空间

float extractFirstFloat(std::string& str) {
    std::regex floatRegex(R"(([-+]?\d*\.?\d+)([eE][-+]?\d+)?)");
    std::smatch match;
    
    if (std::regex_search(str, match, floatRegex) && match.size() > 0) {
        try {
            return std::stof(match.str(0));
        } catch (const std::invalid_argument& ia) {
            std::cerr << "Invalid argument: " << ia.what() << std::endl;
        } catch (const std::out_of_range& oor) {
            std::cerr << "Out of range: " << oor.what() << std::endl;
        }
    }
    return 0.0f; // 如果没有找到浮点数，返回0.0
}

// 回调函数，用于处理读取到的数据
void read_and_publish(ros::Publisher& pub) {
    if (ser.available()) {
        std::string line = ser.readline(); // 读取一行数据

        // 获取当前时间戳
        ros::Time current_time = ros::Time::now();
        timestamped_depth_msg.header.stamp = current_time;

        // 提取深度值
        std::istringstream iss(line);
        std::string iss_content;
        std::getline(iss, iss_content); // 读取整个行到iss_content

        timestamped_depth_msg.depth = extractFirstFloat(iss_content);
        pub.publish(timestamped_depth_msg);
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "timestamped_depth_publisher");
    ros::NodeHandle nh;

    // 设置串口参数并打开串口
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port: " << e.what());
        return -1;
    }

    if (!ser.isOpen()) {
        ROS_ERROR_STREAM("Failed to open serial port.");
        return -1;
    }

    // 创建一个发布者，指定话题名称和消息类型
    ros::Publisher timestamped_depth_pub = nh.advertise<dmuuv_msgs::DepthSensor>("timestamped_depth", 1000); // 修改命名空间
    ros::Rate loop_rate(2); // 设置循环频率为10Hz

    while (ros::ok()) {
        read_and_publish(timestamped_depth_pub);
        // ros::spinOnce();
        loop_rate.sleep();
    }

    // 关闭串口
    ser.close();
    return 0;
}
