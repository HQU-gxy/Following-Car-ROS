#pragma once

#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

struct uwbData
{
    uint16_t distance;
    float degree;
    bool paused;
};

class uwb : public rclcpp::Node
{
public:
    uwb();
    void cleanup();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::shared_ptr<serial::Serial> uwbSerial;

    std::jthread uwbDataThread;
    std::atomic_bool running;
    std::string uwbDataStr;
    bool uwbDataAvail;

    void timerCallback();
    void uwbDataThreadCb();
    bool parseData(uwbData &data);
};