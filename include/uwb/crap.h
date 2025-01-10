#pragma once

#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "serial/serial.h"

struct uwbData {
	uint16_t distance; // In cm
	float degree;      // In deg
	bool paused;
};

class uwb : public rclcpp::Node {
public:
	uwb();
	void cleanup();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	std::shared_ptr<serial::Serial> uwbSerial;

	std::string uwbDataStr;
	bool uwbDataAvail;
	bool parseData(uwbData &data);

	void uwbDataThreadCb();
	void timerCallback();
};