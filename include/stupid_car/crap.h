#pragma once

#include <stdint.h>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

class Car : public rclcpp::Node {
public:
	Car();
	void cleanup();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

	std::pair<float,float> uwbData; // Distance, Degree
	bool uwbDataAvail;

	void timerCallback();
};