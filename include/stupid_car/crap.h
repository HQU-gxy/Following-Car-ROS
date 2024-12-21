#pragma once

#include <stdint.h>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class Car : public rclcpp::Node {
public:
	Car();
	void cleanup();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_; // Pose to Nav2
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;     // Location from UWB module

	tf2_ros::Buffer::SharedPtr tf2Buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf2Listener_;
};