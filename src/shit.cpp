#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "stupid_car/crap.h"

using namespace std::chrono_literals;


Car::Car() : Node("Car") {
	auto onMessage = [this](geometry_msgs::msg::Twist msg) {
		try {
			auto distance = msg.linear.x;  // m
			auto angle    = msg.angular.z; // rad
			if ((!distance) && (!angle)) {
				return;
			}

			geometry_msgs::msg::PoseStamped relativeLocation;
			relativeLocation.header.frame_id    = "uwb";
			relativeLocation.pose.position.x    = distance * cos(angle);
			relativeLocation.pose.position.y    = distance * sin(angle);
			relativeLocation.pose.position.z    = 0;
			relativeLocation.pose.orientation.w = 1;

			auto goalMessage = tf2Buffer_->transform(relativeLocation, "map");
			this->posePublisher_->publish(goalMessage);
		} catch (tf2::TransformException &e) {
			RCLCPP_WARN(get_logger(), "Could not transform: %s", e.what());
		}
	};

	subscription_  = this->create_subscription<geometry_msgs::msg::Twist>("/uwb_loc", 10, onMessage);
	posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10); // Create a publisher for control msg

	tf2Buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
	tf2Listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
}


int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_unique<Car>();
	rclcpp::spin(std::move(node));

	rclcpp::shutdown();
	return 0;
}
