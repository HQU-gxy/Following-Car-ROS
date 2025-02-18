#include <chrono>
#include <memory>
#include <string>
#include <sstream>

#include "car/crap.h"

using namespace std::chrono_literals;

constexpr float ANGULAR_K       = 0.8;
constexpr float MIN_ANGULAR_VEL = 0.5;
constexpr float MAX_ANGULAR_VEL = 1;
constexpr float ANGULAR_TOLER   = 0.5;

constexpr float LINEAR_K       = 0.2;
constexpr float MIN_LINEAR_VEL = 0.2;
constexpr float MAX_LINEAR_VEL = 2.0;
constexpr float REACHED_DIST   = 1.5;
constexpr float GO_BACK_DIST   = 1.0;

Car::Car() : Node("Car") {
	// declare_parameter<float>("angular_vel", 0.5);

	auto onMessage = [this](geometry_msgs::msg::Twist uwbMsg) {
		geometry_msgs::msg::Twist ctrl_message;
		ctrl_message.linear.x  = 0;
		ctrl_message.angular.z = 0;

		if ((!uwbMsg.linear.x) && (!uwbMsg.angular.z)) {
			publisher_->publish(ctrl_message);
			return;
		}

		float linear  = uwbMsg.linear.x;
		float angular = uwbMsg.angular.z;
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %f m, Degree: %f", linear, angular);


		// Go forward when the distance is greater than 150cm
		if (linear > REACHED_DIST) {
			ctrl_message.linear.x = std::max(MIN_LINEAR_VEL, std::min(linear * LINEAR_K, MAX_LINEAR_VEL));
		}
		// Go back when the distance is less than 100cm
		else if (linear < GO_BACK_DIST) {
			ctrl_message.linear.x = -0.5;
		}

		if (angular > ANGULAR_TOLER) {
			if (!lastIsLinear || lastIsLinear++ >= 3) {
				lastIsLinear           = 0;
				ctrl_message.angular.z = std::max(MIN_ANGULAR_VEL, std::min(angular * ANGULAR_K, MAX_ANGULAR_VEL));
				ctrl_message.linear.x  = 0;
			}
		} else if (angular > ANGULAR_TOLER / 2) {
			lastIsLinear           = 1;
			ctrl_message.angular.z = std::max(MIN_ANGULAR_VEL, std::min(angular * ANGULAR_K, MAX_ANGULAR_VEL));
		} else if (angular < -ANGULAR_TOLER) {
			if (!lastIsLinear || lastIsLinear++ >= 3) {
				lastIsLinear           = 0;
				ctrl_message.linear.x  = 0;
				ctrl_message.angular.z = -std::max(MIN_ANGULAR_VEL, std::min(-angular * ANGULAR_K, MAX_ANGULAR_VEL));
			}
		} else if (angular < -ANGULAR_TOLER / 2) {
			lastIsLinear           = 1;
			ctrl_message.angular.z = -std::max(MIN_ANGULAR_VEL, std::min(-angular * ANGULAR_K, MAX_ANGULAR_VEL));
		} else {
			lastIsLinear = 1;
		}


		this->publisher_->publish(ctrl_message);
	};

	subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/uwb_loc", 10, onMessage);
	publisher_    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // Create a publisher for control msg
}

void Car::cleanup() {
	// Stop the car
	geometry_msgs::msg::Twist shutMsg;
	shutMsg.linear.set__x(0).set__y(0).set__z(0);
	publisher_->publish(shutMsg);
}


int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_unique<Car>();
	rclcpp::spin(std::move(node));
	node->cleanup();

	rclcpp::shutdown();
	return 0;
}