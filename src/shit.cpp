#include <chrono>
#include <memory>
#include <string>
#include <sstream>

#include "stupid_car/crap.h"

using namespace std::chrono_literals;


Car::Car() : Node("Car") {
	auto onMessage = [this](geometry_msgs::msg::Twist msg) {
		if ((!msg.linear.x) && (!msg.angular.z)) {
			return;
		}

		this->uwbData.first  = msg.linear.x;
		this->uwbData.second = msg.angular.z;
		this->uwbDataAvail   = true;
	};

	subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/uwb_loc", 10, onMessage);
	timer_        = this->create_wall_timer(200ms, std::bind(&Car::timerCallback, this));
	publisher_    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // Create a publisher for control msg
}

void Car::cleanup() {
	// Stop the car
	geometry_msgs::msg::Twist shutMsg;
	shutMsg.linear.set__x(0).set__y(0).set__z(0);
	publisher_->publish(shutMsg);
}

void Car::timerCallback() {
	auto ctrl_message = geometry_msgs::msg::Twist();
	if (uwbDataAvail) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %f cm, Degree: %f", uwbData.first, uwbData.second);
		// Go forward when the distance is greater than 150cm
		if (uwbData.first > 1.5) {
			ctrl_message.linear.x = std::min(uwbData.first * 0.1, 0.8);
		}
		// Go back when the distance is less than 100cm
		else if (uwbData.first < 1) {
			ctrl_message.linear.x = -0.5;
		}
		ctrl_message.angular.z = uwbData.second * 0.01;
	}
	this->publisher_->publish(ctrl_message);
	uwbDataAvail = false;
};


int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_unique<Car>();
	rclcpp::spin(std::move(node));
	node->cleanup();

	rclcpp::shutdown();
	return 0;
}
