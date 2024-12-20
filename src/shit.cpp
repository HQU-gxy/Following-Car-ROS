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

	subscription_  = this->create_subscription<geometry_msgs::msg::Twist>("/uwb_loc", 10, onMessage);
	timer_         = this->create_wall_timer(200ms, std::bind(&Car::timerCallback, this));
	posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_update", 10); // Create a publisher for control msg
}

void Car::timerCallback() {
	geometry_msgs::msg::PoseStamped goal_message;

	// TODO: transform UWB Data to pose on the map

	this->posePublisher_->publish(goal_message);
	uwbDataAvail = false;
};


int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_unique<Car>();
	rclcpp::spin(std::move(node));

	rclcpp::shutdown();
	return 0;
}
