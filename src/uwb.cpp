#include <chrono>
#include <memory>
#include <string>
#include <sstream>

#include "uwb_localization/crap.h"

using namespace std::chrono_literals;

constexpr auto uwbDevicePath = "/dev/uwb_module";
const auto timeOut           = serial::Timeout::simpleTimeout(20);

uwb::uwb() : Node("uwb") {
	try {
		uwbSerial = std::make_shared<serial::Serial>(uwbDevicePath, 115200, timeOut);
		// uwbSerial->open();
	} catch (const serial::IOException &e) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open port: %s", e.what());
	}

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s is opened.", uwbDevicePath);
	uwbDataThread = std::jthread(&uwb::uwbDataThreadCb, this);

	running.store(true);
	// Create a publisher for localization msg
	publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("uwb_loc", 10);

	timer_ = this->create_wall_timer(200ms, std::bind(&uwb::timerCallback, this));
}

void uwb::cleanup() {
	// Stop the car
	geometry_msgs::msg::Twist shutMsg;
	shutMsg.linear.set__x(0).set__y(0).set__z(0);
	publisher_->publish(shutMsg);

	// Stop the serial thread
	running.store(false);
	uwbSerial->close();
}

void uwb::timerCallback() {
	uwbData data;
	auto message = geometry_msgs::msg::Twist();
	if (parseData(data)) {
		if (data.paused) {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Paused");
		}

		else {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %d cm, Degree: %f", data.distance, data.degree);
			message.linear.x  = data.distance / 100.0; // cm to m
			message.angular.z = data.degree;
		}
	}
	this->publisher_->publish(message);
};

void uwb::uwbDataThreadCb() {
	while (rclcpp::ok() && running.load()) {
		if (uwbSerial->available()) {
			uwbDataStr   = uwbSerial->readline();
			uwbDataAvail = true;
			uwbSerial->flush();
		}
	}
}

bool uwb::parseData(uwbData &data) {
	// MPxxxx,tag_id,x_cm,y_cm,distance_cm,range_number,pdoa_deg,aoa_deg,distance_offset_cm,pdoa_offset_deg,A1_distance_cm,key\r\n
	if (uwbDataAvail) {
		// Check if the data is *probably* valid
		if (uwbDataStr.starts_with("MP")) {
			std::stringstream ss(uwbDataStr);

			// Split the string by comma
			std::string token;
			std::vector<std::string> tokens;
			while (getline(ss, token, ',')) {
				tokens.push_back(token);
			}
			data.distance = std::stoi(tokens[4]);
			data.degree   = std::stof(tokens[7]);
			data.paused = tokens[11].starts_with('0');

			uwbDataAvail = false;
			return true;
		}
		uwbDataAvail = false;
	}
	return false;
}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_unique<uwb>();
	rclcpp::spin(std::move(node));
	node->cleanup();

	rclcpp::shutdown();
	return 0;
}
