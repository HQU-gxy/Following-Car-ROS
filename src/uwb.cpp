#include <chrono>
#include <memory>
#include <string>
#include <sstream>

#include "uwb_localization/crap.h"

using namespace std::chrono_literals;

uwb::uwb() : Node("uwb")
{
  // Try to fuck the serial port's ass
  const std::string uwbDevicePath = "/dev/uwb_module";
  const auto timeOut = serial::Timeout::simpleTimeout(20);
  try
  {
    uwbSerial = std::make_shared<serial::Serial>(uwbDevicePath, 115200, timeOut);
    // uwbSerial->open();
  }
  catch (const serial::IOException &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open port: %s", e.what());
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s is opened.", uwbDevicePath.c_str());
  uwbDataThread = std::jthread(&uwb::uwbDataThreadCb, this);

  running.store(true);
  // Create a publisher for control msg
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = this->create_wall_timer(200ms, std::bind(&uwb::timerCallback, this));
}

void uwb::cleanup()
{
  // Stop the car
  geometry_msgs::msg::Twist shutMsg;
  shutMsg.linear.set__x(0).set__y(0).set__z(0);
  publisher_->publish(shutMsg);

  // Stop the serial thread
  running.store(false);
  uwbSerial->close();
}

void uwb::timerCallback()
{
  uwbData data;
  auto message = geometry_msgs::msg::Twist();
  if (parseData(data))
  {
    if (data.paused)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Paused");

    else
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %d cm, Degree: %f", data.distance, data.degree);

      // Go forward when the distance is greater than 150cm
      if (data.distance > 150)
        message.linear.x = std::min(data.distance * 0.001, 0.8);
      // Go back when the distance is less than 100cm
      else if (data.distance < 100)
        message.linear.x = -0.5;

      message.angular.z = data.degree * 0.01;
    }
  }
  this->publisher_->publish(message);
};

void uwb::uwbDataThreadCb()
{
  while (rclcpp::ok() && running.load())
  {
    if (uwbSerial->available())
    {
      uwbDataStr = uwbSerial->readline();
      uwbDataAvail = true;
      uwbSerial->flush();
    }
  }
}

bool uwb::parseData(uwbData &data)
{
  // MPxxxx,tag_id,x_cm,y_cm,distance_cm,range_number,pdoa_deg,aoa_deg,distance_offset_cm,pdoa_offset_deg,A1_distance_cm,key\r\n
  if (uwbDataAvail)
  {
    if (uwbDataStr.starts_with("MP")) // Check if the data is *probably* valid
    {
      std::stringstream ss(uwbDataStr);

      // Split the string by comma
      std::string token;
      std::vector<std::string> tokens;
      while (getline(ss, token, ','))
      {
        tokens.push_back(token);
      }
      data.distance = std::stoi(tokens[4]);
      data.degree = std::stof(tokens[7]);
      RCLCPP_INFO(rclcpp::get_logger("paused"), tokens[11].c_str());
      data.paused = tokens[11].starts_with('0');

      uwbDataAvail = false;

      return true;
    }
    uwbDataAvail = false;
  }
  return false;
}

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  auto node = std::make_shared<uwb>();
  rclcpp::spin(node);
  node->cleanup();

  rclcpp::shutdown();
  return 0;
}
