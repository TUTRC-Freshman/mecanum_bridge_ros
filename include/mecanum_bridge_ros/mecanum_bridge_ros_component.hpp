#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace mecanum_bridge_ros {

class MecanumBridgeRosComponent : public rclcpp::Node {
public:
  MecanumBridgeRosComponent(const rclcpp::NodeOptions &options) : MecanumBridgeRosComponent("", options) {}

  MecanumBridgeRosComponent(const std::string &name_space = "",
                            const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("mecanum_bridge_ros", name_space, options) {

    serial_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", rclcpp::QoS{10});
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS{10}, std::bind(&MecanumBridgeRosComponent::twist_sub_cb, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  void twist_sub_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    {
      std_msgs::msg::UInt8MultiArray serial_msg = get_en_msg('A', 'A', 800);
      serial_pub_->publish(std::move(serial_msg));
    }

    {
      std_msgs::msg::UInt8MultiArray serial_msg = get_in_msg('A', 1, true);
      serial_pub_->publish(std::move(serial_msg));
    }

    {
      std_msgs::msg::UInt8MultiArray serial_msg = get_in_msg('A', 2, false);
      serial_pub_->publish(std::move(serial_msg));
    }
  }

  uint8_t checksum(const std::vector<uint8_t> &data) {
    uint8_t chk = 0;
    for (const auto &e : data) {
      chk += e;
    }
    return (~chk + 1) & 0xFF;
  }

  std_msgs::msg::UInt8MultiArray get_en_msg(char board, char motor, uint16_t duty) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = std::vector<uint8_t>{0xFF, 0, board, 'E', motor, duty, duty >> 8};
    msg.data[1] = msg.data.size() - 2;
    uint8_t chk = checksum(msg.data);
    msg.data.push_back(chk);
    return msg;
  }

  std_msgs::msg::UInt8MultiArray get_in_msg(char board, uint8_t pin, bool value) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = std::vector<uint8_t>{0xFF, 0, board, 'I', pin, value};
    msg.data[1] = msg.data.size() - 2;
    uint8_t chk = checksum(msg.data);
    msg.data.push_back(chk);
    return msg;
  }
};

} // namespace mecanum_bridge_ros
