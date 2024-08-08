#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <sensor_msgs/msg/joy.hpp>

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

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS{10}, std::bind(&MecanumBridgeRosComponent::joy_sub_cb, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void twist_sub_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double x = msg->linear.x;
    double y = msg->linear.y;
    // double z = msg->angular.z; // 旋回
    double V1 = (x - y) * 400;
    double V2 = (-x - y) * 400;
    double V3 = (-x + y) * 400;
    double V4 = (x + y) * 400;

    if (V1 < 0) {
      serial_pub_->publish(get_in_msg('B', 1, false));
      serial_pub_->publish(get_in_msg('B', 2, true));
    } else {
      serial_pub_->publish(get_in_msg('B', 1, true));
      serial_pub_->publish(get_in_msg('B', 2, false));
    }
    serial_pub_->publish(get_en_msg('B', 'A', std::abs(V1)));

    if (V2 < 0) {
      serial_pub_->publish(get_in_msg('B', 3, true));
      serial_pub_->publish(get_in_msg('B', 4, false));
    } else {
      serial_pub_->publish(get_in_msg('B', 3, false));
      serial_pub_->publish(get_in_msg('B', 4, true));
    }
    serial_pub_->publish(get_en_msg('B', 'B', std::abs(V2)));

    if (V3 < 0) {
      serial_pub_->publish(get_in_msg('A', 3, true));
      serial_pub_->publish(get_in_msg('A', 4, false));
    } else {
      serial_pub_->publish(get_in_msg('A', 3, false));
      serial_pub_->publish(get_in_msg('A', 4, true));
    }
    serial_pub_->publish(get_en_msg('A', 'B', std::abs(V3)));

    if (V4 < 0) {
      serial_pub_->publish(get_in_msg('A', 1, false));
      serial_pub_->publish(get_in_msg('A', 2, true));
    } else {
      serial_pub_->publish(get_in_msg('A', 1, true));
      serial_pub_->publish(get_in_msg('A', 2, false));
    }
    serial_pub_->publish(get_en_msg('A', 'A', std::abs(V4)));
  }

  void joy_sub_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
    double x = msg->axes[1] * -1;
    double y = msg->axes[0] * -1;
    // double z = msg->angular.z; // 旋回
    double V1 = (x - y) * 400;
    double V2 = (-x - y) * 400;
    double V3 = (-x + y) * 400;
    double V4 = (x + y) * 400;

    if (V1 < 0) {
      serial_pub_->publish(get_in_msg('B', 1, false));
      serial_pub_->publish(get_in_msg('B', 2, true));
    } else {
      serial_pub_->publish(get_in_msg('B', 1, true));
      serial_pub_->publish(get_in_msg('B', 2, false));
    }
    serial_pub_->publish(get_en_msg('B', 'A', std::abs(V1)));

    if (V2 < 0) {
      serial_pub_->publish(get_in_msg('B', 3, true));
      serial_pub_->publish(get_in_msg('B', 4, false));
    } else {
      serial_pub_->publish(get_in_msg('B', 3, false));
      serial_pub_->publish(get_in_msg('B', 4, true));
    }
    serial_pub_->publish(get_en_msg('B', 'B', std::abs(V2)));

    if (V3 < 0) {
      serial_pub_->publish(get_in_msg('A', 3, true));
      serial_pub_->publish(get_in_msg('A', 4, false));
    } else {
      serial_pub_->publish(get_in_msg('A', 3, false));
      serial_pub_->publish(get_in_msg('A', 4, true));
    }
    serial_pub_->publish(get_en_msg('A', 'B', std::abs(V3)));

    if (V4 < 0) {
      serial_pub_->publish(get_in_msg('A', 1, false));
      serial_pub_->publish(get_in_msg('A', 2, true));
    } else {
      serial_pub_->publish(get_in_msg('A', 1, true));
      serial_pub_->publish(get_in_msg('A', 2, false));
    }
    serial_pub_->publish(get_en_msg('A', 'A', std::abs(V4)));
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
