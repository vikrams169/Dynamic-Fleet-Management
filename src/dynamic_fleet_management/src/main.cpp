/**
 * @file main.cpp
 * @author Vikram Setty (vikrams@umd.edu) Vinay Lanka (vlanka@umd.edu)
 * @brief The script that calls ROS 2 functionality to implment ROS 2 nodes
using the library functions already implemented.
 * @version 0.1
 * @date 2023-12-09
 *
 * @copyright Copyright (c) 2023 Vikram Setty, Vinay Lanka

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "EnvironmentHeader.hpp"
#include "SimAgentHeader.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class RobotCommandPublisher : public rclcpp::Node {
 public:
  RobotCommandPublisher() : Node("robot_commander") {
    velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer = this->create_wall_timer(
        500ms, std::bind(&RobotCommandPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto velocity_msg = geometry_msgs::msg::Twist();
    velocity_msg.linear.x = 0.01;
    velocity_msg.angular.z = 0.01;
    velocity_pub->publish(velocity_msg);;
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
  rclcpp::TimerBase::SharedPtr timer;
};

/**
 * @brief The main function where all calls to ROS node functionalities are made
 *
 * @return * int
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
