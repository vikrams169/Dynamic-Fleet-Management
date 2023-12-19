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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
// #include <RVO.h>

#include "EnvironmentHeader.hpp"
// #include "SimAgentHeader.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @brief A class that gives a ROS 2 functionality to the 'swarm control'
 * library to simulate collision avoidance in Gazebo using Turtlebot 3 Waffle
 *
 */
class RobotCommandPublisher : public rclcpp::Node {
 public:
  RobotCommandPublisher() : Node("robot_commander") {
    // Start Simulation Create Env
    std::vector<std::vector<double>> start_positions{
        robot_1_pose, robot_2_pose, robot_3_pose, robot_4_pose};
    RVO::Vector2 robot_1_goal(5.0f, 5.0f);
    RVO::Vector2 robot_2_goal(-5.0f, -5.0f);
    RVO::Vector2 robot_3_goal(-5.0f, 5.0f);
    RVO::Vector2 robot_4_goal(5.0f, -5.0f);

    std::vector<RVO::Vector2> goal_positions{robot_1_goal, robot_2_goal,
                                             robot_3_goal, robot_4_goal};
    testenv = new Environment(4, 0.5, start_positions, goal_positions);
    // Publishers
    velocity_pub_robot1 = this->create_publisher<geometry_msgs::msg::Twist>(
        "/robot1/cmd_vel", 10);
    velocity_pub_robot2 = this->create_publisher<geometry_msgs::msg::Twist>(
        "/robot2/cmd_vel", 10);
    velocity_pub_robot3 = this->create_publisher<geometry_msgs::msg::Twist>(
        "/robot3/cmd_vel", 10);
    velocity_pub_robot4 = this->create_publisher<geometry_msgs::msg::Twist>(
        "/robot4/cmd_vel", 10);
    timer = this->create_wall_timer(
        500ms, std::bind(&RobotCommandPublisher::timer_callback, this));
    // Odom Subscribers
    robot1_odom_subscription =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom", 10,
            std::bind(&RobotCommandPublisher::robot1_odom_callback, this,
                      std::placeholders::_1));
    robot2_odom_subscription =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot2/odom", 10,
            std::bind(&RobotCommandPublisher::robot2_odom_callback, this,
                      std::placeholders::_1));
    robot3_odom_subscription =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot3/odom", 10,
            std::bind(&RobotCommandPublisher::robot3_odom_callback, this,
                      std::placeholders::_1));
    robot4_odom_subscription =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot4/odom", 10,
            std::bind(&RobotCommandPublisher::robot4_odom_callback, this,
                      std::placeholders::_1));

    iteration_complete = false;
    orientation_complete = false;
  }

 private:
  /**
   * @brief The called function from the subscriber callback that uses the
   * published odometry topic to get the pose of the specified robot (closing
   * the loop)
   *
   * @param msg The '/odom' topic message
   * @return * std::vector<double> The pose of the robot
   */
  std::vector<double> extract_pose(nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    std::vector<double> robot_pose;
    robot_pose.push_back(x);
    robot_pose.push_back(y);
    robot_pose.push_back(yaw);
    return robot_pose;
  }

  /**
   * @brief The ododmetry callback for Robot 1
   *
   * @param msg '/odom' topic message
   * @return * void
   */
  void robot1_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_1_pose = extract_pose(msg);
  }

  /**
   * @brief The odometry callback for Robot 2
   *
   * @param msg '/odom' topic message
   * @return * void
   */
  void robot2_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_2_pose = extract_pose(msg);
  }

  /**
   * @brief The odometry calllback for Robot 3
   *
   * @param msg '/odom' topic message
   * @return * void
   */
  void robot3_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_3_pose = extract_pose(msg);
  }

  /**
   * @brief The odometry callback for Robot 4
   *
   * @param msg '/odom' topic message
   * @return * void
   */
  void robot4_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_4_pose = extract_pose(msg);
  }

  /**
   * @brief The timer callback for the '/cmd_vel' publisher for the
   * robots/agents that publishes the required angular velocity followed by the
   * required linear velocity
   *
   * @return * void
   */
  void timer_callback() {
    std::vector<std::vector<double>> robot_poses{robot_1_pose, robot_2_pose,
                                                 robot_3_pose, robot_4_pose};
    if (!iteration_complete) {
      RCLCPP_INFO_STREAM(this->get_logger(), "New iteration");
      testenv->perform_iteration();
      iteration_complete = !iteration_complete;
      orientation_complete = false;
    }
    std::vector<std::vector<double>> robot_desired_positions =
        testenv->getSimAgentDesiredPositions();
    // Heading Control
    std::vector<double> desired_headings;
    for (int i = 0; i < 4; i++) {
      desired_headings.push_back(
          std::atan2((robot_desired_positions[i][1] - robot_poses[i][1]),
                     (robot_desired_positions[i][0] - robot_poses[i][0])));
    }
    auto velocity_msg_robot1 = geometry_msgs::msg::Twist();
    auto velocity_msg_robot2 = geometry_msgs::msg::Twist();
    auto velocity_msg_robot3 = geometry_msgs::msg::Twist();
    auto velocity_msg_robot4 = geometry_msgs::msg::Twist();

    if (abs(desired_headings[0] - robot_poses[0][2]) > 0.05) {
      velocity_msg_robot1.linear.x = 0.0;
      if (desired_headings[0] - robot_poses[0][2] > 0) {
        velocity_msg_robot1.angular.z = 0.05;
      } else {
        velocity_msg_robot1.angular.z = -0.05;
      }
      velocity_pub_robot1->publish(velocity_msg_robot1);
    } else {
      velocity_msg_robot1.linear.x = 0.0;
      velocity_msg_robot1.angular.z = 0.0;
      velocity_pub_robot1->publish(velocity_msg_robot1);
    }
    if (abs(desired_headings[1] - robot_poses[1][2]) > 0.05) {
      velocity_msg_robot2.linear.x = 0.0;
      if (desired_headings[1] - robot_poses[1][2] > 0) {
        velocity_msg_robot2.angular.z = 0.05;
      } else {
        velocity_msg_robot2.angular.z = -0.05;
      }
      velocity_pub_robot2->publish(velocity_msg_robot2);
    } else {
      velocity_msg_robot2.linear.x = 0.0;
      velocity_msg_robot2.angular.z = 0.0;
      velocity_pub_robot2->publish(velocity_msg_robot2);
    }

    if (abs(desired_headings[2] - robot_poses[2][2]) > 0.05) {
      velocity_msg_robot3.linear.x = 0.0;
      if (desired_headings[2] - robot_poses[2][2] > 0) {
        velocity_msg_robot3.angular.z = 0.05;
      } else {
        velocity_msg_robot3.angular.z = -0.05;
      }
      velocity_pub_robot3->publish(velocity_msg_robot3);
    } else {
      velocity_msg_robot3.linear.x = 0.0;
      velocity_msg_robot3.angular.z = 0.0;
      velocity_pub_robot3->publish(velocity_msg_robot3);
    }
    if (abs(desired_headings[3] - robot_poses[3][2]) > 0.05) {
      velocity_msg_robot4.linear.x = 0.0;
      if (desired_headings[3] - robot_poses[3][2] > 0) {
        velocity_msg_robot4.angular.z = 0.05;
      } else {
        velocity_msg_robot4.angular.z = -0.05;
      }
      velocity_pub_robot4->publish(velocity_msg_robot4);
    } else {
      velocity_msg_robot4.linear.x = 0.0;
      velocity_msg_robot4.angular.z = 0.0;
      velocity_pub_robot4->publish(velocity_msg_robot4);
    }
    if ((abs(desired_headings[0] - robot_poses[0][2]) < 0.05) &&
        (abs(desired_headings[1] - robot_poses[1][2]) < 0.05) &&
        (abs(desired_headings[2] - robot_poses[2][2]) < 0.05) &&
        (abs(desired_headings[3] - robot_poses[3][2]) < 0.05)) {
      orientation_complete = true;
    }
    if (orientation_complete) {
      double distance_robot_1 = std::sqrt(
          std::pow((robot_desired_positions[0][1] - robot_poses[0][1]), 2) +
          std::pow((robot_desired_positions[0][0] - robot_poses[0][0]), 2));
      double distance_robot_2 = std::sqrt(
          std::pow((robot_desired_positions[1][1] - robot_poses[1][1]), 2) +
          std::pow((robot_desired_positions[1][0] - robot_poses[1][0]), 2));
      double distance_robot_3 = std::sqrt(
          std::pow((robot_desired_positions[2][1] - robot_poses[2][1]), 2) +
          std::pow((robot_desired_positions[2][0] - robot_poses[2][0]), 2));
      double distance_robot_4 = std::sqrt(
          std::pow((robot_desired_positions[3][1] - robot_poses[3][1]), 2) +
          std::pow((robot_desired_positions[3][0] - robot_poses[3][0]), 2));
      if (abs(distance_robot_1) > 0.05) {
        velocity_msg_robot1.linear.x = 0.1;
        velocity_msg_robot1.angular.z = 0.0;
        velocity_pub_robot1->publish(velocity_msg_robot1);
      }
      if (abs(distance_robot_2) > 0.05) {
        velocity_msg_robot2.linear.x = 0.1;
        velocity_msg_robot2.angular.z = 0.0;
        velocity_pub_robot2->publish(velocity_msg_robot2);
      }
      if (abs(distance_robot_3) > 0.05) {
        velocity_msg_robot3.linear.x = 0.1;
        velocity_msg_robot3.angular.z = 0.0;
        velocity_pub_robot3->publish(velocity_msg_robot3);
      }
      if (abs(distance_robot_4) > 0.05) {
        velocity_msg_robot4.linear.x = 0.1;
        velocity_msg_robot4.angular.z = 0.0;
        velocity_pub_robot4->publish(velocity_msg_robot4);
      }
      if ((abs(distance_robot_1) < 0.05) && (abs(distance_robot_2) < 0.05) &&
          (abs(distance_robot_3) < 0.05) && (abs(distance_robot_4) < 0.05)) {
        iteration_complete = false;
      }
    }
  }
  bool orientation_complete;
  bool iteration_complete;
  Environment* testenv;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_robot1;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_robot2;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_robot3;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_robot4;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      robot1_odom_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      robot2_odom_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      robot3_odom_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      robot4_odom_subscription;
  std::vector<double> robot_1_pose{-5.0f, -5.0f, 0.0};
  std::vector<double> robot_2_pose{5.0f, 5.0f, 0.0};
  std::vector<double> robot_3_pose{5.0f, -5.0f, 0.0};
  std::vector<double> robot_4_pose{-5.0f, 5.0f, 0.0};
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
