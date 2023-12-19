/**
 * @file basic_test.cpp
 * @author Vikram Setty (vikrams@umd.edu)
 * @brief The testing suite for the ROS 2 functionality
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
#include <gtest/gtest.h>
#include <stdlib.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief The class that sets up and initializes the ROS 2 tests
 *
 */
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup should occur before every test instance.
    RCLCPP_INFO_STREAM(node_->get_logger(), "SETUP!!");

    /*
     * 1.) Define any ros2 package and exectuable you want to test
     *  example: package name = cpp_pubsub, node name = minimal_publisher,
     * executable = talker
     */
    bool retVal = StartROSExec("dynamic_fleet_management", "robot_commander",
                               "robot_commander");
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    // Tear-down should occur after every test instance
    RCLCPP_INFO_STREAM(node_->get_logger(), "TEARDOWN!!");

    // Stop the running ros2 node, if any.
    bool retVal = StopROSExec();
    ASSERT_TRUE(retVal);

    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  bool StartROSExec(const char* pkg_name, const char* node_name,
                    const char* exec_name) {
    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];
    snprintf(execName, 16, "%s",
             exec_name);  // pkill uses exec name <= 15 char only
    killCmd_ss << "pkill --signal SIGINT " << execName
               << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopROSExec();

    // Start a ros2 node and wait for it to get ready:
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0) return false;

    // Wait for at most 10 seconds for the node to show up, otherwise it's an
    // error!
    retVal = -1;
    int count = 0;
    while ((count++ < 10) && (retVal != 0)) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return (retVal == 0);
  }

  bool StopROSExec() {
    // if node is not running, don't need to kill it
    if ((killCmd_ss.str().empty()) || system(cmdInfo_ss.str().c_str()) != 0)
      return true;

    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

/**
 * @brief Construct a new test f object for testing out command velocity
 *
 */
TEST_F(TaskPlanningFixture, Robot1CmdVel) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using geometry_msgs::msg::Twist;
  using SUBSCRIBER = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription =
      node_->create_subscription<geometry_msgs::msg::Twist>(
          "/robot1/cmd_vel", 10,
          // Lambda expression begins
          [&](const geometry_msgs::msg::Twist& msg) {
            RCLCPP_INFO(node_->get_logger(), "I heard: '%f'", msg.linear.x);
            hasData = true;
          }  // end of lambda expression
      );

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

/**
 * @brief Construct a new test f object for testing out command velocity
 *
 */
TEST_F(TaskPlanningFixture, Robot2CmdVel) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using geometry_msgs::msg::Twist;
  using SUBSCRIBER = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription =
      node_->create_subscription<geometry_msgs::msg::Twist>(
          "/robot2/cmd_vel", 10,
          // Lambda expression begins
          [&](const geometry_msgs::msg::Twist& msg) {
            RCLCPP_INFO(node_->get_logger(), "I heard: '%f'", msg.linear.x);
            hasData = true;
          }  // end of lambda expression
      );

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

/**
 * @brief Construct a new test f object for testing out command velocity
 *
 */
TEST_F(TaskPlanningFixture, Robot3CmdVel) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using geometry_msgs::msg::Twist;
  using SUBSCRIBER = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription =
      node_->create_subscription<geometry_msgs::msg::Twist>(
          "/robot3/cmd_vel", 10,
          // Lambda expression begins
          [&](const geometry_msgs::msg::Twist& msg) {
            RCLCPP_INFO(node_->get_logger(), "I heard: '%f'", msg.linear.x);
            hasData = true;
          }  // end of lambda expression
      );

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

/**
 * @brief Construct a new test f object for testing out command velocity
 *
 */
TEST_F(TaskPlanningFixture, Robot4CmdVel) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using geometry_msgs::msg::Twist;
  using SUBSCRIBER = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription =
      node_->create_subscription<geometry_msgs::msg::Twist>(
          "/robot4/cmd_vel", 10,
          // Lambda expression begins
          [&](const geometry_msgs::msg::Twist& msg) {
            RCLCPP_INFO(node_->get_logger(), "I heard: '%f'", msg.linear.x);
            hasData = true;
          }  // end of lambda expression
      );

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

using namespace std::chrono_literals;
/**
 * @brief Construct a new test f object for subscription
 *
 */
TEST_F(TaskPlanningFixture, Robot1Odom) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using nav_msgs::msg::Odometry;
  using PUBLISHER = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;
  bool hasData = false;
  PUBLISHER publisher =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot1/odom", 10);
  auto odom_msg_robot = nav_msgs::msg::Odometry();
  odom_msg_robot.header.stamp = node_->get_clock()->now();
  odom_msg_robot.header.frame_id = "odom";
  odom_msg_robot.child_frame_id = "base_footprint";
  odom_msg_robot.pose.pose.position.x = 0.0;
  odom_msg_robot.pose.pose.position.y = 0.0;
  odom_msg_robot.pose.pose.position.z = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;

  odom_msg_robot.twist.twist.linear.x = 0.0;
  odom_msg_robot.twist.twist.linear.y = 0.0;
  odom_msg_robot.twist.twist.linear.z = 0.0;
  odom_msg_robot.twist.twist.angular.x = 0.0;
  odom_msg_robot.twist.twist.angular.y = 0.0;
  odom_msg_robot.twist.twist.angular.z = 0.0;
  publisher->publish(odom_msg_robot);
  // /*
  //  * 3.) check to see if we get data winhin 3 sec
  //  */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    publisher->publish(odom_msg_robot);
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  // EXPECT_TRUE(hasData);
}

TEST_F(TaskPlanningFixture, Robot2Odom) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using nav_msgs::msg::Odometry;
  using PUBLISHER = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;
  bool hasData = false;
  PUBLISHER publisher =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot2/odom", 10);
  auto odom_msg_robot = nav_msgs::msg::Odometry();
  odom_msg_robot.header.stamp = node_->get_clock()->now();
  odom_msg_robot.header.frame_id = "odom";
  odom_msg_robot.child_frame_id = "base_footprint";
  odom_msg_robot.pose.pose.position.x = 0.0;
  odom_msg_robot.pose.pose.position.y = 0.0;
  odom_msg_robot.pose.pose.position.z = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;

  odom_msg_robot.twist.twist.linear.x = 0.0;
  odom_msg_robot.twist.twist.linear.y = 0.0;
  odom_msg_robot.twist.twist.linear.z = 0.0;
  odom_msg_robot.twist.twist.angular.x = 0.0;
  odom_msg_robot.twist.twist.angular.y = 0.0;
  odom_msg_robot.twist.twist.angular.z = 0.0;
  publisher->publish(odom_msg_robot);
  // /*
  //  * 3.) check to see if we get data winhin 3 sec
  //  */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    publisher->publish(odom_msg_robot);
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  // EXPECT_TRUE(hasData);
}

TEST_F(TaskPlanningFixture, Robot3Odom) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using nav_msgs::msg::Odometry;
  using PUBLISHER = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;
  bool hasData = false;
  PUBLISHER publisher =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot3/odom", 10);
  auto odom_msg_robot = nav_msgs::msg::Odometry();
  odom_msg_robot.header.stamp = node_->get_clock()->now();
  odom_msg_robot.header.frame_id = "odom";
  odom_msg_robot.child_frame_id = "base_footprint";
  odom_msg_robot.pose.pose.position.x = 0.0;
  odom_msg_robot.pose.pose.position.y = 0.0;
  odom_msg_robot.pose.pose.position.z = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;

  odom_msg_robot.twist.twist.linear.x = 0.0;
  odom_msg_robot.twist.twist.linear.y = 0.0;
  odom_msg_robot.twist.twist.linear.z = 0.0;
  odom_msg_robot.twist.twist.angular.x = 0.0;
  odom_msg_robot.twist.twist.angular.y = 0.0;
  odom_msg_robot.twist.twist.angular.z = 0.0;
  publisher->publish(odom_msg_robot);
  // /*
  //  * 3.) check to see if we get data winhin 3 sec
  //  */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    publisher->publish(odom_msg_robot);
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  // EXPECT_TRUE(hasData);
}

TEST_F(TaskPlanningFixture, Robot4Odom) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using nav_msgs::msg::Odometry;
  using PUBLISHER = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;
  bool hasData = false;
  PUBLISHER publisher =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot4/odom", 10);
  auto odom_msg_robot = nav_msgs::msg::Odometry();
  odom_msg_robot.header.stamp = node_->get_clock()->now();
  odom_msg_robot.header.frame_id = "odom";
  odom_msg_robot.child_frame_id = "base_footprint";
  odom_msg_robot.pose.pose.position.x = 0.0;
  odom_msg_robot.pose.pose.position.y = 0.0;
  odom_msg_robot.pose.pose.position.z = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 0.0;
  odom_msg_robot.pose.pose.orientation.x = 1.0;

  odom_msg_robot.twist.twist.linear.x = 0.0;
  odom_msg_robot.twist.twist.linear.y = 0.0;
  odom_msg_robot.twist.twist.linear.z = 0.0;
  odom_msg_robot.twist.twist.angular.x = 0.0;
  odom_msg_robot.twist.twist.angular.y = 0.0;
  odom_msg_robot.twist.twist.angular.z = 0.0;
  publisher->publish(odom_msg_robot);
  // /*
  //  * 3.) check to see if we get data winhin 3 sec
  //  */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while ((elapsed_time < 3s) && !hasData) {
    publisher->publish(odom_msg_robot);
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  // EXPECT_TRUE(hasData);
}

/**
 * @brief The main function that executes the testing functionality
 *
 * @param argc The number of command line arguments
 * @param argv The array of command line arguments
 * @return * int
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
