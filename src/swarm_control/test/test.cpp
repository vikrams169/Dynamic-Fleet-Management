/**
 * @file test.cpp
 * @author Vikram Setty (vikrams@umd.edu) Vinay Lanka (vlanka@umd.edu)
 * @brief The file that contains the tests for the entire library
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

#include "EnvironmentHeader.hpp"
#include "SimAgentHeader.hpp"

/**
 * @brief A basic test to check integer equality
 *
 */
TEST(Test1, Case1) { EXPECT_EQ(1, 1); }

/**
 * @brief A basic test to check numerical approximation
 *
 */
TEST(Test1, Case2) { ASSERT_NEAR(1, 1.01, 0.1); }

/**
 * @brief A basic test to check string equality
 *
 */
TEST(Test1, Case3) { EXPECT_STREQ("GoTerps", "GoTerps"); }

const int NO_OF_AGENTS = 2;
std::vector<RVO::Vector2> AGENT_GOALS;

/**
 * @brief Text fixture class for testing the Environment class.
 *
 */
// class EnvironmentTests : public testing::Test {
//  public:
//  protected:
//   void SetUp() override {
//     testEnviroment = new Environment(NO_OF_AGENTS, AGENT_GOALS);
//     std::cout << "Calling Fixture SetUp\n";
//   };

//   void TearDown() override {
//     delete testEnviroment;
//     std::cout << "Calling Fixture TearDown\n";
//   };
//   Environment *testEnviroment;
// };

// /**
//  * @brief Testing the perform_iteration method of the Environment class
//  *
//  */
// TEST_F(EnvironmentTests, test_perform_iteration) {
//   testEnviroment->perform_iteration();
// }

// std::vector<double> initial_pos = {0, 0};
// std::vector<double> initial_vel = {0, 0};
// std::vector<double> new_vel = {0, 0};

// /**
//  * @brief Text fixture class for testing the SimAgent class.
//  *
//  */
// class SimAgentTests : public testing::Test {
//  public:
//  protected:
//   void SetUp() override {
//     testSimAgent = new SimAgent(initial_pos, initial_vel);
//     std::cout << "Calling Fixture SetUp\n";
//   };

//   void TearDown() override {
//     delete testSimAgent;
//     std::cout << "Calling Fixture TearDown\n";
//   };
//   SimAgent *testSimAgent;
// };

// /**
//  * @brief Testing the move_agent method of the SimAgent class
//  *
//  */
// TEST_F(SimAgentTests, test_perform_iteration) {
//   testSimAgent->move_agent(new_vel);
// }
