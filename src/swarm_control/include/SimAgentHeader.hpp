/**
 * @file SimAgentHeader.hpp
 * @author Vikram Setty (vikrams@umd.edu) Vinay Lanka (vlanka@umd.edu)
 * @brief The header that initializes the class that deals with individual
agents
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
#include <memory>
#include <vector>

#pragma once

/**
 * @brief The class that maintains and updates a single agent's position and
 * velocity, along with the specified desired velocity
 *
 */
class SimAgent {
 private:
  std::vector<double> position{0.0f, 0.0f};
  double heading;
  std::vector<double> velocity;
  std::vector<double> desired_velocity;
  std::vector<double> desired_position;

 public:
  SimAgent(std::vector<double> initial_pos, std::vector<double> initial_vel);
  void update_desired_vel(std::vector<double> desired_vel);
  void update_desired_pos(std::vector<double> desired_pos);
  void update_agent(std::vector<double> current_pos,
                    std::vector<double> current_vel);
  std::vector<double> getSimAgentDesiredVelocity();
  std::vector<double> getSimAgentDesiredPosition();
  ~SimAgent();
};
