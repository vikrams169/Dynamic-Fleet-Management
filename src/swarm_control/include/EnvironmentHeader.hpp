/**
 * @file EnvironmentHeader.hpp
 * @author Vikram Setty (vikrams@umd.edu) Vinay Lanka (vlanka@umd.edu)
 * @brief The header file that initializes the class that deals with the
simulation environments and all the agents together
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
#include <RVO.h>

#include <memory>
#include <vector>

#include "SimAgentHeader.hpp"

#pragma once

/**
 * @brief The class that manages all the agents and runs iterations of the
 * simulation
 *
 */
class Environment {
 private:
  std::vector<SimAgent> agents;
  RVO::RVOSimulator *sim;
  std::vector<RVO::Vector2> agent_goals;
  // std::vector<bool> reached_goals;
  // double goal_radius;
  // double total_sim_time;
  // void change_goal_position(int agent_number,
  //                           std::vector<RVO::Vector2> new_goal_pos);
  // bool reached_goal(int agent_number);
 public:
  Environment(int num_agents, float timestep,
              std::vector<std::vector<double>> start_positions,
              std::vector<RVO::Vector2> goal_positions);
  void update_environment(int num_agents,
                          std::vector<std::vector<double>> current_positions,
                          std::vector<std::vector<double>> current_velocities);
  void perform_iteration();
  std::vector<std::vector<double>> getSimAgentDesiredVelocities();
  std::vector<std::vector<double>> getSimAgentDesiredPositions();
  // std::vector<std::vector<double>> setSimAgentVelocities();
  ~Environment();
};
