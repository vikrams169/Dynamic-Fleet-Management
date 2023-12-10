/**
 * @file EnvironmentHeader.hpp
 * @author Vikram Setty (vikrams@umd.edu)
 * @brief The header file that initializes the class that deals with the simulation environments and all the agents together
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
#include<memory>
#include<RVO>
#include<my_dummy_lib_funct2.hpp>
#include<vector>

#pragma once

/**
 * @brief The class that manages all the agents and runs iterations of the simulation
 * 
 */
class Environment {
 private:
  std::vector<SimAgent> *agents;
  RVO::RVOSimulator *sim;
  std::vector<RVO::vector> agent_goals;
  std::vector<bool> reached_goals;
  double goal_radius;
  double total_sim_time;
  void change_goal_position(int agent_number,std::vector<RVO::vector> new_goal_pos);
  bool reached_goal(int agent_number);
 public:
  Environment(int num_agents, std::vector<RVO::vector> goal_positions);
  void perform_iteration();
  ~Environment();
};
