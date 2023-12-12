/**
 * @file EnvironmentImplementation.cpp
 * @author Vikram Setty (vikrams@umd.edu) Vinay Lanka (vlanka@umd.edu)
 * @brief The file that implements stubs (empty function placeholders) for the functions of the 'Environment' class
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
#include "EnvironmentHeader.hpp"
#include "Vector2.h"

/**
 * @brief The constructor for the 'Environment' class that initializes the simulation and agents (and their goal positions)
 * 
 */
Environment::Environment(int num_agents, std::vector<RVO::Vector2> goal_positions){

}

/**
 * @brief A function to change the goal position of a particular agent
 * 
 * @param agent_number Agent number/ID whose goal position needs to be changed
 * @param new_goal_pos New goal position of the agent
 * @return * void 
 */
void Environment::change_goal_position(int agent_number,std::vector<RVO::Vector2> new_goal_pos){
  return;
}

/**
 * @brief A function to check whether the agent reached thir goal or not
 * 
 * @param agent_number The agent number/ID whose condition is being checked
 * @return true The agent has reached their gal
 * @return false The agent has not reached their goal
 */
bool Environment::reached_goal(int agent_number){
  return true;
}

/**
 * @brief A high-level function that runs a single iteration of the simulation
 * 
 * @return * void 
 */
void Environment::perform_iteration(){
  return;
}

/**
 * @brief The destructor of the Environment class that deallocates all dynamically created memory
 * 
 */
Environment::~Environment(){
  
}
