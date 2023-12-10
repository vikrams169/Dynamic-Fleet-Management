/**
 * @file SimAgentImplementation.cpp
 * @author Vikram Setty (vikrams@umd.edu)
 * @brief The file that implements stubs (empty function placeholders) for the functions of the 'SimAgent' class
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
#include <SimAgentHeader.hpp>

/**
 * @brief The constructor for the 'SimAgent' class that initializes the agent's position and velocity
 * 
 */
SimAgent::SimAgent(std::vector<double> initial_pos, std::vector<double> initial_vel){

}

/**
 * @brief The function that updates the agent's position and velocity based on the input received from the high-level planner
 * 
 * @param new_vel The new specified velocity of the agent as specified by the planner
 * @return * void 
 */
void SimAgent::move_agent(std::vector<double> new_vel){
  return;
}

/**
 * @brief The destructor for the 'SimAgent' class that deallocates all the dynamically created memory
 * 
 */
SimAgent::~SimAgent(){
  
}