/**
 * @file SimAgentImplementation.cpp
 * @author Vikram Setty (vikrams@umd.edu) Vinay Lanka (vlanka@umd.edu)
 * @brief The file that implements stubs (empty function placeholders) for the
functions of the 'SimAgent' class
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
 * @brief The constructor for the 'SimAgent' class that initializes the agent's
 * position and velocity
 *
 */
SimAgent::SimAgent(std::vector<double> initial_pos,
                   std::vector<double> initial_vel) {
  position[0] = initial_pos[0];
  position[1] = initial_pos[1];
  heading = initial_pos[2];
  velocity = initial_vel;
}

/**
 * @brief The function that updates the agent's position and velocity based on
 * the input received from the high-level planner
 *
 * @param new_vel The new specified velocity of the agent as specified by the
 * planner
 * @return * void
 */
void SimAgent::update_agent(std::vector<double> current_pos,
                            std::vector<double> current_vel) {
  position[0] = current_pos[0];
  position[1] = current_pos[1];
  heading = current_pos[2];
  velocity[0] = current_vel[0];
  velocity[1] = current_vel[1];
  // velocity[0] =
  // std::sqrt(std::pow(current_vel[0],2)+std::pow(current_vel[1],2));
  // velocity[1] = std::atan2(current_vel[1],current_vel[0]);
}

void SimAgent::update_desired_vel(std::vector<double> desired_vel) {
  desired_velocity = desired_vel;
}
void SimAgent::update_desired_pos(std::vector<double> desired_pos) {
  desired_position = desired_pos;
}

std::vector<double> SimAgent::getSimAgentDesiredVelocity() {
  return desired_velocity;
}

std::vector<double> SimAgent::getSimAgentDesiredPosition() {
  return desired_position;
}

/**
 * @brief The destructor for the 'SimAgent' class that deallocates all the
 * dynamically created memory
 *
 */
SimAgent::~SimAgent() {}
