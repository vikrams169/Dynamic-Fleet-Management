/**
 * @file EnvironmentImplementation.cpp
 * @author Vikram Setty (vikrams@umd.edu) Vinay Lanka (vlanka@umd.edu)
 * @brief The file that implements stubs (empty function placeholders) for the
functions of the 'Environment' class
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
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "EnvironmentHeader.hpp"
#include "SimAgentHeader.hpp"
#include "Vector2.h"
/**
 * @brief The constructor for the 'Environment' class that initializes the
 * simulation and agents (and their goal positions)
 *
 */
Environment::Environment(int num_agents, float timestep,
                         std::vector<std::vector<double>> start_positions,
                         std::vector<RVO::Vector2> goal_positions) {
  sim = new RVO::RVOSimulator();
  // Random Number Generator Seed
  std::srand(static_cast<unsigned int>(std::time(NULL)));
  // Set time step
  sim->setTimeStep(timestep);
  // Specify default parameters for agents that are subsequently added.
  sim->setAgentDefaults(15.0f, 10, 10.0f, 5.0f, 2.0f, 2.0f);
  for (int i = 0; i < num_agents; i++) {
    RVO::Vector2 startPositon = {static_cast<float>(start_positions[i][0]),
                                 static_cast<float>(start_positions[i][1])};
    sim->addAgent(startPositon);

    agent_goals.push_back(goal_positions[i]);

    std::vector<double> initial_pos = start_positions[i];
    std::vector<double> initial_vel{0.0f, 0.0f};
    SimAgent agent(initial_pos, initial_vel);
    agents.push_back(agent);
  }
}

void Environment::perform_iteration() {
  // Set preferred Velocity Step
  for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
    RVO::Vector2 goalVector = agent_goals[i] - sim->getAgentPosition(i);

    if (RVO::absSq(goalVector) > 1.0f) {
      goalVector = RVO::normalize(goalVector);
    }
    float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
    float dist = std::rand() * 0.0001f / RAND_MAX;
    sim->setAgentPrefVelocity(i, goalVector);
    sim->setAgentPrefVelocity(
        i, sim->getAgentPrefVelocity(i) +
               dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
  }
  // Perform Sim Step
  sim->doStep();
  // Store Current State in Sim Agent
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    RVO::Vector2 velocity = sim->getAgentVelocity(i);
    RVO::Vector2 positon = sim->getAgentPosition(i);
    std::vector<double> desired_vel{velocity.x(), velocity.y()};
    std::vector<double> desired_pos{positon.x(), positon.y()};
    agents[i].update_desired_vel(desired_vel);
    agents[i].update_desired_pos(desired_pos);
  }
}

std::vector<std::vector<double>> Environment::getSimAgentDesiredVelocities() {
  std::vector<std::vector<double>> robot_velocities;
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    robot_velocities.push_back(agents[i].getSimAgentDesiredVelocity());
  }
  return robot_velocities;
}

std::vector<std::vector<double>> Environment::getSimAgentDesiredPositions() {
  std::vector<std::vector<double>> robot_positions;
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    robot_positions.push_back(agents[i].getSimAgentDesiredPosition());
  }
  return robot_positions;
}

void Environment::update_environment(
    int num_agents, std::vector<std::vector<double>> current_positions,
    std::vector<std::vector<double>> current_velocities) {
  for (int i = 0; i < num_agents; i++) {
    RVO::Vector2 currentPositon = {static_cast<float>(current_positions[i][0]),
                                   static_cast<float>(current_positions[i][1])};
    RVO::Vector2 currentVelocity = {
        static_cast<float>(current_velocities[i][0]),
        static_cast<float>(current_velocities[i][1])};
    // sim->setAgentVelocity(i,currentVelocity);
    // sim->setAgentPosition(i,currentPositon);
    std::vector<double> current_pos{current_positions[i][0],
                                    current_positions[i][1],
                                    current_positions[i][2]};
    std::vector<double> current_vel{current_positions[i][0],
                                    current_positions[i][1],
                                    current_positions[i][2]};
    agents[i].update_agent(current_pos, current_vel);
  }
}

/**
 * @brief The destructor of the Environment class that deallocates all
 * dynamically created memory
 *
 */
Environment::~Environment() { delete sim; }
