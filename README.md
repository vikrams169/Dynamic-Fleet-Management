# Dynamic-Fleet-Management

[![DFM Tests](https://github.com/vikrams169/Dynamic-Fleet-Management/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)](https://github.com/vikrams169/Dynamic-Fleet-Management/actions/workflows/run-unit-test-and-upload-codecov.yml)
[![codecov](https://codecov.io/gh/vikrams169/Dynamic-Fleet-Management/graph/badge.svg?token=DNDF5KKAKK)](https://codecov.io/gh/vikrams169/Dynamic-Fleet-Management)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

**DFM**: A swarm robotics platform that incorporates multi-agent collision avoidance and path-planning capabilities to lead multiple robots to their goal positions in a shared obstacle-ridden environment

This repository contains the deliverables for the Endterm Project of **Vikram Setty** (119696897) and **Vinay Lanka** (12041665) as a part of the course *ENPM808X: Software Development for Robotics* at the University of Maryland.

## Phase 1

### Project Overview
We present DFM (Dynamic Fleet Management), a multi-robot swarm management platform for Acme Robotics’ warehouse robot group. This project would give a set of warehouse robots the ability to autonomously navigate to their goal positions in a warehouse setting while avoiding collisions with nearby agents.

DFM works using an RVO 2-based centralized path planner that gives ideal velocities based on specified goal positions and other agents' positions. It does so by using the Reciprocal Collision Avoidance Algorithm, using which it receives a high level of performance.

### About the Authors
The authors of DFM are Vikram Setty and Vinay Lanka, both robotics graduate students at the University of Maryland.

Vikram is from Hyderabad, India, and has done his bachelor's and master's degrees with a major in mechanical engineering and a minor in computer science from IIT Ropar. His research interests include perception, navigation, and path planning for robotics and autonomous systems. He is also interested in various areas in artificial intelligence and machine learning, especially computer vision and reinforcement learning.

Vinay is from Hyderabad, India, and has done his bachelor's degree majoring in Electronics and Communication Engineering from VIT Vellore. He has two years of work experience in Robotics, having worked as a Robotics Engineer in Newspace Research and Technologies (Defence Aerospace) and as an R&D Engineer in Neoflux. He's interested in the areas of perception and planning of robots and also shares the common interest of Deep Learning and Computer Vision, especially in the field of Robotics.

### AIP Workflow Used
This project was developed using the Agile Development Process (AIP) along with pair programming (with a driver and navigator), with a focus on test-driven development (TDD). [This](https://docs.google.com/spreadsheets/d/1S3s_57Yvaj8MZw6J9p7SwZRwz12uO2v6gIpWTRvaI18/edit?usp=sharing) sheet has the product backlog, iteration backlogs, and work log for each task done to develop DFM. The end of each iteration is even tagged to distinguish each sprint. Further, the link to the sprint planning and review meeting notes devised to overview each iteration sprint to develop DFM in the most efficient way possible is attached [here](https://docs.google.com/document/d/1pLAjcp51Vj-sIhFrzhY8Alna1NpD0xXxPJ0TkzX-hBk/edit?usp=sharing).

The latest (Phase 1) developed UML class and activity diagrams can be found in the `UML/initial` directory. Any changes to the UMLs in Phase 2 will be added to a different subdirectory within the `UML/` directory.

A short video providing a brief overview of the project and the details explaining the AIP process used is embedded below. A direct link to the same can also be found [here](https://i3.ytimg.com/vi/DN5dJNnc8Vk/maxresdefault.jpg).

[![Video](https://i3.ytimg.com/vi/DN5dJNnc8Vk/maxresdefault.jpg)](https://www.youtube.com/watch?v=DN5dJNnc8Vk)

### Dependencies

#### RVO2
The main dependency for using DFM is RVO 2 - an algorithm for interactive navigation and planning of large numbers of agents in two-dimensional (crowded) environments. At runtime, each agent senses the environment independently and computes a collision-free motion based on the optimal reciprocal collision avoidance (ORCA) formulation. Our algorithm ensures that each agent exhibits no oscillatory behaviors.

The user specifies static obstacles, agents, and the preferred velocities of the agents. The simulation is performed step-by-step via a simple call to the library. The simulation is fully accessible and manipulable during runtime. 

A link to the library can be found [here](https://gamma.cs.unc.edu/RVO2/documentation/2.0/index.html).

## Building the code

Run the commands listed below to build the package and libraries.
```bash
rm -rf build/ install/
colcon build 
source install/setup.bash
```

### Building for Unit and Integration Tests

Further, run these commands to build for running the unit tests and integration tests.
```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

### Running Unit and Integration

Execute the following commands to run the previosuly built unit and integration tests.
```bash
source install/setup.bash
colcon test
```

### Generating Coverage Reports After Running Colcon Test

To obtain the coverage reports (first make sure to have run the unit test already), execute the comands listed below.
```bash
colcon test
```

### Getting the Test Coverage Report for `dynamic_fleet_management`:

To get the test coverage for the ROS 2 package, *dynamic_fleet_management*, run the commands listed below.
``` bash
ros2 run dynamic_fleet_management generate_coverage_report.bash
open build/dynamic_fleet_management/test_coverage/index.html
```

### Getting the Test Coverage Report for `swarm_control`:

To get the same coverage report as last time, however for the library *swarm_control*, run the commands below.
``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select swarm_control \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
open build/swarm_control/test_coverage/index.html
```

### Generating the Combined Test Coverage Report

To obtain the combined test coverage report for the package and library, execute the following in the project's package directory.
``` bash
./do-tests.bash
```

### Generating the Project Documentation

To get the documentation for the project, run the commands below in the project's package directory.
``` bash
./do-docs.bash
```

### Google Coding Style Verification
To check how the written code conforms to the Google C++ style guide, look at the `results/cpplint_output.txt` file to see the output on using the *cpplint* tool on this project. You should not be able to see any issues or problems, with all the files processed successfully.

This can be self-verified as well by running the following command in the highest-level directory of the project.
```sh
# Install cpplint(ignore if already installed):
  sudo apt install cpplint
# Navigate to the 'src' directory
  cd src/
# Self-check Google code style conformity using Cpplint:
  cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order swarm_control/src/*.cpp swarm_control/include/*hpp dynamic_fleet_management/src/*.cpp
```

On running the above command, you should see the same output in the `results/cpplint_output.txt` file.


### Static Code Analysis
To check the static code analysis of this project, check the `results/cppcheck_output.txt` file to see the output on using the *cppcheck* tool. You should not be able to see any issues or problems, with all the files checked successfully.

This can be self-verified as well by running the following command in the highest-level directory of the project.
```sh
# Install cppcheck (ignore if already installed):
  sudo apt install cppcheck
# Navigate to the 'src' directory
  cd src/
# Self-check the static code analysis using Cppcheck:
  cppcheck --enable=all --std=c++11 --std=c++17 --enable=information --check-config --suppress=missingInclude --suppress=*:*test*/ --suppress=unmatchedSuppression $( find . -name *.cpp | grep -vE -e "^./build/")
```

On running the above command, you should see the same output in the `results/cppcheck_output.txt` file.