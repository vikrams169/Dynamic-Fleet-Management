# ENPM808X-final-project-boilerplate

[![codecov](https://codecov.io/gh/TommyChangUMD/ENPM808X-final-project-boilerplate/branch/main/graph/badge.svg?token=KRAHD3BZP7)](https://codecov.io/gh/TommyChangUMD/ENPM808X-final-project-boilerplate)

![CICD Workflow status](https://github.com/TommyChangUMD/ENPM808X-final-project-boilerplate/actions/workflows/my_codecov_upload.yml/badge.svg)

This repo provides a template for setting up:

  - GitHub CI
    - "main" branch runs in a ROS 2 Humble container
  - Codecov badges
  - Colcon workspace structure
  - C++ library that depends on other system libraries such as OpenCV.
    - The library is *self-contained and does not depend on ROS.*
  - ROS 2 package that depends on a C++ library built in the same colcon workspace
  - Establishing package dependency within the colcon workspace.
    - ie. the ROS 2 package will never be built first before the dependent C++ library
  - Multiple subscription within a ROS2 node all listening to the same topic.
    - Only one callback function is needed.
    - More efficient than to have N callback functions.
    - More efficient than to have N ROS nodes.
  - unit test and integration test
  - Doxygen setup
  - ROS2 launch file
  - bash scripts so they can be invoked by issuing the "ros2 run ..." command
  
## How to see package dependency graph

``` bash
colcon graph --dot | dot -Tpng -o depGraph.png
open depGraph.png
```
[<img src=screenshots/depGraph.png
    width="20%" 
    style="display: block; margin: 0 auto"
    />](screenshots/depGraph.png)



## How to build and run demo

```bash
rm -rf build/ install/
colcon build 
source install/setup.bash
ros2 launch my_controller run_demo.launch.py
```

## How to build for tests (unit test and integration test)

```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
```

## How to generate coverage reports
### my_controller

``` bash
ros2 run my_controller generate_coverage_report.bash
open build/my_controller//my_controller/test_coverage/index.html

```

## How to use GitHub CI to upload coverage report to Codecov

### First, sign up Codecov with you GitHub account.

  https://about.codecov.io/sign-up/

### Then, follow the similar instruction in the cpp-boilerplate-v2 repo

  https://github.com/TommyChangUMD/cpp-boilerplate-v2