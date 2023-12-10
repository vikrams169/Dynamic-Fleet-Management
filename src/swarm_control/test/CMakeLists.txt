# Any C++ source files needed to build this target (my_model_tester).
add_executable(swarm_control_tester
  # list of source cpp files:
  main.cpp
  test.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(swarm_control_tester PUBLIC
  # list of include directories:
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/libs/RVO2/src> 
  )

# Any dependent libraires needed to build this target.
target_link_libraries(swarm_control_tester PUBLIC
  # list of libraries:
  gtest
  swarm_control
  )

# Enable CMake’s test runner to discover the tests included in the
# binary, using the GoogleTest CMake module.
gtest_discover_tests(swarm_control_tester)


# # Build executable:
# add_executable (my_model_tester
#   test/main.cpp
#   )
# target_link_libraries(my_model_tester
#   # list of libraries
#   my_model
#   gtest
#   )
