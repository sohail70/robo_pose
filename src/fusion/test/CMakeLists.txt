cmake_minimum_required(VERSION 3.8)
project(test)


find_package(GTest REQUIRED)
include_directories(
    ${GTEST_INCLUDE_DIRS}
    ../include
    )



set(TEST_SOURCES
    test_main.cpp 
    test_motion_model_factory.cpp 
    ../src/motion_model_factory.cpp
    ../src/motion_model.cpp
    ../src/constant_heading_rate.cpp
    ../src/car.cpp

)

add_executable(run_tests ${TEST_SOURCES})
target_link_libraries(run_tests ${GTEST_BOTH_LIBRARIES} pthread)

enable_testing()
add_test(NAME YourTestSuite COMMAND run_tests)