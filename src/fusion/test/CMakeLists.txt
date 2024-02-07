cmake_minimum_required(VERSION 3.8)
project(test)


find_package(GTest REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(autodiff REQUIRED)

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS} # nazarama build mishe ba matplot lib!
    ../include #matplotlib header only ro ham gozashtam in to 
    )

set(TEST_SOURCES
    test_main.cpp 
    # test_motion_model.cpp
    # test_visualization.cpp
    test_eigen.cpp
    # test_ekf.cpp
    # # test_state_space.cpp
    # test_autodiff.cpp
    # ../src/motion_model_factory.cpp
    # ../src/motion_model.cpp
    # ../src/constant_heading_rate.cpp
    # ../src/quadcopter.cpp
    # ../src/visualization.cpp
    # ../src/sensor_data.cpp
    # ../src/ekf.cpp 
    # ../src/fusion.cpp
    # ../src/state_space.cpp
    # ../src/mediator.cpp

)

add_executable(run_tests ${TEST_SOURCES})
target_link_libraries(run_tests ${GTEST_BOTH_LIBRARIES} pthread Python3::Python Python3::NumPy Eigen3::Eigen autodiff::autodiff) #python  ro nazari matplot lib kar nemide
ament_target_dependencies(run_tests rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs)

enable_testing()
add_test(NAME YourTestSuite COMMAND run_tests)