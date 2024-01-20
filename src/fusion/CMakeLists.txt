cmake_minimum_required(VERSION 3.8)
project(fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(autodiff REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS} #for fusemuse library i didin't have to specify this but for motion library it was essential! weird! Good practice to include this!
)

add_library(
  motion SHARED #shared to nazari .a mide yani static lib- i had to make it shared so that i do not get linker error in the load.cpp in the simple_model pkg
  src/state_space.cpp
  src/motion_model_factory.cpp 
  src/motion_model.cpp 
  src/constant_heading_rate.cpp
  src/car.cpp
)
set_target_properties(motion PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_target_dependencies(motion PUBLIC rclcpp pluginlib)
target_link_libraries(motion PUBLIC Eigen3::Eigen autodiff::autodiff)
add_executable(fusion src/main.cpp)
ament_target_dependencies(fusion PUBLIC rclcpp)
target_link_libraries(fusion PUBLIC fusemuse motion sensor_data Eigen3::Eigen)

add_library(
  visualization
  src/visualization.cpp
)
ament_target_dependencies(visualization PUBLIC rclcpp std_msgs visualization_msgs geometry_msgs)

add_library(
  sensor_data
  src/sensor_data.cpp
  src/mediator.cpp
)
ament_target_dependencies(sensor_data PUBLIC rclcpp geometry_msgs sensor_msgs std_msgs nav_msgs)

add_library(
  fusemuse
  src/filter_node.cpp
  src/filter_factory.cpp
  src/fusion.cpp
  src/ekf.cpp
)
ament_target_dependencies(fusemuse PUBLIC rclcpp geometry_msgs sensor_msgs tf2_ros )
target_link_libraries(fusemuse PUBLIC motion sensor_data Eigen3::Eigen visualization)


# set(fusion_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
# message("fusion_INCLUDE_DIR is set to: ${fusion_INCLUDE_DIR}")
install(TARGETS fusion  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf worlds params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/fusion DESTINATION include) #necessary for ament_export_include_directories and export is neccary for the simple model pkg so that ament_target_dependencies have the include directory information
install(TARGETS motion  DESTINATION lib)



ament_python_install_package(script)
install(PROGRAMS
  script/gui.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(motion)
ament_export_dependencies(Eigen3 autodiff)
ament_package()
