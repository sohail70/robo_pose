cmake_minimum_required(VERSION 3.8)
project(model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(autodiff REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fusion REQUIRED)
find_package(pluginlib REQUIRED)

add_library(model src/model.cpp)
target_compile_features(model PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  model
  "autodiff"
  "Eigen3"
  "fusion"
  "pluginlib"
)
target_link_libraries(model Eigen3::Eigen autodiff::autodiff)

pluginlib_export_plugin_description_file(fusion plugins.xml)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(model PRIVATE "MODEL_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS model
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(load src/load.cpp)
ament_target_dependencies(load "pluginlib" "fusion")
target_link_libraries(load "Eigen3::Eigen" "autodiff::autodiff")
install(TARGETS load DESTINATION lib/${PROJECT_NAME})

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  model
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
