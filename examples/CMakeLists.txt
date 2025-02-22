cmake_minimum_required(VERSION 3.8)

find_package(ompl REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
# find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executable for multicopter.cpp
add_executable(multicopter multicopter.cpp)
target_link_libraries(multicopter HyRRT ${OMPL_LIBRARIES})

# Add executable for visualize.cpp
add_executable(visualize visualize/src/ros_visualize.cpp)
target_link_libraries(visualize ${RCLCPP_LIBRARIES})

# Add executable for bouncing_ball.cpp
# add_executable(bouncing_ball bouncing_ball.cpp)
# target_link_libraries(bouncing_ball HyRRT ${OMPL_LIBRARIES})

project(visualize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(visualize rclcpp std_msgs geometry_msgs visualization_msgs rosidl_typesupport_c)

install(TARGETS
visualize
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Include necessary directories
include_directories(${OMPL_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
