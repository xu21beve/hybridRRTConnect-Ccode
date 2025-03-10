# cmake_minimum_required(VERSION 3.8)
# project(visualize)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(Qt5 REQUIRED COMPONENTS Widgets)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(ament_lint_auto REQUIRED)

# include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

# set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs" "visualization_msgs")


# add_executable(ros_visualize src/ros_visualize.cpp)
# target_link_libraries(ros_visualize Qt5::Widgets)
# ament_target_dependencies(ros_visualize ${dependencies})

# install(TARGETS
#   ros_visualize
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()
