cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(GLUT REQUIRED) 

# add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(pose_subscriber src/pose_subscriber.cpp)
add_executable(move_test src/move_test.cpp)


# target_include_directories(hello_moveit PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   hello_moveit
#   "moveit_ros_planning_interface"
#   "rclcpp"
#   "geometry_msgs"
#   "moveit_core"
# )

ament_target_dependencies(
  pose_subscriber 
  "rclcpp" 
  "geometry_msgs"
  "moveit_ros_planning_interface"
  "moveit_core"
  # "moveit_ros_perception"
  )

ament_target_dependencies(
  move_test 
  "rclcpp" 
  "geometry_msgs"
  "moveit_ros_planning_interface"
  "moveit_msgs"

  )

#   # Link GLUT library
# target_link_libraries(pose_subscriber GLUT::GLUT)
# target_link_libraries(move_test GLUT::GLUT)

  # install(TARGETS hello_moveit
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pose_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  move_test
  DESTINATION lib/${PROJECT_NAME}
)

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
