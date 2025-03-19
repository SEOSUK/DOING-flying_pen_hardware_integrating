cmake_minimum_required(VERSION 3.8)
project(test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # Add Eigen3
find_package(Curses REQUIRED)
find_package(crazyflie_interfaces REQUIRED)  # 추가

#SU_RVIZ
add_executable(su_rviz src/su_rviz.cpp)
ament_target_dependencies(su_rviz
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs
 crazyflie_interfaces)

install(TARGETS
  su_rviz
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(su_rviz PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

target_link_libraries(su_rviz)

#SU_FKIK
add_executable(su_fkik src/su_fkik.cpp src/ButterworthFilter.cpp)
ament_target_dependencies(su_fkik
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs
 crazyflie_interfaces)

install(TARGETS
  su_fkik
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(su_fkik PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

target_link_libraries(su_fkik ButterworthFilter)

#butterworth filter
add_library(ButterworthFilter src/ButterworthFilter.cpp)
target_include_directories(ButterworthFilter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

###

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
