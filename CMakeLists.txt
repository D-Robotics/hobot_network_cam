cmake_minimum_required(VERSION 3.8)
project(hobot_network_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# include
include_directories(include
  ${CMAKE_SYSROOT}/usr/
  ${CMAKE_SYSROOT}/usr/include
  ${CMAKE_SYSROOT}/usr/hobot/include
)

# link
link_directories(
  ${CMAKE_SYSROOT}/usr/lib
  ${CMAKE_SYSROOT}/usr/hobot/lib
  ${CMAKE_SYSROOT}/usr/lib/hbmedia
  ${CMAKE_SYSROOT}/usr/lib/system/lib
)

# add executable
add_executable(hobot_network_cam src/main.cpp src/video_get_codec.c)
target_include_directories(hobot_network_cam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hobot_network_cam PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(hobot_network_cam
  rclcpp
  sensor_msgs
)
target_link_libraries(hobot_network_cam multimedia avformat avcodec avutil ssl crypto drm z)

# install
install(TARGETS hobot_network_cam DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
