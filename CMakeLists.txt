cmake_minimum_required(VERSION 3.8)
project(ros2_can_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  can_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
#
add_executable(can_interface src/can_interface.cpp)
ament_target_dependencies(can_interface ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_include_directories(can_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS can_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()
