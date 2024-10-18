cmake_minimum_required(VERSION 3.8)
project(melfa_rv5as_masterclass)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(melfa_msgs REQUIRED)
find_package(melfa_masterclass_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(plc_ src/plc_.cpp)
target_include_directories(plc_ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(plc_ PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(plc_ ${THIS_PACKAGE_INCLUDE_DEPENDS} melfa_msgs melfa_masterclass_msgs std_msgs rclcpp)

add_executable(hmi_ src/hmi_.cpp)
target_include_directories(hmi_ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hmi_ PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(hmi_ ${THIS_PACKAGE_INCLUDE_DEPENDS} melfa_msgs melfa_masterclass_msgs std_msgs rclcpp)


add_executable(pick_n_place_ src/pick_n_place_.cpp)
target_include_directories(pick_n_place_ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_n_place_ PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pick_n_place_ ${THIS_PACKAGE_INCLUDE_DEPENDS} moveit_ros_planning_interface melfa_msgs melfa_masterclass_msgs std_msgs rclcpp)

add_executable(ex2 src/ex2.cpp)
target_include_directories(ex2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ex2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ex2 ${THIS_PACKAGE_INCLUDE_DEPENDS} moveit_ros_planning_interface melfa_msgs rclcpp)

add_executable(ex3 src/ex3.cpp)
target_include_directories(ex3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ex3 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ex3 ${THIS_PACKAGE_INCLUDE_DEPENDS} moveit_ros_planning_interface melfa_msgs rclcpp)


install(TARGETS plc_ hmi_ pick_n_place_
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)

ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
