cmake_minimum_required(VERSION 3.10)
project(tr5_demo_test)
IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
ENDIF ()
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tr5_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/quill
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsoncpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco
  ${PROJECT_SOURCE_DIR}/include/tr5_demo_test
  ${CERES_INCLUDE_DIRS}
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/quill
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsoncpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/lin_arm64

)

set(SOURCES
    src/GLFW_callbacks.cpp
    src/mpc.cpp
    src/wbc_priority.cpp
    src/demo.cpp
)

add_executable(demo_node src/walk_mpc_wbc_copy.cpp src/data_logger.cpp src/mock_interface.cpp src/pino_kin_dyn.cpp src/MJ_interface.cpp src/mpc.cpp src/wbc_priority.cpp
      src/LPF_fst.cpp src/priority_tasks.cpp src/ramp_trajectory.cpp src/useful_math.cpp src/foot_placement.cpp src/gait_scheduler.cpp
      src/GLFW_callbacks.cpp src/joystick_interpreter.cpp src/PVT_ctrl.cpp src/bezier_1D.cpp src/csv_input_loader.cpp src/Eul_W_filter.cpp src/StateEst.cpp)

add_executable(sensor_publisher src/publisher.cpp)

add_executable(sensor_publisher2 src/publisher2.cpp)

target_link_libraries(demo_node
  pinocchio_lin_arm64 
  urdfdom_model_lin_arm64 
  console_bridge_lin_arm64
  tinyxml_lin_arm64
  quill_lin_arm64
  glfw3_lin_arm64
  jsoncpp_lin_arm64
  qpOASES_lin_arm64
  mujoco
  dl
  ${CERES_LIBRARIES}
)

set_target_properties(demo_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN/../../third_party/mujoco/lin_arm64"
)

ament_target_dependencies(demo_node 
                    rclcpp 
                    std_msgs
                    tf2
                    tf2_ros
                    geometry_msgs
                    sensor_msgs
                    nav_msgs
                    nmea_msgs
                    mavros_msgs
                    tr5_interfaces
                    rosidl_default_runtime)

ament_target_dependencies(sensor_publisher rclcpp std_msgs tr5_interfaces)

ament_target_dependencies(sensor_publisher2 rclcpp std_msgs tr5_interfaces)

install(TARGETS
  sensor_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sensor_publisher2
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  demo_node
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

