# Install script for directory: /home/niic/tr5-orin/tr5_test/src/tr5_demo_test

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/install/tr5_demo_test")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test" TYPE EXECUTABLE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/sensor_publisher")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher"
         OLD_RPATH "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/urdfdom:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/pinocchio:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/quill:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/glfw:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/jsoncpp:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/qpOASES:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/mujoco/lin_arm64:/opt/ros/humble/lib:/home/niic/tr5-orin/ros2_ws/install/tr5_interfaces/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher2")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher2"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test" TYPE EXECUTABLE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/sensor_publisher2")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher2")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher2"
         OLD_RPATH "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/urdfdom:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/pinocchio:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/quill:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/glfw:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/jsoncpp:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/qpOASES:/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/third_party/mujoco/lin_arm64:/opt/ros/humble/lib:/home/niic/tr5-orin/ros2_ws/install/tr5_interfaces/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/sensor_publisher2")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test" TYPE EXECUTABLE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/demo_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/demo_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/demo_node")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tr5_demo_test/demo_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/tr5_demo_test")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/tr5_demo_test")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test/environment" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test/environment" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_index/share/ament_index/resource_index/packages/tr5_demo_test")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test/cmake" TYPE FILE FILES
    "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_core/tr5_demo_testConfig.cmake"
    "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/ament_cmake_core/tr5_demo_testConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tr5_demo_test" TYPE FILE FILES "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/niic/tr5-orin/tr5_test/src/tr5_demo_test/build/tr5_demo_test/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
