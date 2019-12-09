# Install script for directory: /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/lab4_cam

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lab4_cam/srv" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/lab4_cam/srv/ImageSrv.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lab4_cam/cmake" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/lab4_cam/catkin_generated/installspace/lab4_cam-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/include/lab4_cam")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/share/roseus/ros/lab4_cam")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/share/common-lisp/ros/lab4_cam")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/share/gennodejs/ros/lab4_cam")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/lib/python2.7/dist-packages/lab4_cam")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/lib/python2.7/dist-packages/lab4_cam")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/lab4_cam/catkin_generated/installspace/lab4_cam.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lab4_cam/cmake" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/lab4_cam/catkin_generated/installspace/lab4_cam-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lab4_cam/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/lab4_cam/catkin_generated/installspace/lab4_camConfig.cmake"
    "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/lab4_cam/catkin_generated/installspace/lab4_camConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lab4_cam" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/lab4_cam/package.xml")
endif()

