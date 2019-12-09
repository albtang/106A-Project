# Install script for directory: /home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/ar_track_alvar/ar_track_alvar_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/msg" TYPE FILE FILES
    "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/ar_track_alvar/ar_track_alvar_msgs/msg/AlvarMarker.msg"
    "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/ar_track_alvar/ar_track_alvar_msgs/msg/AlvarMarkers.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/cmake" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/include/ar_track_alvar_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/share/roseus/ros/ar_track_alvar_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/share/common-lisp/ros/ar_track_alvar_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/share/gennodejs/ros/ar_track_alvar_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/lib/python2.7/dist-packages/ar_track_alvar_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/devel/lib/python2.7/dist-packages/ar_track_alvar_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/cmake" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgsConfig.cmake"
    "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs" TYPE FILE FILES "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/src/ar_track_alvar/ar_track_alvar_msgs/package.xml")
endif()

