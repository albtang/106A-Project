execute_process(COMMAND "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/moveit_python/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/tetrisBot/106A-Project/build/moveit_python/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
