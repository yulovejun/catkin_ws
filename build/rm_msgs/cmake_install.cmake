# Install script for directory: /home/rm/catkin_ws/src/rm_65_robot/rm_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rm/catkin_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rm/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rm/catkin_ws/install" TYPE PROGRAM FILES "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rm/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rm/catkin_ws/install" TYPE PROGRAM FILES "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rm/catkin_ws/install/setup.bash;/home/rm/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rm/catkin_ws/install" TYPE FILE FILES
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/setup.bash"
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rm/catkin_ws/install/setup.sh;/home/rm/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rm/catkin_ws/install" TYPE FILE FILES
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/setup.sh"
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rm/catkin_ws/install/setup.zsh;/home/rm/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rm/catkin_ws/install" TYPE FILE FILES
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/setup.zsh"
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rm/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rm/catkin_ws/install" TYPE FILE FILES "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/msg" TYPE FILE FILES
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Arm_Analog_Output.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Arm_Digital_Output.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Arm_IO_State.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/JointPos.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/MoveC.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/MoveJ.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/MoveJ_P.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/MoveL.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Tool_Analog_Output.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Tool_Digital_Output.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Tool_IO_State.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Plan_State.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/ChangeTool_State.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/ChangeTool_Name.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/ChangeWorkFrame_State.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/ChangeWorkFrame_Name.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Arm_Current_State.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/GetArmState_Command.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Stop.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Joint_Teach.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Pos_Teach.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Ort_Teach.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Stop_Teach.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Gripper_Set.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Gripper_Pick.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Joint_Enable.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/IO_Update.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Turtle_Driver.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Socket_Command.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Start_Multi_Drag_Teach.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Set_Force_Position.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Force_Position_Move_Joint.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Force_Position_Move_Pose.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Force_Position_State.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Six_Force.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Manual_Set_Force_Pose.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/CartePos.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Lift_Height.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Lift_Speed.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Joint_Current.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Joint_Step.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/ArmState.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Hand_Posture.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Hand_Seq.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Hand_Speed.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Hand_Force.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/Hand_Angle.msg"
    "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/msg/LiftState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/cmake" TYPE FILE FILES "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/rm_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/rm/catkin_ws/devel/.private/rm_msgs/include/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/rm/catkin_ws/devel/.private/rm_msgs/share/roseus/ros/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/rm/catkin_ws/devel/.private/rm_msgs/share/common-lisp/ros/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/rm/catkin_ws/devel/.private/rm_msgs/share/gennodejs/ros/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/rm/anaconda3/envs/grasp/bin/python3" -m compileall "/home/rm/catkin_ws/devel/.private/rm_msgs/lib/python3/dist-packages/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/rm/catkin_ws/devel/.private/rm_msgs/lib/python3/dist-packages/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/rm_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/cmake" TYPE FILE FILES "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/rm_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/cmake" TYPE FILE FILES
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/rm_msgsConfig.cmake"
    "/home/rm/catkin_ws/build/rm_msgs/catkin_generated/installspace/rm_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs" TYPE FILE FILES "/home/rm/catkin_ws/src/rm_65_robot/rm_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/rm/catkin_ws/build/rm_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/rm/catkin_ws/build/rm_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
