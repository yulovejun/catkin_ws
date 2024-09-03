# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vi_grab: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ivi_grab:/home/rm/catkin_ws/src/vi_grab/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vi_grab_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg" NAME_WE)
add_custom_target(_vi_grab_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vi_grab" "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vi_grab
  "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vi_grab
)

### Generating Services

### Generating Module File
_generate_module_cpp(vi_grab
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vi_grab
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vi_grab_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vi_grab_generate_messages vi_grab_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(vi_grab_generate_messages_cpp _vi_grab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vi_grab_gencpp)
add_dependencies(vi_grab_gencpp vi_grab_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vi_grab_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(vi_grab
  "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vi_grab
)

### Generating Services

### Generating Module File
_generate_module_eus(vi_grab
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vi_grab
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(vi_grab_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(vi_grab_generate_messages vi_grab_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(vi_grab_generate_messages_eus _vi_grab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vi_grab_geneus)
add_dependencies(vi_grab_geneus vi_grab_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vi_grab_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vi_grab
  "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vi_grab
)

### Generating Services

### Generating Module File
_generate_module_lisp(vi_grab
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vi_grab
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vi_grab_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vi_grab_generate_messages vi_grab_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(vi_grab_generate_messages_lisp _vi_grab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vi_grab_genlisp)
add_dependencies(vi_grab_genlisp vi_grab_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vi_grab_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(vi_grab
  "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vi_grab
)

### Generating Services

### Generating Module File
_generate_module_nodejs(vi_grab
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vi_grab
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(vi_grab_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(vi_grab_generate_messages vi_grab_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(vi_grab_generate_messages_nodejs _vi_grab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vi_grab_gennodejs)
add_dependencies(vi_grab_gennodejs vi_grab_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vi_grab_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vi_grab
  "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vi_grab
)

### Generating Services

### Generating Module File
_generate_module_py(vi_grab
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vi_grab
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vi_grab_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vi_grab_generate_messages vi_grab_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rm/catkin_ws/src/vi_grab/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(vi_grab_generate_messages_py _vi_grab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vi_grab_genpy)
add_dependencies(vi_grab_genpy vi_grab_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vi_grab_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vi_grab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vi_grab
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(vi_grab_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vi_grab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vi_grab
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(vi_grab_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vi_grab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vi_grab
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(vi_grab_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vi_grab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vi_grab
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(vi_grab_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vi_grab)
  install(CODE "execute_process(COMMAND \"/home/rm/anaconda3/envs/grasp/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vi_grab\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vi_grab
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(vi_grab_generate_messages_py std_msgs_generate_messages_py)
endif()
