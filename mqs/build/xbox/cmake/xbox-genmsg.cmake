# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xbox: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ixbox:/home/lab/mqs/src/xbox/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xbox_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/op.msg" NAME_WE)
add_custom_target(_xbox_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xbox" "/home/lab/mqs/src/xbox/msg/op.msg" ""
)

get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/marine.msg" NAME_WE)
add_custom_target(_xbox_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xbox" "/home/lab/mqs/src/xbox/msg/marine.msg" ""
)

get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/land.msg" NAME_WE)
add_custom_target(_xbox_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xbox" "/home/lab/mqs/src/xbox/msg/land.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(xbox
  "/home/lab/mqs/src/xbox/msg/op.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xbox
)
_generate_msg_cpp(xbox
  "/home/lab/mqs/src/xbox/msg/marine.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xbox
)
_generate_msg_cpp(xbox
  "/home/lab/mqs/src/xbox/msg/land.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xbox
)

### Generating Services

### Generating Module File
_generate_module_cpp(xbox
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xbox
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xbox_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xbox_generate_messages xbox_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/op.msg" NAME_WE)
add_dependencies(xbox_generate_messages_cpp _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/marine.msg" NAME_WE)
add_dependencies(xbox_generate_messages_cpp _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/land.msg" NAME_WE)
add_dependencies(xbox_generate_messages_cpp _xbox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xbox_gencpp)
add_dependencies(xbox_gencpp xbox_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xbox_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(xbox
  "/home/lab/mqs/src/xbox/msg/op.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xbox
)
_generate_msg_eus(xbox
  "/home/lab/mqs/src/xbox/msg/marine.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xbox
)
_generate_msg_eus(xbox
  "/home/lab/mqs/src/xbox/msg/land.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xbox
)

### Generating Services

### Generating Module File
_generate_module_eus(xbox
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xbox
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xbox_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xbox_generate_messages xbox_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/op.msg" NAME_WE)
add_dependencies(xbox_generate_messages_eus _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/marine.msg" NAME_WE)
add_dependencies(xbox_generate_messages_eus _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/land.msg" NAME_WE)
add_dependencies(xbox_generate_messages_eus _xbox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xbox_geneus)
add_dependencies(xbox_geneus xbox_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xbox_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(xbox
  "/home/lab/mqs/src/xbox/msg/op.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xbox
)
_generate_msg_lisp(xbox
  "/home/lab/mqs/src/xbox/msg/marine.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xbox
)
_generate_msg_lisp(xbox
  "/home/lab/mqs/src/xbox/msg/land.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xbox
)

### Generating Services

### Generating Module File
_generate_module_lisp(xbox
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xbox
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xbox_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xbox_generate_messages xbox_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/op.msg" NAME_WE)
add_dependencies(xbox_generate_messages_lisp _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/marine.msg" NAME_WE)
add_dependencies(xbox_generate_messages_lisp _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/land.msg" NAME_WE)
add_dependencies(xbox_generate_messages_lisp _xbox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xbox_genlisp)
add_dependencies(xbox_genlisp xbox_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xbox_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(xbox
  "/home/lab/mqs/src/xbox/msg/op.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xbox
)
_generate_msg_nodejs(xbox
  "/home/lab/mqs/src/xbox/msg/marine.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xbox
)
_generate_msg_nodejs(xbox
  "/home/lab/mqs/src/xbox/msg/land.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xbox
)

### Generating Services

### Generating Module File
_generate_module_nodejs(xbox
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xbox
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xbox_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xbox_generate_messages xbox_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/op.msg" NAME_WE)
add_dependencies(xbox_generate_messages_nodejs _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/marine.msg" NAME_WE)
add_dependencies(xbox_generate_messages_nodejs _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/land.msg" NAME_WE)
add_dependencies(xbox_generate_messages_nodejs _xbox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xbox_gennodejs)
add_dependencies(xbox_gennodejs xbox_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xbox_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(xbox
  "/home/lab/mqs/src/xbox/msg/op.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xbox
)
_generate_msg_py(xbox
  "/home/lab/mqs/src/xbox/msg/marine.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xbox
)
_generate_msg_py(xbox
  "/home/lab/mqs/src/xbox/msg/land.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xbox
)

### Generating Services

### Generating Module File
_generate_module_py(xbox
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xbox
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xbox_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xbox_generate_messages xbox_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/op.msg" NAME_WE)
add_dependencies(xbox_generate_messages_py _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/marine.msg" NAME_WE)
add_dependencies(xbox_generate_messages_py _xbox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/mqs/src/xbox/msg/land.msg" NAME_WE)
add_dependencies(xbox_generate_messages_py _xbox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xbox_genpy)
add_dependencies(xbox_genpy xbox_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xbox_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xbox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xbox
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xbox_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xbox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xbox
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xbox_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xbox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xbox
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xbox_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xbox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xbox
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xbox_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xbox)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xbox\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xbox
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xbox_generate_messages_py std_msgs_generate_messages_py)
endif()
