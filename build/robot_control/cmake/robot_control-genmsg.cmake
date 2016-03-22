# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_control: 15 messages, 3 services")

set(MSG_I_FLAGS "-Irobot_control:/home/rafael/SORS_Application/src/robot_control/msg;-Irobot_control:/home/rafael/SORS_Application/devel/share/robot_control/msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:robot_control/searchGoal"
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg" "robot_control/createMapFeedback:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg" "robot_control/searchActionFeedback:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:robot_control/searchActionResult:robot_control/searchResult:std_msgs/Header:robot_control/searchFeedback:robot_control/searchActionGoal:robot_control/searchGoal"
)

get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg" "robot_control/createMapResult:robot_control/createMapActionGoal:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:robot_control/createMapActionResult:robot_control/createMapFeedback:robot_control/createMapGoal:robot_control/createMapActionFeedback"
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg" "robot_control/createMapResult:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:robot_control/searchFeedback"
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg" "robot_control/createMapGoal:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg" ""
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg" "actionlib_msgs/GoalStatus:robot_control/searchResult:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg" NAME_WE)
add_custom_target(_robot_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_control" "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_msg_cpp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)

### Generating Services
_generate_srv_cpp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_srv_cpp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)
_generate_srv_cpp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
)

### Generating Module File
_generate_module_cpp(robot_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_control_generate_messages robot_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_cpp _robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_control_gencpp)
add_dependencies(robot_control_gencpp robot_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_control_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_msg_lisp(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)

### Generating Services
_generate_srv_lisp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_srv_lisp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)
_generate_srv_lisp(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
)

### Generating Module File
_generate_module_lisp(robot_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_control_generate_messages robot_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_lisp _robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_control_genlisp)
add_dependencies(robot_control_genlisp robot_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_control_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_msg_py(robot_control
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)

### Generating Services
_generate_srv_py(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_srv_py(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)
_generate_srv_py(robot_control
  "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
)

### Generating Module File
_generate_module_py(robot_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_control_generate_messages robot_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/getPositions.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/msg/laserMeasures.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/addToMap.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchAction.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/src/robot_control/srv/defineGlobalPath.srv" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapAction.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapActionGoal.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/searchActionResult.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafael/SORS_Application/devel/share/robot_control/msg/createMapFeedback.msg" NAME_WE)
add_dependencies(robot_control_generate_messages_py _robot_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_control_genpy)
add_dependencies(robot_control_genpy robot_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(robot_control_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(robot_control_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(robot_control_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(robot_control_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(robot_control_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(robot_control_generate_messages_py std_msgs_generate_messages_py)
