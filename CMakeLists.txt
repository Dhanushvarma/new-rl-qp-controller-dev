cmake_minimum_required(VERSION 3.22)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME NewRLQPController)
set(PROJECT_DESCRIPTION "NewRLQPController")
set(PROJECT_URL "")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
find_package(mc_joystick_plugin REQUIRED)

# Reference motions of the tracking policies (motions/*.csv), installed next to
# the policies. A tracking slot lists this directory unless its tracking
# section sets motions_dir. Set before src/: the controller is built with it as
# the default.
set(RL_MOTIONS_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/motions")

add_subdirectory(src)

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
if(BUILD_TESTING)
  add_subdirectory(tests/tracking)
endif()

# Bundled policies, installed so the configured yaml can reference them with
# a machine-independent path. The whole policy/ tree (e.g.
# policy/tracking/P6_final/policy.onnx), read when the install runs, so a newly
# added policy is never left out of the install tree.
set(RL_POLICY_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/policy")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/policy/" DESTINATION "${RL_POLICY_INSTALL_PREFIX}"
        FILES_MATCHING PATTERN "*.onnx")

# Read when the install runs: a motion added to motions/ needs no reconfigure.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/motions/" DESTINATION "${RL_MOTIONS_INSTALL_PREFIX}"
        FILES_MATCHING PATTERN "*.csv")

configure_file(etc/NewRLQPController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/NewRLQPController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/NewRLQPController.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
