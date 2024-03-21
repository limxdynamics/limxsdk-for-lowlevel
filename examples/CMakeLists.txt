cmake_minimum_required(VERSION 3.5)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install/examples)

find_package(Eigen3 REQUIRED)

include_directories(./)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SRCS
  ./common/util.cpp
  ./wl_controller_base.cpp
  ./pf_controller_base.cpp
)

if (WIN32)
  set(LINK_LIBS limx_robot_sdk common shlwapi)
else()
  set(LINK_LIBS limx_robot_sdk common pthread)
endif()

add_library(common ${SRCS})

add_executable(wl_template wl_template.cpp)
target_link_libraries(wl_template limx_robot_sdk)
install(TARGETS wl_template DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

add_executable(wl_zero_torque wl_zero_torque.cpp)
target_link_libraries(wl_zero_torque ${LINK_LIBS})
install(TARGETS wl_zero_torque DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

add_executable(wl_standup wl_standup.cpp)
target_link_libraries(wl_standup ${LINK_LIBS})
install(TARGETS wl_standup DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

add_executable(wl_sitdown wl_sitdown.cpp)
target_link_libraries(wl_sitdown ${LINK_LIBS})
install(TARGETS wl_sitdown DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

add_executable(pf_joint_move pf_joint_move.cpp)
target_link_libraries(pf_joint_move ${LINK_LIBS})
install(TARGETS pf_joint_move DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

add_executable(pf_groupJoints_move pf_groupJoints_move.cpp)
target_link_libraries(pf_groupJoints_move ${LINK_LIBS})
install(TARGETS pf_groupJoints_move DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

