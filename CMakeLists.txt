cmake_minimum_required(VERSION 3.5)

project(limx_robot_sdk)

set(CMAKE_BUILD_TYPE "Release")

include_directories(${PROJECT_SOURCE_DIR}/include)

if (DEFINED CATKIN_DEVEL_PREFIX)
  # In ROS compilation environment
  message("Currently in ROS compilation environment")
  
  find_package(catkin REQUIRED COMPONENTS 
    roscpp
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp
  )

  include_directories(${catkin_INCLUDE_DIRS})

  add_custom_target(pointfoot_lowlevel_include_target ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/include/. ${CATKIN_DEVEL_PREFIX}/include
  )

  if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
    add_custom_target(pointfoot_lowlevel_target ALL
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/liblimx_robot_sdk.so ${CATKIN_DEVEL_PREFIX}/lib
    )
  elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
    add_custom_target(pointfoot_lowlevel_target ALL
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/liblimx_robot_sdk.so ${CATKIN_DEVEL_PREFIX}/lib
    )
  elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
    add_custom_target(pointfoot_lowlevel_target ALL
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/liblimx_robot_sdk.so ${CATKIN_DEVEL_PREFIX}/lib
    )
  elseif (WIN32)
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
    add_custom_target(pointfoot_lowlevel_target ALL
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/limx_robot_sdk.dll ${CATKIN_DEVEL_PREFIX}/lib
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/limx_robot_sdk.lib ${CATKIN_DEVEL_PREFIX}/lib
    )
  else()
    message(FATAL_ERROR "Unsupported architecture!")
  endif()

  link_directories( ${LIB_PATH} )

  install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  
  # Install library files to build output
  install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
else()
  if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
  elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
  elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
  elseif (WIN32)
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
    install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
  else()
    message(FATAL_ERROR "Unsupported architecture!")
  endif()

  link_directories(${LIB_PATH})

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX})

  # Install library files to build output
  install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()

option(EXAMPLE "Build examples ON/OFF" OFF)
if(EXAMPLE)
  add_subdirectory(examples)
endif()

