cmake_minimum_required(VERSION 3.5)

project(limx_robot_sdk)

set(CMAKE_BUILD_TYPE "Release")

option(EXAMPLE "Build examples ON/OFF" OFF)

include_directories(${PROJECT_SOURCE_DIR}/include)

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

if(EXAMPLE)
  add_subdirectory(examples)
endif()
