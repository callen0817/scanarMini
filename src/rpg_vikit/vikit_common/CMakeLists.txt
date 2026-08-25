cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME vikit_common)
project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
set(USE_ROS False) 

# --- Fix Architecture Flags ---
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    # x86 specific flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mmmx -msse -msse2 -msse3 -mssse3")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(aarch64)|(ARM64)|(arm)")
    # ARM specific flags (NEON is standard on aarch64)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
    add_definitions(-DVIKIT_ARM_NEON)
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -Wno-class-memaccess")

# Packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# Source files
list(APPEND SOURCEFILES 
    src/atan_camera.cpp
    src/omni_camera.cpp
    src/math_utils.cpp
    src/vision.cpp
    src/performance_monitor.cpp
    src/robust_cost.cpp
    src/user_input_thread.cpp
    src/pinhole_camera.cpp
    src/equidistant_camera.cpp
    src/polynomial_camera.cpp
    src/homography.cpp
    src/img_align.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    fmt::fmt
)

# Tests
add_executable(test_vk_common_camera test/test_camera.cpp)
target_link_libraries(test_vk_common_camera ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
target_link_libraries(test_vk_common_triangulation ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
target_link_libraries(test_vk_common_patch_score ${PROJECT_NAME} ${OpenCV_LIBS})

# Installation and Config
if(NOT USE_ROS)
    set(vikit_common_LIBRARIES ${PROJECT_NAME})
    set(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR})
    set(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
    
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE)
    export(PACKAGE vikit_common)

    install(DIRECTORY include/ DESTINATION include)
    install(TARGETS ${PROJECT_NAME} DESTINATION lib)
endif()