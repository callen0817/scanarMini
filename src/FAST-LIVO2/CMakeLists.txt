cmake_minimum_required(VERSION 3.10)
project(fast_livo)

set(CMAKE_BUILD_TYPE "Release")

# --- LIVOX GUARD ---
option(USE_LIVOX "Build with Livox Driver support" OFF)
if(USE_LIVOX)
    add_definitions(-DUSE_LIVOX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

IF("$ENV{ROS_DISTRO}" STRLESS "iron")
  add_definitions(-DPRE_ROS_IRON)
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
  add_definitions(-DARM_ARCH)
else()
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops")
  add_definitions(-DX86_ARCH)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# --- OPENMP FIND ---
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(OMP_LIBS OpenMP::OpenMP_CXX)
else()
    set(OMP_LIBS "")
endif()

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

if(USE_LIVOX)
    find_package(livox_ros_driver2 REQUIRED)
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${vikit_common_INCLUDE_DIRS}
  ${vikit_ros_INCLUDE_DIRS}
  include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vikit_common/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vikit_ros/lib
)

set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  vikit_common
  vikit_ros
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
)

if(USE_LIVOX)
    list(APPEND dependencies livox_ros_driver2)
endif()

# Libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)
add_library(utils src/utils.cpp)

ament_target_dependencies(vio ${dependencies})
ament_target_dependencies(lio ${dependencies})
ament_target_dependencies(pre ${dependencies})
ament_target_dependencies(imu_proc ${dependencies})
ament_target_dependencies(laser_mapping ${dependencies})

# --- WORKAROUND FOR UNDEFINED REFERENCE ---
# We point directly to the shared objects to ensure the linker 
# pulls in every symbol available in the vikit build.
set(VIKIT_LIBS 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vikit_ros/lib/libvikit_ros.so
    ${CMAKE_CURRENT_SOURCE_DIR}/../../install/vikit_common/lib/libvikit_common.so
)

target_link_libraries(laser_mapping 
    ${VIKIT_LIBS}
    ${OMP_LIBS}
)

target_link_libraries(vio ${OMP_LIBS})
target_link_libraries(lio utils ${OMP_LIBS})
target_link_libraries(pre ${OMP_LIBS})
target_link_libraries(imu_proc ${OMP_LIBS})

# Main Executable
add_executable(fastlivo_mapping src/main.cpp)
ament_target_dependencies(fastlivo_mapping ${dependencies})

# Explicit Link for Executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${VIKIT_LIBS}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OMP_LIBS}
)

install(TARGETS fastlivo_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME})
ament_package()