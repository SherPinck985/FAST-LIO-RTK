# ROS 1/catkin 构建入口：核心目标只有 fastlio_mapping。
# IMU 与 IKFoM 主要通过头文件进入编译，ikd-tree 为直接编译的子模块源文件。

cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

# 当前同时设置 Debug 与 -O3；调试/性能分析应看最终编译参数，而非只看构建类型。
SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# 将源码包目录编译到程序中，Log/ 和 PCD/ 的输出位置由它决定。
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

# 根据 CPU 架构/核数选择观测匹配的 OpenMP 线程配置，与树内部 pthread 重建独立。
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# 保留的 Python 开发依赖；当前主流程没有调用 matplotlibcpp 绘图但仍包含/链接 Python。
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS 通信、消息与驱动依赖；即便使用标准 PointCloud2，当前源码仍依赖 Livox 头文件。
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

# 经纬高到局部 ENU 的转换仍使用成熟的地理坐标库。
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/LocalCartesian.hpp)
find_library(GEOGRAPHICLIB_LIBRARY NAMES Geographic GeographicLib)
if(NOT GEOGRAPHICLIB_INCLUDE_DIR OR NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR "Install libgeographic-dev for RTK coordinate conversion")
endif()

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)
include_directories(${GEOGRAPHICLIB_INCLUDE_DIR})

# 生成 fast_lio/Pose6D.h，供 IMU 帧内轨迹记录使用；无需手工添加生成的头文件。
add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

# 主程序 + 雷达预处理 + 地图树实现；其余算法实现通过 include 进入。
add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
add_dependencies(fastlio_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${GEOGRAPHICLIB_LIBRARY})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
