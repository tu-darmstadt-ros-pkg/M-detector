cmake_minimum_required(VERSION 3.16)
project(m_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(TBB REQUIRED)
find_package(OpenMP QUIET)

find_package(OpenCV REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Development)

message(STATUS "Eigen include: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "PCL include:   ${PCL_INCLUDE_DIRS}")
message(STATUS "OpenCV ver:    ${OpenCV_VERSION}")
message(STATUS "Python3 inc:   ${Python3_INCLUDE_DIRS}")

include(ProcessorCount)
ProcessorCount(N_CORES)
if(NOT N_CORES)
  set(N_CORES 1)
endif()

set(MP_PROC_NUM 1)
if(N_CORES GREATER 5)
  set(MP_PROC_NUM 4)
elseif(N_CORES GREATER 3)
  math(EXPR MP_PROC_NUM "${N_CORES} - 2")
endif()

set(MDETECTOR_COMMON_SRCS
  src/DynObjFilter.cpp
  src/DynObjCluster.cpp
)

add_executable(dynfilter
  src/dynfilter_with_odom.cpp
  ${MDETECTOR_COMMON_SRCS}
)

add_executable(display_prediction
  src/display_prediction.cpp
)

add_executable(cal_recall
  src/cal_recall_multi.cpp
)

set(PROJ_PUBLIC_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

foreach(tgt dynfilter display_prediction cal_recall)
  target_include_directories(${tgt}
    PRIVATE
      ${PROJ_PUBLIC_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
      ${Python3_INCLUDE_DIRS}
  )

  if(MP_PROC_NUM GREATER 1)
    target_compile_definitions(${tgt} PRIVATE MP_EN MP_PROC_NUM=${MP_PROC_NUM})
  else()
    target_compile_definitions(${tgt} PRIVATE MP_PROC_NUM=1)
  endif()

  ament_target_dependencies(${tgt}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    pcl_conversions
    livox_ros_driver2
  )

  target_link_libraries(${tgt}
    TBB::tbb
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Python3::Python
  )

  if(OpenMP_CXX_FOUND)
    target_link_libraries(${tgt} OpenMP::OpenMP_CXX)
  endif()

  target_compile_options(${tgt} PRIVATE -fexceptions)
endforeach()

install(TARGETS
  dynfilter
  display_prediction
  cal_recall
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
