cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})
add_executable (volume_estimation volume_estimation.cpp)
target_link_libraries(volume_estimation ${PCL_LIBRARIES})
add_executable (volume_estimation_ply volume_estimation_ply.cpp)
target_link_libraries(volume_estimation_ply ${PCL_LIBRARIES})