cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})
add_executable (volume_estimation_pcd volume_estimation_pcd.cpp)
target_link_libraries(volume_estimation_pcd ${PCL_LIBRARIES})
add_executable (volume_estimation_ply volume_estimation_ply.cpp)
target_link_libraries(volume_estimation_ply ${PCL_LIBRARIES})
add_executable (volume_estimation_obj ${SOURCES_FILES_LIBS} volume_estimation_obj.cpp)
target_link_libraries(volume_estimation_obj ${PCL_LIBRARIES})

add_executable (visualize_partial_view ${SOURCES_FILES_LIBS} visualize_partial_view.cpp)
target_link_libraries(visualize_partial_view ${PCL_LIBRARIES})

add_executable (center_of_mass_estimation_obj ${SOURCES_FILES_LIBS} center_of_mass_estimation_obj.cpp)
target_link_libraries(center_of_mass_estimation_obj ${PCL_LIBRARIES})
add_executable (center_of_mass_estimation_pcd ${SOURCES_FILES_LIBS} center_of_mass_estimation_pcd.cpp)
target_link_libraries(center_of_mass_estimation_pcd ${PCL_LIBRARIES})