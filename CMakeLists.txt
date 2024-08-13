cmake_minimum_required(VERSION 3.5 FATAL_ERROR)


project(radius_or_conditional_removal)


find_package(PCL 1.2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


#add_executable (pcd_write pcd_write.cpp)
add_executable (radius_or_conditional_removal RadiusOutlier_or_Conditional_removal.cpp)

target_link_libraries (radius_or_conditional_removal ${PCL_LIBRARIES})