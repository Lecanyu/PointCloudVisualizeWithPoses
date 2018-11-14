cmake_minimum_required(VERSION 3.3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(PointCloudVisualizeWithPoses)

find_package(PCL 1.8 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(ShowAppN2M ShowAppN2M.cpp)
add_executable(ShowMatPose ShowMatPose.cpp)
add_executable(ShowQuadPose ShowQuadPose.cpp)
target_link_libraries(ShowAppN2M ${PCL_LIBRARIES})
target_link_libraries(ShowMatPose ${PCL_LIBRARIES})
target_link_libraries(ShowQuadPose ${PCL_LIBRARIES})