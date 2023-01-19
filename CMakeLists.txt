cmake_minimum_required(VERSION 3.22)

project(lidar_point_pillars)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB SRCS *.cpp)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(.)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} STATIC ${SRCS})

add_executable(${PROJECT_NAME}_test test/main.cpp)
add_dependencies(${PROJECT_NAME}_test ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${OpenCV_LIBS})