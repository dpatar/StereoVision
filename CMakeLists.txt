cmake_minimum_required(VERSION 3.16.3)

project(StereoVision)

set(OpenCV_DIR /home/dpatar/Library/build)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} src/main.cpp)
# MyLib
add_subdirectory(StereoLib)
target_include_directories(${PROJECT_NAME} PUBLIC StereoLib)

target_link_directories(${PROJECT_NAME} PUBLIC StereoLib)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} StereoLib)