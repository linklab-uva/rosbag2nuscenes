cmake_minimum_required(VERSION 3.8)
project(rosbag2nuscenes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(delphi_esr_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

if (NOT TARGET yaml-cpp)
    add_subdirectory(yaml-cpp)
endif() 

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include/
)

file(GLOB_RECURSE SOURCE_FILES
    src/*.cpp
)

add_executable(${PROJECT_NAME} main.cpp ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME} sensor_msgs delphi_esr_msgs rosbag2_cpp)
