cmake_minimum_required(VERSION 3.0.2)
project(ros_sharp_time_synchronizer)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
  COMPONENTS
    image_transport
    roscpp
    cv_bridge
    sensor_msgs
)

find_package(Boost REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
)

add_executable(convert src/convert.cpp)
target_link_libraries(convert ${catkin_LIBRARIES} )

