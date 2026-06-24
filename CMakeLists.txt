cmake_minimum_required(VERSION 2.6)
project(wam_teleop)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(tinyxml2 REQUIRED)

find_path(NIDAQMX_INCLUDE_DIR
    NAMES NIDAQmx.h
    PATHS
        /usr/include
        /usr/local/include
        /usr/local/natinst/nidaqmx/include
)

find_library(NIDAQMX_LIBRARY
    NAMES nidaqmx
    PATHS
        /usr/lib
        /usr/local/lib
        /usr/local/natinst/nidaqmx/lib
)

if (NOT NIDAQMX_INCLUDE_DIR OR NOT NIDAQMX_LIBRARY)
    message(FATAL_ERROR "NI-DAQmx not found.")
endif()

include_directories(${NIDAQMX_INCLUDE_DIR})

add_executable(follower src/follower.cpp src/lib/udp_handler.cpp)
target_link_libraries(follower
    ${BARRETT_LIBRARIES}
    ${catkin_LIBRARIES}
    Boost::system
    Boost::thread
)

add_executable(follower_ft src/follower_ft.cpp src/lib/udp_handler_ft.cpp)
target_link_libraries(follower_ft
    ${BARRETT_LIBRARIES}
    ${catkin_LIBRARIES}
    Boost::system
    Boost::thread
)

add_executable(leader src/leader.cpp src/lib/udp_handler.cpp)
target_link_libraries(leader
    ${BARRETT_LIBRARIES}
    ${catkin_LIBRARIES}
    Boost::system
    Boost::thread
)

add_executable(leader_ft src/leader_ft.cpp src/lib/udp_handler_ft.cpp)
target_link_libraries(leader_ft
    ${BARRETT_LIBRARIES}
    ${catkin_LIBRARIES}
    Boost::system
    Boost::thread
    tinyxml2
    ${NIDAQMX_LIBRARY}
)

