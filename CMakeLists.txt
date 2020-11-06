cmake_minimum_required(VERSION 2.8.3)
project(pandar128sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( Boost REQUIRED )
find_package( PCL REQUIRED COMPONENTS common )
 
SET(CMAKE_BUILD_TYPE "Release") 

include_directories(
    .
    include
    src/taskflow
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} SHARED
    src/input.cc
    src/laser_ts.cpp
    src/pandar128Driver.cc
    src/pandar128SDK.cc
    src/platUtil.cc
    src/tcp_command_client.c
    src/util.c
)

target_link_libraries(${PROJECT_NAME} 
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    pcap
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(PCL REQUIRED)
    add_executable(pandar128sdkTest
        test/test.cc
    )

    target_link_libraries(pandar128sdkTest
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
    )
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})