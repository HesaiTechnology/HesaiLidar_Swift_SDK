cmake_minimum_required(VERSION 2.8.3)
project(PandarSwiftSDK)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( Boost REQUIRED )
find_package( PCL REQUIRED COMPONENTS common )
find_package(OpenSSL REQUIRED)
 
SET(CMAKE_BUILD_TYPE "Release") 

include_directories(
    .
    include
    src/taskflow
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
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
    ${OPENSSL_LIBRARIES}
    pcap
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(PCL REQUIRED)
    add_executable(PandarSwiftTest
        test/test.cc
    )

    target_link_libraries(PandarSwiftTest
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
    )
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})