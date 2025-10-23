cmake_minimum_required(VERSION 3.5)

project(dynamixel_sdk)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DYNAMIXEL_SDK_SOURCES
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/packet_handler.cpp
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/protocol1_packet_handler.cpp
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/protocol2_packet_handler.cpp
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/group_sync_read.cpp
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/group_sync_write.cpp
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/group_bulk_read.cpp
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/group_bulk_write.cpp
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/port_handler.cpp
)

if(APPLE)
    list(APPEND DYNAMIXEL_SDK_SOURCES
        ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/port_handler_mac.cpp
    )
elseif(UNIX)
    list(APPEND DYNAMIXEL_SDK_SOURCES
        ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/port_handler_linux.cpp
    )
elseif(WIN32)
    list(APPEND DYNAMIXEL_SDK_SOURCES
        ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/src/dynamixel_sdk/port_handler_windows.cpp
    )
endif()

add_library(dynamixel_sdk ${DYNAMIXEL_SDK_SOURCES})

target_include_directories(dynamixel_sdk PUBLIC 
    ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk/c++/include/dynamixel_sdk
)
