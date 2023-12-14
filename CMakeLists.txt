cmake_minimum_required(VERSION 3.18)

project(roboflex_dynamixel)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# -------------------- 
# Resolve dependencies

# Include FetchContent Module
include(FetchContent)

# Fetch and make dynamixel available
FetchContent_Declare(dynamixel_sdk
    # for some reason, we sometimes have a problem on arm
    #GIT_REPOSITORY https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    #GIT_TAG        3.7.51
    URL         https://github.com/ROBOTIS-GIT/DynamixelSDK/archive/refs/tags/3.7.51.tar.gz
    SOURCE_DIR  "${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk"
)
FetchContent_GetProperties(dynamixel_sdk)
if(NOT dynamixel_sdk_POPULATED)
  FetchContent_Populate(dynamixel_sdk)
  
  # This will add your custom CMakeLists.txt to the current build
  add_subdirectory(${CMAKE_SOURCE_DIR}/cmake_dynamixel ${CMAKE_BINARY_DIR}/_deps/dynamixel_sdk-build)
endif()

# download and build roboflex_core
FetchContent_Declare(roboflex_core
    GIT_REPOSITORY https://github.com/flexrobotics/roboflex.git
    GIT_TAG        main
)
set(BUILD_ROBOFLEX_PYTHON_EXT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(roboflex_core)


# -------------------- 
# Define the library

add_library(roboflex_dynamixel STATIC
    src/dynamixel_controller.cpp
    src/dynamixel.cpp
    include/roboflex_dynamixel/dynamixel_controller.h
    include/roboflex_dynamixel/dynamixel.h
)

# Set some properties on our library
set_property(TARGET roboflex_dynamixel PROPERTY 
    POSITION_INDEPENDENT_CODE ON
)

# Include directories when we compile our library
target_include_directories(roboflex_dynamixel PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
    ${dynamixel_sdk_SOURCE_DIR}/c++/include
)

# Link against the necessary libraries
target_link_libraries(roboflex_dynamixel PUBLIC 
    roboflex_core 
    dynamixel_sdk
)


# -------------------- 
# Examples

# test_group_node_cpp example
add_executable(test_group_node_cpp examples/test_group_node_cpp.cpp)
target_link_libraries(test_group_node_cpp PRIVATE roboflex_dynamixel)


# -------------------- 
# install

# If you need to install the dynamixel library
install(TARGETS roboflex_dynamixel 
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
#install(FILES dynamixel.h DESTINATION include/roboflex_dynamixel)
install(DIRECTORY include/roboflex_dynamixel
    DESTINATION include
)


# --------------------
# build python bindings

add_subdirectory(python)


# Not sure when we should do ExternalProject_Add vs find_package:
#
# include(ExternalProject)
# ExternalProject_Add(
#     roboflex_core
#     PREFIX ${CMAKE_BINARY_DIR}/_deps
#     SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../roboflex_core
#     CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
#     #CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_SOURCE_DIR}/../roboflex_core
#     #CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_SOURCE_DIR}/../roboflex_core/build/output_tst
# )

# # Get the source directory of the external project roboflex_core
# ExternalProject_Get_Property(roboflex_core SOURCE_DIR)
