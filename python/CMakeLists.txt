# python/CMakeLists.txt
cmake_minimum_required(VERSION 3.18)

project(roboflex_dynamixel_ext)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# If you don't do this, then the pybind11_add_module will think it's
# standalone and will not link correctly.
set(PYBIND11_CPP_STANDARD -std=c++20)


# -------------------- 
# Resolve dependencies

find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)

include(FetchContent)

# download and build pybind11
FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.11.1
)
FetchContent_MakeAvailable(pybind11)


# -------------------- 
# Define the library

# Use the pybind11 provided function to create a module.
pybind11_add_module(roboflex_dynamixel_ext
    pybindings.cpp
)

# Link against your the library and any necessary dependencies
target_link_libraries(roboflex_dynamixel_ext PUBLIC 
    roboflex_dynamixel
)

# If you have specific compile definitions or options for just the Python module
# target_compile_definitions(pyroboflex PRIVATE SOME_DEFINITION)

# You can set properties for the target if necessary.
set_target_properties(roboflex_dynamixel_ext PROPERTIES
    POSITION_INDEPENDENT_CODE ON

    # # We want to use an explicit RPATH, so we can
    # # find libuvc.so where we installed it.
    # BUILD_WITH_INSTALL_RPATH FALSE
    # INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    # INSTALL_RPATH_USE_LINK_PATH TRUE
)


# -------------------- 
# install

# Install the generated Python module to the desired destination.
# This installs the compiled module.
# install(TARGETS roboflex_webcam_uvc_ext
#     LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/roboflex/webcam_uvc
# )

# # Install the auxiliary Python files
# install(FILES
#     __init__.py
#     DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/roboflex/webcam_uvc
# )
