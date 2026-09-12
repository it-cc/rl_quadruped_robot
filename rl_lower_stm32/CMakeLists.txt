cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the build type
# if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
# endif()

# Set the project name
set(CMAKE_PROJECT_NAME quadruped_robot)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Collect user sources from User/src
file(GLOB_RECURSE USER_CPP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User/src/controller/*.cpp
    ${CMAKE_SOURCE_DIR}/User/src/interface/*.cpp
)

file(GLOB_RECURSE USER_C_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User/src/interface/*.c
)

file(GLOB_RECURSE QUADRUPED_CPP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Quadruped_Api/Src/*.cpp
    ${CMAKE_SOURCE_DIR}/Quadruped_Hardware/Src/*.cpp
    ${CMAKE_SOURCE_DIR}/Quadruped_RL_Controller/Src/*.cpp
    ${CMAKE_SOURCE_DIR}/Quadruped_User/Src/*.cpp
    ${CMAKE_SOURCE_DIR}/Quadruped_Test/Src/*.cpp
)

# micro-ROS transport and support files
set(MICROROS_SOURCES
    ${CMAKE_SOURCE_DIR}/Quadruped_Library/micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
    ${CMAKE_SOURCE_DIR}/Quadruped_Library/micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
    ${CMAKE_SOURCE_DIR}/Quadruped_Library/micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
    ${CMAKE_SOURCE_DIR}/Quadruped_Library/micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_CPP_SOURCES}
    ${USER_C_SOURCES}
    ${QUADRUPED_CPP_SOURCES}
    ${MICROROS_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/User/include
    ${CMAKE_SOURCE_DIR}/Quadruped_Api/Inc
    ${CMAKE_SOURCE_DIR}/Quadruped_Hardware/Inc
    ${CMAKE_SOURCE_DIR}/Quadruped_RL_Controller/Inc
    ${CMAKE_SOURCE_DIR}/Quadruped_Library
    ${CMAKE_SOURCE_DIR}/Quadruped_User/Inc
    ${CMAKE_SOURCE_DIR}/Quadruped_Test/Inc
    # micro-ROS include paths
    ${CMAKE_SOURCE_DIR}/Quadruped_Library/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:DEBUG_UART>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # micro-ROS static library
    ${CMAKE_SOURCE_DIR}/Quadruped_Library/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a
    # Add user defined libraries
)
