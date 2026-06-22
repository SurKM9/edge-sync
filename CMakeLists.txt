cmake_minimum_required(VERSION 3.15...3.25)
project(edge_sync
    VERSION 1.0.0
    LANGUAGES CXX
)

# Tells CMake to export the exact compiler flags and include paths into a JSON file for the IDE
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Force out-of-source builds to reduce clutter in source directories
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "In-source builds are forbidden. Please create a build directory")
endif()

## Set global C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## modules for compiler warnings and sanitizers
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CompilerWarnings)

## top-level header only or interface libraries
add_library(edge_sync_headers INTERFACE)
target_include_directories(edge_sync_headers INTERFACE
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
)

# NO_MODULE tells CMake to look for the modern Eigen3Config.cmake file
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# By linking Eigen3::Eigen as an INTERFACE, any file that includes our 
# edge_sync_headers will automatically get the correct -I paths for Eigen.
target_link_libraries(edge_sync_headers INTERFACE Eigen3::Eigen)

## Add subdirectories
#add_subdirectory(src/core)
#add_subdirectory(src/concurrency)
#add_subdirectory(src/ingestion)
#add_subdirectory(src/sync)
#add_subdirectory(src/fusion)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    enable_testing()
    #add_subdirectory(tests)
endif()

# Explicitly tell CMake to compile our new sync_engine.cpp file
add_library(edge_sync_core 
    src/sync/sync_engine.cpp
    src/fusion/complementary_filter.cpp
)

# Link our headers (and Eigen) to this newly compiled code
target_link_libraries(edge_sync_core PUBLIC edge_sync_headers)

# Create the main program
add_executable(edge_sync_node src/main.cpp)

# Link our core library (which includes the sync and fusion modules)
target_link_libraries(edge_sync_node PRIVATE edge_sync_core)