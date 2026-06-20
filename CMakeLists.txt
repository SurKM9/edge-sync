cmake_minimum_required(VERSION 3.15...3.25)
project(edge_sync
    VERSION 1.0.0
    LANGUAGES CXX
)

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
