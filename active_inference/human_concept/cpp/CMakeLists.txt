cmake_minimum_required(VERSION 3.16)
project(human_concept_core CXX)

# Standalone, dependency-light core: Eigen only (no torch / DSR / Qt).
# Keep flags clean (no -march=native, no -DEIGEN_MAX_ALIGN_BYTES=0) so this links
# cleanly into the libdsr-based agent later — see ../../CLAUDE.md.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3 -Wall)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(human_core STATIC
    core/human_kinematic_model.cpp
    core/vfe_inference.cpp)
target_include_directories(human_core PUBLIC core)
target_link_libraries(human_core PUBLIC Eigen3::Eigen)
