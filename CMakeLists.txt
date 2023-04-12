project(demo)
cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-fopenmp -O3)
add_link_options(-fopenmp -O3)
add_executable(demo main.cpp)
target_include_directories(demo PRIVATE ./classes)