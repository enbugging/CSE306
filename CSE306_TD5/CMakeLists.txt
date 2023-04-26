project(demo)
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-fopenmp -O3 -fcf-protection=none -march=native -mtune=native -fno-math-errno)
add_link_options(-fopenmp -O3 -fcf-protection=none -march=native -mtune=native -fno-math-errno)
add_executable(demo main.cpp)