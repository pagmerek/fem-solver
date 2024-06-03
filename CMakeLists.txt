cmake_minimum_required(VERSION 3.14)
project(cufem)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Eigen3 REQUIRED)
link_directories(${Eigen_INCLUDE_DIRS})

add_subdirectory(src)

enable_testing()
add_subdirectory(test)


