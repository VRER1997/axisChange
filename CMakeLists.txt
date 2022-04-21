project(change)
cmake_minimum_required(VERSION 3.5)

find_package(Eigen3 REQUIRED)

add_executable(change src/change.cpp)
target_link_libraries(change Eigen3::Eigen)