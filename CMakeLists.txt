cmake_minimum_required(VERSION 3.14)
project(VSCMG LANGUAGES CXX)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(include)
 
add_executable (VSCMG main.cpp src/VSCMG.cpp)
target_link_libraries (VSCMG Eigen3::Eigen)

