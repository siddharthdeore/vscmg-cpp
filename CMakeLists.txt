cmake_minimum_required(VERSION 3.14)
project(VSCMG LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(include)
 
add_library (ADCS  src/VSCMG.cpp include/RigidBody.h)
target_link_libraries (ADCS Eigen3::Eigen)

add_executable (TestVSCMG  examples/vscmg.cpp )
target_link_libraries (TestVSCMG ADCS)

add_executable (TestRigidBody  examples/rigid_body.cpp )
target_link_libraries (TestRigidBody ADCS)    



