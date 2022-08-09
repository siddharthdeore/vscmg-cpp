cmake_minimum_required(VERSION 3.14)
project(VSCMG LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories("include")

file(GLOB ADCS_SRC
     "include/ADCS/Core/*.h"
)

add_library(ADCS SHARED ${ADCS_SRC})
target_link_libraries (ADCS Eigen3::Eigen)
set_target_properties(ADCS PROPERTIES LINKER_LANGUAGE CXX)


add_library (VSCMG SHARED
    src/ADCS/Systems/VSCMG.cpp)
target_link_libraries (VSCMG ADCS Eigen3::Eigen)

add_library (RigidBody SHARED
    src/ADCS/Systems/RigidBody.cpp include/ADCS/Systems/RigidBody.h)
target_link_libraries (RigidBody ADCS Eigen3::Eigen)



option(BUILD_EXAMPLES "Build Example " ON)
if(${BUILD_EXAMPLES})
    add_executable (TestVSCMG  examples/vscmg.cpp )
    target_link_libraries (TestVSCMG VSCMG)

    add_executable (TestRigidBody  examples/rigid_body.cpp )
    target_link_libraries (TestRigidBody RigidBody)    
endif()


