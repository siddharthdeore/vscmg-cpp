cmake_minimum_required(VERSION 3.14)
project(VSCMG LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories("include")
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB ADCS_SRC "include/ADCS/Core/*.h")

add_library(ADCS SHARED ${ADCS_SRC})
target_link_libraries (ADCS Eigen3::Eigen)
set_target_properties(ADCS PROPERTIES LINKER_LANGUAGE CXX)


file(GLOB SYSTEMS_MODULES "src/ADCS/Systems/*.cpp")
foreach(module_path ${SYSTEMS_MODULES})
    get_filename_component(module_name ${module_path} NAME_WE)

    add_library(${module_name} SHARED ${module_path})
    target_link_libraries (${module_name} ADCS Eigen3::Eigen)
    set_target_properties(${module_name} PROPERTIES LINKER_LANGUAGE CXX)

endforeach()


option(BUILD_EXAMPLES "Build Example " ON)
if(${BUILD_EXAMPLES})
    add_executable (TestVSCMG  examples/vscmg.cpp )
    target_link_libraries (TestVSCMG VSCMG)

    add_executable (TestRigidBody  examples/rigid_body.cpp )
    target_link_libraries (TestRigidBody RigidBody)    
endif()


