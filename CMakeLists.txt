cmake_minimum_required(VERSION 3.14)
project(VSCMG LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/bin/ )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/bin/ )
set(CMAKE_ROOT_PATH ${PROJECT_SOURCE_DIR})

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build (Debug, Release etc." FORCE)


option(BUILD_EXAMPLES       "Build Example "        ON)
option(BUILD_PYTHON_LIB "Build Python Wrapper " OFF)

include_directories("include")

if(${BUILD_PYTHON_LIB})
    add_compile_definitions(BUILD_PYTHON_LIB)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(ProjectDeps)

######################################################
# ADCS Core lib
######################################################

file(GLOB ADCS_SRC "include/ADCS/Core/*.h")
add_library(ADCS SHARED ${ADCS_SRC})
target_link_libraries (ADCS Eigen3::Eigen ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
set_target_properties(ADCS PROPERTIES LINKER_LANGUAGE CXX)

######################################################
# ADCS System modules
######################################################

file(GLOB SYSTEMS_MODULES "src/ADCS/Systems/*.cpp")
foreach(module_path ${SYSTEMS_MODULES})
    get_filename_component(module_name ${module_path} NAME_WE)

    add_library(${module_name} SHARED ${module_path} )
    target_link_libraries (${module_name} ADCS Eigen3::Eigen  ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}  ${BOOST_NUMPY_LIBRARY})
    set_target_properties(${module_name} PROPERTIES LINKER_LANGUAGE CXX)
    target_include_directories(${module_name}  PRIVATE ${PYTHON_INCLUDE_DIRS})
endforeach()

######################################################
# Python wrapper library
######################################################
set(CMAKE_SHARED_MODULE_PREFIX "")
add_library(pyadcs MODULE "src/ADCS/pyadcs.cpp")
target_include_directories(pyadcs PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries (pyadcs ADCS Eigen3::Eigen RigidBody VSCMG  ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}  ${BOOST_NUMPY_LIBRARY})
IF (WIN32)
    set_target_properties(pyadcs PROPERTIES SUFFIX .pyd)
ELSE()
    set_target_properties(pyadcs PROPERTIES SUFFIX ".so")
ENDIF()


######################################################
# Executable Samples
######################################################

if(${BUILD_EXAMPLES})
    add_executable (TestVSCMG  examples/vscmg.cpp )
    target_link_libraries (TestVSCMG VSCMG)

    add_executable (TestRigidBody  examples/rigid_body.cpp )
    target_link_libraries (TestRigidBody RigidBody)    
endif()


if(${BUILD_PYTHON_LIB})
    file(GLOB python_samples "python/*.py")
    foreach(sample ${python_samples})
        file(COPY ${sample} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    endforeach()
endif()