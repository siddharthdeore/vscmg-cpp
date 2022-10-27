cmake_minimum_required(VERSION 3.14)
project(VSCMG 
        LANGUAGES CXX C
        VERSION 0.0.0)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build (Debug, Release etc." FORCE)
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ROOT_PATH ${PROJECT_SOURCE_DIR})

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


option(BUILD_EXAMPLES   "Build Example "        ON)
option(BUILD_PYTHON_LIB "Build Python Wrapper " OFF)

include_directories("include" "Systems/include")

if(${BUILD_PYTHON_LIB})
    add_compile_definitions(BUILD_PYTHON_LIB)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
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
set_target_properties(ADCS PROPERTIES PUBLIC_HEADER
    "include/ADCS/Core/Controllers.h;include/ADCS/Core/IBaseSystem.h;include/ADCS/Core/NumPyArrayData.h;include/ADCS/Core/kinematics.h;include/ADCS/Core/utils.h"
)
install(TARGETS ADCS
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ADCS/Core/"
)

######################################################
# ADCS System modules
######################################################

#set(CMAKE_SHARED_LIBRARY_PREFIX "")
file(GLOB SYSTEMS_MODULES "src/ADCS/Systems/*.cpp")
foreach(module_path ${SYSTEMS_MODULES})
    get_filename_component(module_name ${module_path} NAME_WE)

    add_library(${module_name} SHARED ${module_path} )
    target_link_libraries (${module_name} ADCS Eigen3::Eigen  ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}  ${BOOST_NUMPY_LIBRARY})
    set_target_properties(${module_name} PROPERTIES LINKER_LANGUAGE CXX)
    set_target_properties(${module_name} PROPERTIES PUBLIC_HEADER "include/ADCS/Systems/${module_name}.h")

    if(${BUILD_PYTHON_LIB})
        install(TARGETS ${module_name}
            LIBRARY DESTINATION lib
            PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ADCS/Systems/")
    else()
    install(TARGETS ${module_name} 
            LIBRARY DESTINATION ${Python3_SITELIB}/adcs 
            PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ADCS/Systems/")
    endif()
    #target_include_directories(${module_name} PUBLIC ${PYTHON_INCLUDE_DIRS})
endforeach()

######################################################
# Python wrapper library
######################################################
add_library(Controller MODULE "src/ADCS/py_controller.cpp")
target_include_directories(Controller PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries (Controller Eigen3::Eigen ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}  ${BOOST_NUMPY_LIBRARY})
IF (WIN32)
    set_target_properties(Controller PROPERTIES SUFFIX .pyd)
ELSE()
    set_target_properties(Controller PROPERTIES SUFFIX ".so")
ENDIF()

if(${BUILD_PYTHON_LIB})

    install(TARGETS Controller
        LIBRARY DESTINATION ${Python3_SITELIB}/adcs
    )
    install(FILES __init__.py DESTINATION ${Python3_SITELIB}/adcs)
else()
    install(TARGETS Controller 
            LIBRARY DESTINATION lib
    )
endif()



######################################################
# Executable Samples
######################################################

if(${BUILD_EXAMPLES})
    add_executable (TestVSCMG  examples/cpp/vscmg.cpp )
    target_link_libraries (TestVSCMG VSCMG)

    add_executable (TestRigidBody  examples/cpp/rigid_body.cpp )
    target_link_libraries (TestRigidBody RigidBody)
endif()

######################################################
# copy python samples to binary output directory
######################################################

if(${BUILD_PYTHON_LIB})
    file(GLOB python_samples "examples/python/*.py")
    foreach(sample ${python_samples})
        file(COPY ${sample} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    endforeach()
endif()



# make uninstall
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)

