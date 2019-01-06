
cmake_minimum_required(VERSION 2.8)

project(HierarchicalFaceCluster)
set(PROJECT_VERSION 0.0.0.0)
set(PROJECT_BRIEF "\"No description\"")

set(LIBIGL_INCLUDE_DIR "" CACHE STRING "Path for libigl include directory")
set(LIBIGL_EXTERN_PATH "" CACHE STRING "Path for libigl external include directory")

# Enable/disable testing
option(TESTS_ENABLED "Enable automatic tests" ON)
# Enable/disable test coverage
option(COVERAGE_ENABLED "Enable test coverage" OFF)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")

include_directories(${CMAKE_SOURCE_DIR}/src)

if(COVERAGE_ENABLED)
    set(CMAKE_CXX_FLAGS_DEBUG "-Wall -fPIC -O0 -ggdb -fprofile-arcs -ftest-coverage")
else()
    set(CMAKE_CXX_FLAGS_DEBUG "-Wall -O0 -ggdb")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "Compiler does not support C++11")
endif()

find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(docs
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, defaulting to Debug")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Boost, a collection of useful C++ utilities and idioms
#find_package(Boost COMPONENTS
#    program_options
#    filesystem
#    thread
#    system
#    REQUIRED)
#include_directories(${Boost_INCLUDE_DIR})
#set(LIBRARIES ${LIBRARIES} ${Boost_LIBRARIES})
set(LIBRARIES ${LIBRARIES} pthread)

# ROOT, a library for HEP data analysis (contains RooFit)
#find_package(ROOT REQUIRED)
#include_directories(${ROOT_INCLUDE_DIR})
#set(LIBRARIES ${LIBRARIES} ${ROOT_LIBRARIES})
#set(LIBRARIES ${LIBRARIES} ${ROOFIT_LIBRARIES})
 
# Eigen3, a template library for high performance linear algebra
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# glog, a powerful logging framework
#find_package(Glog REQUIRED)
#include_directories(${GLOG_INCLUDE_DIRS})
#set(LIBRARIES ${LIBRARIES} ${GLOG_LIBRARIES})



add_subdirectory(src)
add_subdirectory(test)

# vim:ft=cmake

