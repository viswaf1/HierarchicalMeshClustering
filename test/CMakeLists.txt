include(ExternalProject)

#find_package(glfw REQUIRED)
#include_directories(${GLFW_INCLUDE_DIR})
#set(LIBRARIES ${LIBRARIES} ${GLFW_LIBRARY})

message(STATUS "Libigl include path: ${LIBIGL_INCLUDE_DIR}")
message(STATUS "Libigl external include path: ${LIBIGL_EXTERN_PATH}")
include_directories(${LIBIGL_EXTERN_PATH}/glad/include)
add_subdirectory(${LIBIGL_EXTERN_PATH}/ ${CMAKE_CURRENT_BINARY_DIR}/extern)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -I${LIBIGL_INCLUDE_DIR} ")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -I${LIBIGL_INCLUDE_DIR}")


option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
find_package(LIBIGL REQUIRED QUIET)


set(TARGET HierarchicalFaceCluster-lib)
#set(GTEST_TARGET ${TARGET}_test)
set(TEST_TARGET HFC_test)

add_executable(HFC_test test_hello.cpp)
target_link_libraries(
    HFC_test
    ${TARGET}
    ${LIBRARIES}
    igl::core igl::opengl_glfw
)


# vim:ft=cmake

