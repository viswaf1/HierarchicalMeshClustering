
# Specify .cpp files
set(TARGET_NAME HierarchicalFaceCluster)
set(SRC DualGraph.cpp DualGraph.h MatVecsMulBatched.h
    MutablePriorityQueue.h Utils.h)

# Build an executable from main.cpp and all specified source files
#add_executable(${TARGET_NAME} main.cpp ${SRC})
#target_link_libraries(${TARGET_NAME} ${LIBRARIES})

# Build a library from all specified source files
# This is required for using googletest
add_library(${TARGET_NAME}-lib ${SRC})
target_link_libraries(${TARGET_NAME}-lib ${LIBRARIES})
set_target_properties(${TARGET_NAME}-lib PROPERTIES OUTPUT_NAME HierarchicalFaceCluster)

# vim:ft=cmake


