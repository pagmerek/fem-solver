add_library(MeshParser MeshParser.cpp MeshParser.h)
add_executable(solver solver.cpp)
target_link_libraries(solver MeshParser)
