include_directories(include)

add_subdirectory(solvers)
add_subdirectory(utils)

add_executable(beam beam.cpp)

target_link_libraries(beam Mesh)
target_link_libraries(beam SolverFEM)
target_link_libraries(beam OnelabTools)
