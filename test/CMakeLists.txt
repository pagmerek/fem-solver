
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/f8d7d77c06936315286eb55f8de22cd23c188571.zip
    )

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(
    MeshParserTest
    MeshParserTest.cpp
    )

target_link_libraries(
    MeshParserTest
    MeshParser
    GTest::gtest_main
)

add_executable(
    SolverEigenTest
    SolverEigenTest.cpp
    )

target_link_libraries(
    SolverEigenTest
    SolverFEM
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(MeshParserTest WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/test/")
gtest_discover_tests(SolverEigenTest WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/test/")
