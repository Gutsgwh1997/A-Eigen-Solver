cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11")
include_directories("/usr/include/eigen3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(TestEigen TestEigen.cpp)
add_executable(MatDcom MatDecomposition.cpp)

add_executable(LeastSquares NoLinearLeastSquares.cpp)
target_link_libraries(LeastSquares ${PROJECT_NAME}_backend)


add_executable(LeastSquares3 NoLinearLeastSquares3.cpp)
target_link_libraries(LeastSquares3 ${PROJECT_NAME}_backend)
