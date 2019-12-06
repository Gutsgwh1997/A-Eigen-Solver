cmake_minimum_required(VERSION 2.8)
project(Test)

set(CMAKE_CXX_FLAGS "-std=c++11")
include_directories("/usr/include/eigen3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(testeigen TestEigen.cpp)
add_executable(matdcom MatDecomposition.cpp)


# list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake")
# find_package( G2O REQUIRED )
# include_directories( ${G2O_INCLUDE_DIRS} )

# add_executable(leastSquaresUseg2o NoLinearLeastSquaresUseg2o.cpp)
# target_link_libraries( leastSquaresUseg2o  g2o_core g2o_stuff)
