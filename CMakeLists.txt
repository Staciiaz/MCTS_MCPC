cmake_minimum_required(VERSION 3.17)
project(MCTS_MCPC)

set(CMAKE_CXX_STANDARD 20)
add_executable(solve solve.cpp)
add_executable(test test.cpp)
add_executable(generate generate.cpp)
add_executable(cpc cpc.cpp)
add_executable(test_cpc test_cpc.cpp)