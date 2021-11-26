cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(astar-pathfinder LANGUAGES CXX)

find_package(Boost REQUIRED COMPONENTS graph)

add_executable(${PROJECT_NAME} main.cpp)

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PRIVATE Boost::boost Boost::graph)