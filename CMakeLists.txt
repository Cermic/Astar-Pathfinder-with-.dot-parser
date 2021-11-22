cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(A-Star-Pathfinder LANGUAGES CXX)

file(GLOB_RECURSE sourceFiles "source/*.cpp")

# find_package(SDL2 CONFIG REQUIRED)
# find_package(sdl2-ttf CONFIG REQUIRED)
# find_package(GLEW REQUIRED)
# find_package(glm CONFIG REQUIRED)
find_package(Boost REQUIRED [COMPONENTS <libs>...])

add_executable(${PROJECT_NAME} ${sourceFiles})

# target_link_libraries(${PROJECT_NAME} PRIVATE GLEW::GLEW)
# target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2 SDL2::SDL2main)
# target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2_ttf)
target_link_libraries(main PRIVATE Boost::boost Boost::<lib1> Boost::<lib2> ...)