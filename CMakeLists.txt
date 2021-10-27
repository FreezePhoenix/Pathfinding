cmake_minimum_required (VERSION 3.5)
project ("Pathfinding")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3 -Wall -g -fPIC")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(../../lib/TriangleManipulator ./build/TriangleManipulator EXCLUDE_FROM_ALL)
add_subdirectory(../../lib/spdlog ./build/spdlog EXCLUDE_FROM_ALL)

add_library(spdlog_HEADERS ALIAS spdlog_header_only)

add_library(Pathfinding_HEADERS INTERFACE)

target_include_directories(Pathfinding_HEADERS INTERFACE include ../../include)
target_link_libraries(Pathfinding_HEADERS INTERFACE TriangleManipulator_HEADERS spdlog_HEADERS)

add_library(Pathfinding SHARED
  "src/Service.cpp"
)

add_library(Objectifier STATIC
  "src/Objectifier.cpp"
)

add_library(Writer STATIC
  "src/Writer.cpp"
)

target_link_libraries(Writer PUBLIC Pathfinding_HEADERS)
target_link_libraries(Objectifier PUBLIC Pathfinding_HEADERS)

target_link_libraries(Pathfinding PUBLIC Pathfinding_HEADERS Writer Objectifier)

set_property(TARGET Pathfinding PROPERTY COMPILE_OPTIONS "-fPIC")
set_property(TARGET Objectifier PROPERTY COMPILE_OPTIONS "-fPIC")
set_property(TARGET Writer PROPERTY COMPILE_OPTIONS "-fPIC")