cmake_minimum_required (VERSION 3.5)
project ("Pathfinding")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

add_subdirectory(lib/clipper-lib ./build/clipper-lib EXCLUDE_FROM_ALL)
set_property(TARGET clipper_static PROPERTY COMPILE_OPTIONS -Duse_int32)

add_subdirectory(../../lib/TriangleManipulator ./build/TriangleManipulator EXCLUDE_FROM_ALL)
add_subdirectory(../../lib/spdlog ./build/spdlog EXCLUDE_FROM_ALL)

add_library(spdlog_HEADERS ALIAS spdlog_header_only)

add_library(Pathfinding_HEADERS INTERFACE)

target_include_directories(Pathfinding_HEADERS INTERFACE include ../../include)
target_link_libraries(Pathfinding_HEADERS INTERFACE TriangleManipulator_HEADERS spdlog_HEADERS clipper_static)

add_library(Objectifier OBJECT
  "src/Objectifier.cpp"
)

add_library(Pather OBJECT
  "src/Pather.cpp"
)

add_library(Pathfinding SHARED
  "src/Service.cpp"
)

target_link_libraries(Pather PUBLIC Pathfinding_HEADERS)
target_link_libraries(Objectifier PUBLIC Pathfinding_HEADERS)

target_link_libraries(Pathfinding PUBLIC Pathfinding_HEADERS Objectifier Pather)
