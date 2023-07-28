cmake_minimum_required (VERSION 3.5)
project ("Pathfinding")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")

set(SPDLOG_USE_STD_FORMAT ON)

add_subdirectory(lib/clipper-lib ./build/clipper-lib EXCLUDE_FROM_ALL)
set_property(TARGET clipper_static PROPERTY COMPILE_OPTIONS -Duse_int32)

add_subdirectory(../../lib/TriangleManipulator ./build/TriangleManipulator EXCLUDE_FROM_ALL)
add_subdirectory(../../lib/spdlog ./build/spdlog EXCLUDE_FROM_ALL)

add_library(Pathfinding SHARED
  "src/Objectifier.cpp"
  "src/Pather.cpp"
  "src/Service.cpp"
)

include_directories(include ../../include)

target_link_libraries(
    Pathfinding 
  PUBLIC 
    TriangleManipulator_HEADERS
    spdlog_header_only
    clipper_static
)
