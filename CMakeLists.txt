cmake_minimum_required(VERSION 3.20)

project(drone_mapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(COMPILE_FLAGS
    -Wall
    -Wextra
    -Werror
    -pedantic
)

find_package(mp-units CONFIG REQUIRED)

file(GLOB_RECURSE DRONE_SOURCES
    "src/drone/*.cpp"
    "src/IO/*.cpp"
    "src/simulation/*.cpp"
)

add_library(drone_core
    ${DRONE_SOURCES}
)

target_compile_features(drone_core PUBLIC cxx_std_20)

target_include_directories(drone_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_options(drone_core
    PRIVATE
        ${COMPILE_FLAGS}
)

target_link_libraries(drone_core
    PUBLIC
        mp-units::mp-units
)

add_executable(drone_mapper src/main.cpp)
target_link_libraries(drone_mapper PRIVATE drone_core)

target_compile_options(drone_mapper
    PRIVATE
        ${COMPILE_FLAGS}
)

enable_testing()

