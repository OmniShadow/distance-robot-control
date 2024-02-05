# distance_sensor/CMakeLists.txt
cmake_minimum_required(VERSION 3.0)
project(distance_sensor)

include_directories(
    include
    include/serial
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(distance_sensor SHARED ${SOURCES})
target_compile_options(distance_sensor PRIVATE -Wall -pthread)
target_link_libraries(distance_sensor PRIVATE pigpio rt)
target_include_directories(distance_sensor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(distance_sensor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)