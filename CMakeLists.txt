cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 20)
project(calibrazione VERSION 0.1.0 LANGUAGES C CXX)


add_executable(velocity_response velocity_response.cpp)
add_executable(regulator regulator.cpp)

add_subdirectory(csvlogger)
add_subdirectory(distance_sensor)
add_subdirectory(meca500_ethercat_cpp)


target_link_libraries(velocity_response PRIVATE csvlogger)
target_compile_options(velocity_response PRIVATE -Wall -pthread)
target_link_libraries(velocity_response PRIVATE pigpio rt)
target_link_libraries(velocity_response PRIVATE meca500_driver)

target_link_libraries(regulator PRIVATE distance_sensor)
target_compile_options(regulator PRIVATE -Wall -pthread)
target_link_libraries(regulator PRIVATE pigpio rt)
target_link_libraries(regulator PRIVATE meca500_driver)
target_link_libraries(regulator PRIVATE csvlogger)

target_compile_features(velocity_response PRIVATE cxx_std_17)
target_compile_features(regulator PRIVATE cxx_std_17)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
