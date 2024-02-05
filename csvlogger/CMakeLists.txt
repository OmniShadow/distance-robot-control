# csvlogger/CMakeLists.txt
set(CMAKE_CXX_STANDARD 17)
add_library(csvlogger STATIC
    CsvLogger.cpp
    CsvLogger.hpp
)

target_include_directories(csvlogger PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})