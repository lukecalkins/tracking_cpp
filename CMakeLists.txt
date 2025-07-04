cmake_minimum_required(VERSION 3.31)
project(Tracking)

set(CMAKE_CXX_STANDARD 20)



add_executable(Tracking main.cpp)
add_executable(test_sensor test/test_sensor.cpp)
add_executable(test_params test/test_params.cpp)

target_include_directories (Tracking PUBLIC include)
target_include_directories (test_sensor PUBLIC include)
target_include_directories (test_params PUBLIC include)

find_package(Armadillo CONFIG REQUIRED)
target_link_libraries(Tracking PRIVATE armadillo)
target_link_libraries(test_params PRIVATE armadillo)
target_link_libraries(test_sensor PRIVATE armadillo)
