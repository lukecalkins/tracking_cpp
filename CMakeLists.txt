cmake_minimum_required(VERSION 3.31)
project(Tracking)

set(CMAKE_CXX_STANDARD 20)



add_executable(Tracking main.cpp
        src/KalmanFilter.cpp
        src/UnscentedKalmanFilter.cpp
        src/ISAMFilter.cpp)
add_executable(test_sensor test/test_sensor.cpp)
add_executable(test_params test/test_params.cpp
        src/UnscentedKalmanFilter.cpp
        src/KalmanFilter.cpp
        src/ISAMFilter.cpp)
add_executable(test_target test/test_target.cpp)
add_executable(isam_example sketch/isam_example_gpt.cpp)
add_executable(isam_example2 sketch/isam_example2_gpt.cpp)

target_include_directories (Tracking PUBLIC include)
target_include_directories (test_sensor PUBLIC include)
target_include_directories (test_params PUBLIC include)
target_include_directories (test_target PUBLIC include)

find_package(Armadillo CONFIG REQUIRED)
find_package(gtsam CONFIG REQUIRED)
target_link_libraries(Tracking
        PRIVATE
        armadillo
        gtsam)
target_link_libraries(test_params PRIVATE armadillo gtsam)
target_link_libraries(test_sensor PRIVATE armadillo)
target_link_libraries(test_target PRIVATE armadillo)
target_link_libraries(isam_example PRIVATE gtsam)
target_link_libraries(isam_example2 PRIVATE gtsam)


