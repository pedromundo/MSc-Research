cmake_minimum_required(VERSION 3.0)

project(super_resolution)

find_package(OpenCV REQUIRED core imgproc imgcodecs video)
find_package(OpenMP REQUIRED)

add_executable(${PROJECT_NAME} super_resolution.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
