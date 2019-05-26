cmake_minimum_required(VERSION 3.0)

project(depth_capture)

find_package(OpenCV REQUIRED core imgproc highgui imgcodecs)
find_package(libfreenect REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(${PROJECT_NAME} depth_capture.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${FREENECT_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenMP_CXX_LIBRARIES})
