cmake_minimum_required(VERSION 3.0)

project(rotate_align)

find_package(PCL REQUIRED COMPONENTS io common filters registration visualization)

add_executable(${PROJECT_NAME} rotate_align.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES})
