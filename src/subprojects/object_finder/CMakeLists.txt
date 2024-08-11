cmake_minimum_required(VERSION 3.5)
project(object_finder)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include($ENV{ABCMAKE_PATH})
add_component(${PROJECT_NAME})
target_link_components(${PROJECT_NAME} NAME microlog)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
