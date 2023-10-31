# Write CMakeLists.txt for this project
cmake_minimum_required(VERSION 3.10)

project(SortTracker VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SOURCES "*.cpp")
add_executable(SortTracker ${SOURCES})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(/usr/local/include)

target_link_libraries(SortTracker PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)