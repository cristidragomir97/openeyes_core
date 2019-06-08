cmake_minimum_required(VERSION 3.10)
project(openeyes_core)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 14)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


add_executable(openeyes_core main.cpp serial.cpp serial.h)
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})