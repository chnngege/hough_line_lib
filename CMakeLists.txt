cmake_minimum_required(VERSION 3.16)
project(hough_line_lib)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR E:/Software/OpenCV4.3.0/install)
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
message(${OpenCV_INCLUDE_DIRS})
message(${OpenCV_LIBS})

add_library(hough_line_lib library.cpp library.h main.cpp)

add_executable(main main.cpp)
#target_link_libraries(hough_line_lib ${OpenCV_LIBS} )
target_link_libraries(main hough_line_lib ${OpenCV_LIBS})