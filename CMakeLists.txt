cmake_minimum_required(VERSION 3.2)

project(detectLine)

set(CMAKE_BUILD_TYPE "Debug")
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

# Eigen
include_directories(/usr/include/eigen3)

include_directories(include)
aux_source_directory(src SRC)


# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main ${SRC})
# 链接OpenCV库
target_link_libraries(main ${OpenCV_LIBS})