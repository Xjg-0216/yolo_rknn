cmake_minimum_required(VERSION 3.10)

project(UAVPositioning)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${CMAKE_SOURCE_DIR}/include)
# 查找源文件
set(SOURCES
    src/main.cpp
    src/uav_position.cpp
)

# 创建可执行文件
add_executable(uav_positioning ${SOURCES})


add_executable(demo src/demo.cpp)