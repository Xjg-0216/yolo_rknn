cmake_minimum_required(VERSION 3.10)

project(UAVPositioning)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 默认设置为 Debug 模式
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# 添加调试信息
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# 包含头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 查找源文件
set(SOURCES
    src/main.cpp
    src/uav_position.cpp
)

# 创建可执行文件
add_executable(uav_positioning ${SOURCES})

# 为调试目标添加宏定义
target_compile_definitions(uav_positioning PRIVATE ENABLE_DEBUG)

# 添加另一个可执行目标
add_executable(demo src/demo.cpp)
target_compile_definitions(demo PRIVATE ENABLE_DEBUG)
