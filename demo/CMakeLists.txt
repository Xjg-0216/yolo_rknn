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
include_directories(${CMAKE_SOURCE_DIR}/include
"/usr/include/eigen3"
)

# 查找源文件
set(SOURCES
    src/demo_geolocation.cpp
    src/drone_objlocation.cpp
)

# 创建可执行文件
add_executable(demo_geolocation ${SOURCES})
target_link_libraries(demo_geolocation ${DroneObjlocation_LIBS})

