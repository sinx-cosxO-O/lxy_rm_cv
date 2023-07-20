cmake_minimum_required(VERSION 3.12)
project(cv)

# set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR "D://Program Files//opencv-4.5.3//opencv-4.5.3_build//install//")

# 寻找 OpenCV 包cv      
find_package(OpenCV REQUIRED)

# 添加源文件
add_executable(cv detector.cpp)

# 链接 OpenCV 库
target_link_libraries(cv ${OpenCV_LIBS})