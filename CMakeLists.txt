cmake_minimum_required(VERSION 3.12)
project(cv)

# set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR "D://Program Files//opencv-4.5.3//opencv-4.5.3_build//install//")

# 寻找 OpenCV 包cv      
find_package(OpenCV REQUIRED)

# 指定当前目录下的源文件，保存到<ALL_SRCS>变量中
aux_source_directory(. ALL_SRCS)

# 制定生成目标
add_executable(cv ${ALL_SRCS})

# 链接 OpenCV 库
target_link_libraries(cv ${OpenCV_LIBS})